#include <Arduino.h>
#include "EKF/KalmanFilter.h"
#include "Encoder/Encoder.h"
#include "Imu/Imu.h"
#include "Motor/Motor.h"
#include "Pins.h"
#include "Car/Car.h"
#include "Controller/Controller.h"
#include "Controller/ControllerState.h"
#include "Logging/Logger.h"

// the purpose of this task is to replace arduino's default loop() with a FreeRTOS task for multiple reasons:
// Deterministic timing - allows us to execute on a guarantee schedule wheras delay() or millis() scheduing drifts
// Priority - this allows us to assign priority to each task (logging, control)
// Specify core use - specify which core (0,1) to run this task on so we can separate it from bluetooth task
void controlTask(void *params)
{
	// getting a reference to the logger singleton
	Logger &logger = Logger::getInstance();

	// setting up pins as Inputs or Outputs
	pinSetup();

	// instantiating the bluetooth controller and motors
	Controller controller;

	// instantiating the car
	Car car(L_MOTOR_PLUS_PIN, L_MOTOR_MINUS_PIN, L_MOTOR_SPEED_PIN,
			R_MOTOR_PLUS_PIN, R_MOTOR_MINUS_PIN, R_MOTOR_SPEED_PIN,
			L_ENCODER_PHASE_A_PIN, L_ENCODER_PHASE_B_PIN,
			R_ENCODER_PHASE_A_PIN, R_ENCODER_PHASE_B_PIN);

	// instantiating the IMU
	Imu imu;

	// setting up the standby state for the motor controller
	bool stbyState = LOW;

	// setting up the blinking LED for the period the esp32 is searching for a bluetooth controller
	unsigned long pairingBlinkDuration = 500;	 // 500 ms on, 500 off
	unsigned long autonomousBlinkDuration = 200; // 200 ms on, 200 off
	bool btLedState = LOW;
	unsigned long lastBlinkSwitch = 0;

	// logging that the initialization process has finished at the time this occured
	logger.log("[%.1f]-init complete", millis() / 1000.0);

	// list of states the car can enter
	enum State
	{
		PAIRING,
		DISARMED,
		MANUAL,
		AUTONOMOUS,
		CALIBRATION
	};

	State currentState = PAIRING; // start in pairing mode
	State lastState;

	CarMotionState motion;
	ImuReading imuReading;

	// set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
	TickType_t lastWakeTime = xTaskGetTickCount();

	while (true)
	{
		unsigned long currentTime = millis();

		ControllerState controllerState = controller.getControllerState();

		// enter pairing mode if controller disconnects
		if (!controllerState.connected)
		{
			currentState = PAIRING;
		}

		// performing unique actions based on the cars state
		switch (currentState)
		{
		case PAIRING:
			// logging the current state while waiting to pair to a controller
			logger.log(true, "[%.1f]-current state: PAIRING", millis() / 1000.0);

			// leaving the pairing state when a controller connects
			if (controllerState.connected)
			{
				lastState = currentState;
				currentState = DISARMED;
				break;
			}

			// stopping the motors if the controller disconnects
			if (lastState != PAIRING)
			{
				stbyState = LOW;
				digitalWrite(STBY_PIN, stbyState);
				car.stop();
			}

			// causing the bluetooth LED to blink while searching for a controller
			// blinking will occur at a frequency based on pairingBlinkDuration
			if (currentTime - lastBlinkSwitch > pairingBlinkDuration)
			{
				lastBlinkSwitch = currentTime;
				digitalWrite(BT_LED_PIN, btLedState);
				btLedState = !btLedState;
			}

			break;

		case DISARMED:
			// logging the current state while waiting in the disarmed state
			logger.log(true, "[%.1f]-current state: DISARMED", millis() / 1000.0);

			// turning the bluetooth LED ON and the standby LED OFF when entering the state
			if (lastState != DISARMED)
			{
				btLedState = HIGH;
				digitalWrite(BT_LED_PIN, btLedState);
				stbyState = LOW;
				digitalWrite(STBY_PIN, stbyState);
				car.stop();
			}

			// switching to MANUAL based on controller inputs
			if (controllerState.leftBumper)
			{
				lastState = currentState;
				currentState = MANUAL; // always go straight to manual once stby button is toggled on
				break;
			}

			// switching to CALIBRATION based on controller inputs
			if (controllerState.dpadDown && controllerState.buttonA)
			{
				lastState = currentState;
				currentState = CALIBRATION;
				break;
			}

			break;

		case MANUAL:
			// turning the bluetooth LED ON and the standby LED ON when entering the state
			if (lastState != MANUAL)
			{
				stbyState = HIGH;
				digitalWrite(STBY_PIN, stbyState);
				// need to turn the btLED on (at least for the time being for indication)
				btLedState = HIGH;
				digitalWrite(BT_LED_PIN, btLedState);
			}

			// switching to DISARMED based on controller inputs
			if (controllerState.leftBumper)
			{
				lastState = currentState;
				currentState = DISARMED;
				break;
			}

			// switching to AUTONOMOUS based on controller inputs
			if (controllerState.buttonY)
			{
				lastState = currentState;
				currentState = AUTONOMOUS;
				break;
			}

			// powering the motors with manual control
			car.control(controllerState.leftStickY, controllerState.rightStickX);

			// getting the measurements
			imuReading = imu.getImuReading();
			motion = car.getCarMotionState();

			logger.log(true, "[%6.1f] - V: %-7.2f LW: %-7.2f RW: %-7.2f w: %-7.2f GZ: %-7.2f", 
    			millis() / 1000.0, motion.forwardVelocity, motion.leftWheelVelocity, motion.rightWheelVelocity, motion.turnRate, imuReading.gyroZ);
			Serial.printf("[%6.1f] - V: %-7.2f LW: %-7.2f RW: %-7.2f w: %-7.2f GZ: %-7.2f", 
				millis() / 1000.0, motion.forwardVelocity, motion.leftWheelVelocity, motion.rightWheelVelocity, motion.turnRate, imuReading.gyroZ);
			Serial.println();
			break;

		case AUTONOMOUS:
			// turning the both motors off when entering the state
			if (lastState != AUTONOMOUS)
			{
				car.stop();
			}

			// causing the bluetooth LED to blink quickly while in AUTONOMOUS mode
			// blinking will occur at a frequency based on autonomousBlinkDuration
			if (currentTime - lastBlinkSwitch > autonomousBlinkDuration)
			{
				lastBlinkSwitch = currentTime;
				digitalWrite(BT_LED_PIN, btLedState);
				btLedState = !btLedState;
			}

			// switching to DISARMED based on controller inputs
			if (controllerState.leftBumper)
			{
				lastState = currentState;
				currentState = DISARMED;
				break;
			}

			// switching to MANUAL based on controller inputs
			if (controllerState.buttonY)
			{
				lastState = currentState;
				currentState = MANUAL;
				break;
			}

			// future autonomous capabilities go here


			// getting the cars imu reading logging and performing calculations
			imuReading = imu.getImuReading();

			// imu readout testing
			imu.print(imuReading);

			// logging the imu's heading and gyro readings
			logger.log(true, "R:%-3.0f P:%-3.0f Y:%-3.0f GX:%-3.0f GY:%-3.0f GZ:%-3.0f Ax:%-4.1f Ay:%-4.1f Az:%-4.1f",
										 imuReading.roll, imuReading.pitch, imuReading.yaw, imuReading.gyroX, imuReading.gyroY, imuReading.gyroZ, 
										 imuReading.accX, imuReading.accY, imuReading.accZ);

			break;

		case CALIBRATION:
			// logging the current state while waiting performing the imu calibration
			logger.log(true, "[%.1f]-current state: CALIBRATION", millis() / 1000.0);

			// turning the both motors off when entering the state
			// this only occurs once as calibration causes the controlTask to wait and only calibrate once
			if (lastState != CALIBRATION)
			{
				car.stop();
			}

			// turning the bluetooth LED and Standby off when calbrating
			btLedState = LOW;
			digitalWrite(BT_LED_PIN, btLedState);
			stbyState = LOW;
			digitalWrite(STBY_PIN, stbyState);

			// calibrating the IMUs Accelerometer and Gyroscope
			imu.calibrateImuAccelGyro();

			// calibrating the IMUs Magnetometer
			imu.calibrateImuMag(CAL_LED_PIN);

			// returning to DISARMED after calibration is complete
			lastState = currentState;
			currentState = DISARMED;
			break;
		}

		// increaseing the rate limit counter so the logger only operates every 10th cycle
		logger.incrementRateLimitCounter();

		// schedule when the task should wake up and run another loop
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
	}
}

void setup()
{
	Serial.begin(115200);
	// creating the logger singleton and starting its task to send data
	Logger::getInstance().createTask();

	// creating the control task to control the car and manage the cars state
	// task, human readable name, stack size allocation, input parameters, priority, returned task handle, core ID
	xTaskCreatePinnedToCore(controlTask, "controlTask", 8000, NULL, 2, NULL, 0);
}

void loop()
{
	// kill the arduino default loop task since we are running a deterministic time scheduler task
	vTaskDelete(NULL);
}
