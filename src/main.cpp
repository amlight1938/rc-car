#include <Arduino.h>
#include "Encoder/Encoder.h"
#include "Imu/Imu.h"
#include "Motor/Motor.h"
#include "Pins.h"
#include "Car/Car.h"
#include "Controller/Controller.h"
#include "Controller/ControllerState.h"
#include "logging/Logger.h"

// the purpose of this task is to replace arduino's default loop() with a FreeRTOS task for multiple reasons:
// Deterministic timing - allows us to execute on a guarantee schedule wheras delay() or millis() scheduing drifts
// Priority - this allows us to assign priority to each task (logging, control)
// Specify core use - specify which core (0,1) to run this task on so we can separate it from bluetooth task
void controlTask(void *params)
{
	Logger& logger = Logger::getInstance();

	// instantiating the bluetooth controller and motors
	Controller controller;

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

	logger.log(Logger::formatLog("[%lu]-init complete", millis()/1000));

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

	// set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
	TickType_t lastWakeTime = xTaskGetTickCount();

	CarMotionState motion;
	ImuReading imuReading;

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
			logger.log(Logger::formatLog("[%lu]-current state: PAIRING", millis()/1000));

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
			logger.log(Logger::formatLog("[%lu]-current state: DISARMED", millis()/1000));

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
			// turning the bluetooth LED ON and the standby LED O when entering the state
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
			
			motion = car.getCarMotionState();

			logger.log(Logger::formatLog("[%lu]-Velocity: %.2f Turn rate: %.2f", millis()/1000, motion.forwardVelocity, motion.turnRate));
			// Serial.printf("Velocity: %.2f", motion.forwardVelocity);
			// Serial.printf("  Turn rate: %.2f", motion.turnRate);
			// Serial.println();
			
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

			// imu readout testing
			imuReading = imu.getImuReading();

			logger.log(Logger::formatLog("[%lu]-R: %.0f P: %.0f Y: %.0f GX: %.0f GY: %.0f GZ: %.0f", 
				millis()/1000, imuReading.roll, imuReading.pitch, imuReading.yaw, imuReading.gyroX, imuReading.gyroY, imuReading.gyroZ));

			break;

		case CALIBRATION:
			logger.log(Logger::formatLog("[%lu]-current state: CALIBRATION", millis()/1000));
				
			// turning the both motors off when entering the state
			car.stop();

			// turning the bluetooth LED and Standby  off when calbrating
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

		// schedule when the task should wake up and run another loop
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(50));
	}
}

void setup()
{
	Serial.begin(115200);
	Logger::getInstance().createTask();
	// create and run the task
	// task, human readable name, stack size allocation, input parameters, priority, returned task handle, core ID
	xTaskCreatePinnedToCore(controlTask, "controlTask", 8000, NULL, 2, NULL, 0);
}

void loop()
{
	// kill the arduino default loop task since we are running a deterministic time scheduler task
	vTaskDelete(NULL);
}
