#include <Arduino.h>
#include "Pins.h"
#include "Car/Car.h"
#include "Controller/Controller.h"
#include "EKF/KalmanFilter.h"
#include "Encoder/Encoder.h"
#include "Imu/Imu.h"
#include "Logging/Logger.h"
#include "Motor/Motor.h"

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
		IMU_CALIBRATION,
		BIAS_CALIBRATION
	};

	State currentState = PAIRING; // start in pairing mode
	State lastState;

	// initializing our measurement variables
	ImuReading imuReading;
	CarMotionState motion;
	MeasurementState measurementState;
	KalmanFilter ekf;
	Vector<double, 6> pose;

	// set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
	TickType_t lastWakeTime = xTaskGetTickCount();

	// setting up the lambda function for getting the pose
	// this is expected to be used multiple times in the switch-case
	auto getPose = [&]() -> Vector<double, 6>
	{
		imuReading = imu.getImuReading();
		motion = car.getCarMotionState();
		measurementState.timeSinceLastMeasurement = imuReading.timeSinceLastMeasurement;
		measurementState.velocity = motion.forwardVelocity;
		measurementState.acceleration = imuReading.accX;
		measurementState.omega_encoder = motion.turnRate;
		measurementState.omega_gyro = imuReading.gyroZ;
		ekf.iterate(measurementState);
		return ekf.getState();
	};

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
			// stopping the motors if the controller disconnects
			if (lastState != PAIRING)
			{
				lastState = PAIRING;
				stbyState = LOW;
				digitalWrite(STBY_PIN, stbyState);
				car.stop();
			}

			// leaving the pairing state when a controller connects
			if (controllerState.connected)
			{
				currentState = DISARMED;
				break;
			}

			// logging the current state while waiting to pair to a controller
			logger.log(true, "[%.1f]-current state: PAIRING", millis() / 1000.0);

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
			// turning the bluetooth LED ON and the standby LED OFF when entering the state
			if (lastState != DISARMED)
			{
				lastState = DISARMED;
				btLedState = HIGH;
				digitalWrite(BT_LED_PIN, btLedState);
				stbyState = LOW;
				digitalWrite(STBY_PIN, stbyState);
				car.stop();
			}

			// switching to MANUAL based on controller inputs
			if (controllerState.leftBumper)
			{
				currentState = MANUAL; // always go straight to manual once stby button is toggled on
				break;
			}

			// switching to IMU_CALIBRATION based on controller inputs
			if (controllerState.dpadDown && controllerState.buttonA)
			{
				currentState = IMU_CALIBRATION;
				break;
			}

			// switching to BIAS_CALIBRATION based on controller inputs
			if (controllerState.dpadUp && controllerState.buttonA)
			{
				currentState = BIAS_CALIBRATION;
				break;
			}

			// logging the current state while waiting in the disarmed state
			logger.log(true, "[%.1f]-current state: DISARMED", millis() / 1000.0);

			break;

		case MANUAL:
			// turning the bluetooth LED ON and the standby LED ON when entering the state
			if (lastState != MANUAL)
			{
				lastState = MANUAL;
				stbyState = HIGH;
				digitalWrite(STBY_PIN, stbyState);
				// need to turn the btLED on (at least for the time being for indication)
				btLedState = HIGH;
				digitalWrite(BT_LED_PIN, btLedState);

				// restarting the EKF without having to restart car
				ekf = KalmanFilter();
			}

			// switching to DISARMED based on controller inputs
			if (controllerState.leftBumper)
			{
				currentState = DISARMED;
				break;
			}

			// switching to AUTONOMOUS based on controller inputs
			if (controllerState.buttonY)
			{
				currentState = AUTONOMOUS;
				break;
			}

			// capping the motor throttle based on controller inputs
			if (controllerState.buttonX)
			{
				car.throttleToggle();
			}

			// powering the motors with manual control
			car.control(controllerState.leftStickY, controllerState.rightStickX);

			// getting the measurements and updating the EKF using the lambda function
			pose = getPose();

			// logging data for troubleshooting
			// logger.log(true, "x: %-7.3f y: %-7.3f v: %-7.3f a: %-7.3f theta: %-7.3f w: %-7.3f we: %-7.3f wg: %-7.3f",
			// 		   pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], measurementState.omega_encoder, measurementState.omega_gyro);


			break;

		case AUTONOMOUS:
			// turning the both motors off when entering the state
			if (lastState != AUTONOMOUS)
			{
				lastState = AUTONOMOUS;
				car.stop();
			}

			// switching to DISARMED based on controller inputs
			if (controllerState.leftBumper)
			{
				currentState = DISARMED;
				break;
			}

			// switching to MANUAL based on controller inputs
			if (controllerState.buttonY)
			{
				currentState = MANUAL;
				break;
			}

			// causing the bluetooth LED to blink quickly while in AUTONOMOUS mode
			// blinking will occur at a frequency based on autonomousBlinkDuration
			if (currentTime - lastBlinkSwitch > autonomousBlinkDuration)
			{
				lastBlinkSwitch = currentTime;
				digitalWrite(BT_LED_PIN, btLedState);
				btLedState = !btLedState;
			}

			// future autonomous capabilities go here

			// getting the measurements and updating the EKF using the lambda function
			pose = getPose();

			break;

		case IMU_CALIBRATION:
			// turning the both motors off when entering the state
			// this only occurs once as calibration causes the controlTask to wait and only calibrate once
			if (lastState != IMU_CALIBRATION)
			{
				lastState = IMU_CALIBRATION;
				car.stop();

				// logging the current state while waiting performing the imu calibration
				logger.log(true, "[%.1f]-current state: IMU_CALIBRATION", millis() / 1000.0);

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
				currentState = DISARMED;
			}

			break;

		case BIAS_CALIBRATION:
			// turning the both motors off when entering the state
			// this only occurs once as calibration causes the controlTask to wait and only calibrate once
			if (lastState != BIAS_CALIBRATION)
			{
				lastState = BIAS_CALIBRATION;
				car.stop();

				// logging the current state while waiting performing the bias calibration
				logger.log(true, "[%.1f]-current state: BIAS_CALIBRATION", millis() / 1000.0);

				// turning the bluetooth LED and Standby off when calbrating
				btLedState = LOW;
				digitalWrite(BT_LED_PIN, btLedState);
				stbyState = LOW;
				digitalWrite(STBY_PIN, stbyState);

				// calibrating the IMUs Accelerometer and Gyroscope
				imu.calibrateImuAccelGyroBias(CAL_LED_PIN);

				// returning to DISARMED after calibration is complete
				currentState = DISARMED;
			}

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
