#include "Car/Car.h"

Car::Car()
    : leftMotor(L_MOTOR_PLUS_PIN, L_MOTOR_MINUS_PIN, L_MOTOR_SPEED_PIN),
      rightMotor(R_MOTOR_PLUS_PIN, R_MOTOR_MINUS_PIN, R_MOTOR_SPEED_PIN),
      leftEncoder(L_ENCODER_PHASE_A_PIN, L_ENCODER_PHASE_B_PIN),
      rightEncoder(R_ENCODER_PHASE_A_PIN, R_ENCODER_PHASE_B_PIN)
{
}

Car &Car::getInstance()
{
	// the first time this is called, instance is created and then never created again after that
	// every time after that the same object is returned when this function is called
	static Car instance;
	return instance;
}

CarMotionState Car::getCarMotionState()
{
	CarMotionState state;
	double leftWheelVelocity = -leftEncoder.getVelocity();	// left motor .getCount decreases as the car moves forward
	double rightWheelVelocity = rightEncoder.getVelocity(); // right motor .getCount increase as the car moves forward

	double forwardVel = (rightWheelVelocity + leftWheelVelocity) / 2;		  // m/s
	double turnRate = (rightWheelVelocity - leftWheelVelocity) / TRACK_WIDTH; // rad/s

	state.forwardVelocity = forwardVel;
	state.turnRate = turnRate;
	state.leftWheelVelocity = leftWheelVelocity;
	state.rightWheelVelocity = rightWheelVelocity;
	return state;
}

void Car::control(double leftStickY, double rightStickX)
{
	leftMotor.control(leftStickY, rightStickX);
	rightMotor.control(leftStickY, -rightStickX); // negative right stick to account for chirality differences for the motors
}

void Car::stop()
{
	leftMotor.stop();
	rightMotor.stop();
}

void Car::throttleToggle()
{
	leftMotor.throttleToggle();
	rightMotor.throttleToggle();
}