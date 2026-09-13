#include "Car/Car.h"

Car::Car(uint8_t leftPlusPin, uint8_t leftMinusPin, uint8_t leftSpeedPin,
		 uint8_t rightPlusPin, uint8_t rightMinusPin, uint8_t rightSpeedPin,
		 uint8_t leftEncoderPhaseAPin, uint8_t leftEncoderPhaseBPin,
		 uint8_t rightEncoderPhaseAPin, uint8_t rightEncoderPhaseBPin)
	: leftMotor(leftPlusPin, leftMinusPin, leftSpeedPin),
	  rightMotor(rightPlusPin, rightMinusPin, rightSpeedPin),
	  leftEncoder(leftEncoderPhaseAPin, leftEncoderPhaseBPin),
	  rightEncoder(rightEncoderPhaseAPin, rightEncoderPhaseBPin)
{
}

CarMotionState Car::getCarMotionState()
{
	double leftWheelVelocity = -leftEncoder.getVelocity();	// left motor .getCount decreases as the car moves forward
	double rightWheelVelocity = rightEncoder.getVelocity(); // right motor .getCount increase as the car moves forward

	double forwardVel = (rightWheelVelocity + leftWheelVelocity) / 2;
	double turnRate = (rightWheelVelocity - leftWheelVelocity) / TRACK_WIDTH;
	return {forwardVel, turnRate};
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