#pragma once

#include "Arduino.h"
#include "Motor/Motor.h"
#include "Encoder/Encoder.h"
#include "CarMotionState.h"

class Car
{
public:
	Car(uint8_t leftPlusPin, uint8_t leftMinusPin, uint8_t leftSpeedPin,
		uint8_t rightPlusPin, uint8_t rightMinusPin, uint8_t rightSpeedPin,
		uint8_t leftEncoderPhaseAPin, uint8_t leftEncoderPhaseBPin,
		uint8_t rightEncoderPhaseAPin, uint8_t rightEncoderPhaseBPin);
	CarMotionState getCarMotionState();
	void control(double leftStickY, double rightStickX);
	void stop();
	void throttleToggle();

private:
	static constexpr double TRACK_WIDTH = 0.145; // meters
	Motor leftMotor;
	Motor rightMotor;
	Encoder leftEncoder;
	Encoder rightEncoder;
};