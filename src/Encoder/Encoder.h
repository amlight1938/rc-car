#pragma once

#include "Arduino.h"
#include <ESP32Encoder.h>
#include <deque> 

class Encoder
{
public:
	Encoder(uint8_t phaseAPin, uint8_t phaseBPin);

	// gets the velocity of the motor with 70mm diameter wheels since the previous getVelocity request in meters/second
	double getVelocity();

private:
	// motor tests indicate 2772 triggers/axle rotation
	// wheel diameter is 0.070m, therefore 0.2199m circumference
	// 2772 triggers/0.2199m or 12605 (trig/m)
	static constexpr double TRIGGERS_PER_METER = 12605.0;
	ESP32Encoder encoder;
	int64_t prevMotorCount;
	double triggerRate;
	double velocity;
	unsigned long lastMeasurementTime;
	std::deque<double> rollingAverageDeque;
	const uint8_t DEQUE_MAX_SIZE = 5;
	double currentDequeSum;
};