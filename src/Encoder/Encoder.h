#pragma once

#include "Arduino.h"
#include <ESP32Encoder.h>
#include <deque>

// Calculated values based on data recording variances_calculation.py
// All values shown are in (rad/s)^2
// Var(residual : encoder - gyroZ) = Var(encoder_noise) + Var(gyro_noise)
// R_encoder (gradual) ≈ 0.07376728721366735 − 0.0000008036 ≈ 0.0737665
// R_encoder (intense) ≈ 9.815891440094012 − 0.0000008036 ≈ 9.8158906
// R_encoder (mixed)   ≈ 4.00207743375046 − 0.0000008036 ≈ 4.0020766

class Encoder
{
public:
	Encoder(uint8_t phaseAPin, uint8_t phaseBPin);

	// gets the updated average velocity of the motor for the previous DEQUE_MAX_SIZE measurements
	// based on 70mm diameter wheels
	double getVelocity(); // m/s

private:
	// motor tests indicate 2772 triggers/axle rotation
	// wheel diameter is 0.070m, therefore 0.2199m circumference
	// 2772 triggers/0.2199m or 12605 (trig/m)
	static constexpr double TRIGGERS_PER_METER = 12605.0;

	ESP32Encoder encoder;
	int64_t prevMotorCount;

	unsigned long lastMeasurementTime;
	double triggerRate;

	std::deque<double> rollingAverageDeque;
	const uint8_t DEQUE_MAX_SIZE = 5;
	double currentDequeSum;
};