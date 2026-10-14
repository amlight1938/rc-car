#pragma once

struct MeasurementState
{
	double timeSinceLastMeasurement; // seconds
	double velocity;				 // m/s
	double acceleration;			 // m/s^2
	double omega_encoder;			 // rad/s
	double omega_gyro;				 // rad/s
};