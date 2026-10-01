#pragma once

struct MeasurementState
{
	double timeSinceLastMeasurement; // seconds
	double velocity;				 // m/s
	double omega_encoder;			 // rad/s
	double omega_gyro;				 // rad/s
};