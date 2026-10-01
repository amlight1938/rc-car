#pragma once

struct MeasurementState
{
	double timeSinceLastMeasurement;
	double velocity;
	double omega_encoder;
	double omega_gyro;
};