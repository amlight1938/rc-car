#pragma once

#include "Arduino.h"
#include "Imu/ImuReading.h"
#include "Imu/MPU9250/MPU9250.h" //using ours because were changing some things in file. make sure to include with "" not <>
#include "Logging/Logger.h"

// Calculated values based on data recording variances_calculation.py
// Variances:
// GyroZ    8.035852e-07 rad^2/s^2
// Biases:
// GyroZ   -0.012497 rad/s

class Imu
{
public:
	Imu();

	void print(ImuReading imuState);

	void calibrateImuAccelGyro();

	void calibrateImuMag(uint8_t ledPin);

	ImuReading getImuReading();

private:
	MPU9250 imu;
	unsigned long lastMeasurementTime;
	Logger &logger;
	const double GYRO_Z_BIAS = -0.012497; // rad/s calculated with variance_calculation.py, possible value could change with calibration
};