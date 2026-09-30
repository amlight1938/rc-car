#pragma once

#include "Arduino.h"
#include "Imu/ImuReading.h"
#include "Imu/MPU9250/MPU9250.h" //using ours because were changing some things in file. make sure to include with "" not <>
#include "Logging/Logger.h"

// Calculated values based on data recording variances_calculation.py
// Variances:
// GyroZ    0.003105
// dtype: float64
// Biases:
// GyroZ   -0.724126


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
};