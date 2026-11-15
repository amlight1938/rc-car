#pragma once

#include "Arduino.h"
#include "Imu/ImuReading.h"
#include "Imu/MPU9250/MPU9250.h" // using a fork of the MPU library to change Z axis polarity. Use the include with "" not <>
#include "Logging/Logger.h"

// Calculated values based on data recording variances_calculation.py
// Variances:
// GyroZ    8.035852e-07 (rad/s)^2

class Imu
{
public:
	Imu();

	void print(ImuReading imuState);

	void calibrateImuAccelGyro();

	void calibrateImuMag(uint8_t ledPin);

	void calibrateImuAccelGyroBias(uint8_t ledPin);

	ImuReading getImuReading();

private:
	MPU9250 imu;
	unsigned long lastMeasurementTime;
	Logger &logger;

	// the Accel and Gyro get thier own bias, but this doesn't get the accuracy desired
	// therefore, we add our own bias compensation
	// this can be adjusted with calibrateImuAccelGyroBias()
	double accelXBias = 0.0; // m/s^2
	double gyroZBias = 0.0;	 // rad/s
};