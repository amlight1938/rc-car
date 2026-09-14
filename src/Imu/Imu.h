#pragma once

#include "Arduino.h"
#include "Imu/ImuReading.h"
#include <MPU9250.h>

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
};