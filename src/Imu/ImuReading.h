#pragma once

struct ImuReading
{
	boolean isFreshData;
	unsigned long timeSinceLastMeasurement;
	double roll;
	double pitch;
	double yaw;
	double magX;
	double magY;
	double magZ;
	double gyroX;
	double gyroY;
	double gyroZ;
};

//??do we have roll rate, yaw rate etc. ??