#pragma once

struct ImuReading
{
	boolean isFreshData;
	unsigned long timeSinceLastMeasurement;
	double roll;
	double pitch;
	double yaw;
	// double magX;
	// double magY;
	// double magZ;
	double gyroX;
	double gyroY;
	double gyroZ;
	double accX;
	double accY;
	double accZ;
};

//??do we have roll rate, yaw rate etc. ??