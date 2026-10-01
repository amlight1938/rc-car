#pragma once

struct ImuReading
{
	boolean isFreshData;
	double timeSinceLastMeasurement;
	double roll;
	double pitch;
	double yaw;
	double gyroX;
	double gyroY;
	double gyroZ;
	double accX;
	double accY;
	double accZ;
};
