#pragma once

struct ImuReading
{
	boolean isFreshData;
	double timeSinceLastMeasurement; // seconds
	double roll;					 // rad
	double pitch;					 // rad
	double yaw;						 // rad
	double gyroX;					 // rad/s
	double gyroY;					 // rad/s
	double gyroZ;					 // rad/s
	double accX;					 // m/s^2
	double accY;					 // m/s^2
	double accZ;					 // m/s^2
};
