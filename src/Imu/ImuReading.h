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

// gyroX: roll
// gyroY: pitch
// gyroZ: yaw

// accX: bow (forward)
// accY: port (left)
// accZ: zenith (up)

// accZ at stationary includes gravity (+9.81m/s^2)