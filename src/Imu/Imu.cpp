#include "Imu/Imu.h"

Imu::Imu()
	: logger(Logger::getInstance())
{
	Wire.begin();

	// intentional boot failure if imu does not connect
	if (!imu.setup(0x68)) // change to 0x69 if AD0 is HIGH
	{
		while (1)
		{
			logger.log("[%.1f]-MPU connection failed.c", millis() / 1000.0);
		}
	}

	// Denver, CO sits at 7.5deg North
	imu.setMagneticDeclination(7.5);

	lastMeasurementTime = millis();
}

ImuReading Imu::getImuReading()
{
	boolean fresh = imu.update();
	unsigned long currentTime = millis(); // called after I2C update() call to get more accurate measurement time
	ImuReading imuState;
	imuState.isFreshData = fresh;
	imuState.timeSinceLastMeasurement = (currentTime - lastMeasurementTime) / 1000.0; // seconds

	imuState.roll = imu.getRoll() * DEG_TO_RAD;	  // rad
	imuState.pitch = imu.getPitch() * DEG_TO_RAD; // rad
	imuState.yaw = imu.getYaw() * DEG_TO_RAD;	  // rad

	imuState.gyroX = imu.getGyroX() * DEG_TO_RAD; // rad/s
	imuState.gyroY = imu.getGyroY() * DEG_TO_RAD; // rad/s
	// compensating for the fact that stationary gyroZ value is offset from zero (biased)
	imuState.gyroZ = imu.getGyroZ() * DEG_TO_RAD - GYRO_Z_BIAS; // rad/s

	imuState.accX = imu.getAccX() * 9.81; // m/s^2
	imuState.accY = imu.getAccY() * 9.81; // m/s^2
	imuState.accZ = imu.getAccZ() * 9.81; // m/s^2

	lastMeasurementTime = currentTime; // seconds
	return imuState;
}

void Imu::calibrateImuAccelGyro()
{
	// logging the start of the Accelerometer and Gyroscope calibration
	logger.log("[%.1f]-Accel Gyro calibration starts in 3sec", millis() / 1000.0);
	logger.log("[%.1f]-Leave device still on flat plane", millis() / 1000.0);
	delay(3000);

	// starting the Accelerometer and Gyroscope calibration
	imu.calibrateAccelGyro();

	// logging the completion of the Accelerometer and Gyroscope calibration
	logger.log("[%.1f]-Accel Gyro Calibration Complete", millis() / 1000.0);
}

void Imu::calibrateImuMag(uint8_t ledPin)
{
	// setting the bluetooth LED HIGH to indicate the Mag calibration has started
	digitalWrite(ledPin, HIGH);

	// logging the start of the Mangetometer calibration
	logger.log("[%.1f]-Mag calibration will start in 3sec", millis() / 1000.0);
	logger.log("[%.1f]-Wave in a figure 8 until light turns off", millis() / 1000.0);
	delay(3000);

	// starting the Mangetometer calibration
	imu.calibrateMag();

	// logging the completion of the Mangetometer calibration
	logger.log("[%.1f]-Mag Calibration Complete", millis() / 1000.0);

	// setting the bluetooth LED LOW to indicate the Mag calibration has ended
	digitalWrite(ledPin, LOW);
}