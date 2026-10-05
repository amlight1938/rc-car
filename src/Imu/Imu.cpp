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

	// only updating time since last measurement if the measurement was fresh
	if (fresh)
	{
		imuState.timeSinceLastMeasurement = (currentTime - lastMeasurementTime) / 1000.0; // seconds
		lastMeasurementTime = currentTime;												  // seconds
	}

	imuState.roll = imu.getRoll() * DEG_TO_RAD;	  // rad
	imuState.pitch = imu.getPitch() * DEG_TO_RAD; // rad
	imuState.yaw = imu.getYaw() * DEG_TO_RAD;	  // rad

	imuState.gyroX = imu.getGyroX() * DEG_TO_RAD; // rad/s
	imuState.gyroY = imu.getGyroY() * DEG_TO_RAD; // rad/s
	// compensating for the fact that the stationary gyroZ value is offset from zero (biased)
	imuState.gyroZ = imu.getGyroZ() * DEG_TO_RAD - gyroZBias; // rad/s

	// compensating for the fact that the stationary accX value is offset from zero (biased)
	imuState.accX = (imu.getAccX() * 9.81) - accelXBias; // m/s^2
	imuState.accY = imu.getAccY() * 9.81;				 // m/s^2
	imuState.accZ = imu.getAccZ() * 9.81;				 // m/s^2

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

void Imu::calibrateImuAccelGyroBias()
{
	// logging the start of the Accelerometer and Gyroscope Bias calibration
	logger.log("[%.1f]-Accel Gyro Bias calibration starts in 3sec", millis() / 1000.0);
	logger.log("[%.1f]-Leave device still on flat plane", millis() / 1000.0);
	delay(3000);

	// dropping the current accelXBias and gyroZBias back to 0 for accurate calibration
	accelXBias = 0;
	gyroZBias = 0;

	// starting the Gyroscope calibration
	static const uint16_t dataCount = 500;
	uint16_t collected = 0;
	double accelSum = 0.0;
	double gyroSum = 0.0;
	uint16_t loopCount = 0;

	while (collected < dataCount)
	{
		const ImuReading imuReading = getImuReading();
		if (imuReading.isFreshData)
		{
			// summing all of the accel and gyro readings
			accelSum += imuReading.accX;
			gyroSum += imuReading.gyroZ;
			++collected;
		}

		++loopCount;
		if (loopCount > 500)
		{
			logger.log("[%.1f]-Accel Gyro Bias calibration failed: insufficient fresh data", millis() / 1000.0);
			break;
		}

		delay(5);
	}

	// averaging the accel and gyro readings
	accelXBias = accelSum / dataCount;
	gyroZBias = gyroSum / dataCount;

	// logging the completion of the Accelerometer and Gyroscope calibration
	logger.log("[%.1f]-Accel Gyro Bias Calibration Complete", millis() / 1000.0);
}