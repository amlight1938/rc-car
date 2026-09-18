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
			Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
		}
	}

	imu.setMagneticDeclination(7.5); //7.5 degrees in Denver, CO

	lastMeasurementTime = millis();
}

ImuReading Imu::getImuReading()
{

	boolean fresh = imu.update();
	unsigned long currentTime = millis(); // called after I2C update() call to get more accurate measurement time
	ImuReading imuState;
	imuState.isFreshData = fresh;
	imuState.timeSinceLastMeasurement = currentTime - lastMeasurementTime;
	imuState.roll = imu.getRoll();
	imuState.pitch = imu.getPitch();
	imuState.yaw = imu.getYaw();
	// imuState.magX = imu.getMagX();
	// imuState.magY = imu.getMagY();
	// imuState.magZ = imu.getMagZ();
	imuState.gyroX = imu.getGyroX();
	imuState.gyroY = imu.getGyroY();
	imuState.gyroZ = imu.getGyroZ();
	imuState.accX = imu.getAccX();
	imuState.accY = imu.getAccY();
	imuState.accZ = imu.getAccZ();

	lastMeasurementTime = currentTime;
	return imuState;
}

void Imu::print(ImuReading imuState)
{
	if (imu.update())
	{
		// printing any desired data to the Serial Monitor
		Serial.printf("Roll:  %.2f  ", imu.getRoll());
		Serial.printf("Pitch: %.2f  ", imu.getPitch());
		Serial.printf("Yaw:   %.2f  ", imu.getYaw());
		// Serial.printf("MagX:  %.2f  ", imu.getMagX());
		// Serial.printf("MagY:  %.2f  ", imu.getMagY());
		// Serial.printf("MagZ:  %.2f  ", imu.getMagZ());
		Serial.printf("GyroX: %.2f  ", imu.getGyroX());
		Serial.printf("GyroY: %.2f  ", imu.getGyroY());
		Serial.printf("GryoZ: %.2f  ", imu.getGyroZ());
		Serial.printf("AccX:  %.2f  ", imu.getAccX());
		Serial.printf("AccY:  %.2f  ", imu.getAccY());
		Serial.printf("AccZ:  %.2f  ", imu.getAccZ());
		Serial.println();
	}

		// Serial.printf("Temp: %.2f  ", imu.getTemperature());
		// Serial.printf("AccX: %.2f  ", imu.getAccX());
		// Serial.printf("AccY: %.2f  ", imu.getAccY());
		// Serial.printf("AccZ: %.2f  ", imu.getAccZ());

	// printing any desired data to the Serial Monitor
	// Serial.printf(">Roll: %.2f  ", imuState.roll);
	// Serial.printf(">Pitch: %.2f  ", imuState.pitch);
	// Serial.printf(">Yaw: %.2f  ", imuState.yaw);
	// Serial.printf(">GyroX: %.2f  ", imuState.gyroX);
	// Serial.printf(">GyroY: %.2f  ", imuState.gyroY);
	// Serial.printf(">GyroZ: %.2f  ", imuState.gyroZ);
	// Serial.printf("Temp: %.2f  ", imuState.Temperature);
	// Serial.printf("AccX: %.2f  ", imuState.accX);
	// Serial.printf("AccY: %.2f  ", imuState.accY);
	// Serial.printf("AccZ: %.2f  ", imuState.accZ);
	// Serial.print(">GyroX:");
	// Serial.println(imuState.gyroX);
	// Serial.print(">GyroY:");
	// Serial.println(imuState.gyroY);
	// Serial.print(">GyroZ:");
	// Serial.println(imuState.gyroZ);
	// Serial.print(">Roll:");
	// Serial.println(imuState.roll);
	// Serial.print(">Pitch:");
	// Serial.println(imuState.pitch);
	// Serial.print(">Yaw:");
	// Serial.println(imuState.yaw);

}

void Imu::calibrateImuAccelGyro()
{
	// logging the start of the Accelerometer and Gyroscope calibration
	logger.log(Logger::formatLog("[%lu]-Accel Gyro calibration starts in 3sec", millis() / 1000));
	logger.log(Logger::formatLog("[%lu]-Leave device still on flat plane", millis() / 1000));
	delay(3000);

	// starting the Accelerometer and Gyroscope calibration
	imu.calibrateAccelGyro();

	// logging the completion of the Accelerometer and Gyroscope calibration
	logger.log(Logger::formatLog("[%lu]-Accel Gyro Calibration Complete", millis() / 1000));
}

void Imu::calibrateImuMag(uint8_t ledPin)
{
	// setting the bluetooth LED HIGH to indicate the Mag calibration has started
	digitalWrite(ledPin, HIGH);

	// logging the start of the Mangetometer calibration
	logger.log(Logger::formatLog("[%lu]-Mag calibration will start in 3sec", millis() / 1000));
	logger.log(Logger::formatLog("[%lu]-Wave in a figure 8 until light turns off", millis() / 1000));
	delay(3000);

	// starting the Mangetometer calibration
	imu.calibrateMag();

	// logging the completion of the Mangetometer calibration
	logger.log(Logger::formatLog("[%lu]-Mag Calibration Complete", millis() / 1000));

	// setting the bluetooth LED LOW to indicate the Mag calibration has ended
	digitalWrite(ledPin, LOW);
}