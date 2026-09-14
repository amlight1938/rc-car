#include "Imu/Imu.h"

Imu::Imu()
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
	// imuState.accX = imu.getAccX();
	// imuState.accY = imu.getAccY();
	// imuState.accZ = imu.getAccZ();

	lastMeasurementTime = currentTime;
	return imuState;
}

void Imu::print(ImuReading imuState)
{

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
	Serial.print(">Roll:");
	Serial.println(imuState.roll);
	Serial.print(">Pitch:");
	Serial.println(imuState.pitch);
	Serial.print(">Yaw:");
	Serial.println(imuState.yaw);

	Serial.println();
}

void Imu::calibrateImuAccelGyro()
{
	Serial.println("Accel Gyro calibration will start in 3sec.");
	Serial.println("Please leave the device still on the flat plane.");
	delay(3000);
	imu.calibrateAccelGyro();
	Serial.println("Accel Gyro Calibration Complete.");
}

void Imu::calibrateImuMag(uint8_t ledPin)
{
	// setting the bluetooth LED HIGH to indicate the Mag calibration has started
	digitalWrite(ledPin, HIGH);

	Serial.println("Mag calibration will start in 3sec.");
	Serial.println("Please Wave device in a figure eight until done.");
	delay(3000);
	imu.calibrateMag();
	Serial.println("Mag Calibration Complete.");

	// setting the bluetooth LED LOW to indicate the Mag calibration has ended
	digitalWrite(ledPin, LOW);
	delay(2000);
}