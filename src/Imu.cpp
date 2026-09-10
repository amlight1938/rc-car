#include "Imu.h"

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
}

void Imu::print()
{
    if (imu.update())
    {
        // printing any desired data to the Serial Monitor
        Serial.printf("Roll: %.2f  ", imu.getRoll());
        Serial.printf("Pitch: %.2f  ", imu.getPitch());
        Serial.printf("Yaw: %.2f  ", imu.getYaw());
        Serial.printf("MagX: %.2f  ", imu.getMagX());
        Serial.printf("MagY: %.2f  ", imu.getMagY());
        Serial.printf("MagZ: %.2f  ", imu.getMagZ());
        // Serial.printf("Temp: %.2f  ", imu.getTemperature());
        // Serial.printf("AccX: %.2f  ", imu.getAccX());
        // Serial.printf("AccY: %.2f  ", imu.getAccY());
        // Serial.printf("AccZ: %.2f  ", imu.getAccZ());

        Serial.println();
    }
}

void Imu::calibrateImuAccelGyro()
{
    Serial.println("Accel Gyro calibration will start in 3sec.");
    Serial.println("Please leave the device still on the flat plane.");
    delay(3000);
    imu.calibrateAccelGyro();
    Serial.println("Accel Gyro Calibration Complete.");
}

void Imu::calibrateImuMag(uint8_t LedPin)
{
    // setting the bluetooth LED HIGH to indicate the Mag calibration has started
    digitalWrite(LedPin, HIGH);

    Serial.println("Mag calibration will start in 3sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(3000);
    imu.calibrateMag();
    Serial.println("Mag Calibration Complete.");

    // setting the bluetooth LED LOW to indicate the Mag calibration has ended
    digitalWrite(LedPin, LOW);
    delay(2000);
}

MPU9250 Imu::getMpu()
{
    return imu;
}