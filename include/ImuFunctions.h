#pragma once

#include <MPU9250.h>

inline void connectImu(MPU9250 &imu)
{
    // intentional boot failure if imu does not connect
    if (!imu.setup(0x68)) // change to 0x69 if AD0 is HIGH
    {
        while (1)
        {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        }
    }
}

inline void printImuData(MPU9250 &imu)
{
    // printing any desired data to the Serial Monitor
    Serial.printf("Roll: %.2f  ", imu.getRoll());
    Serial.printf("Pitch: %.2f  ", imu.getPitch());
    Serial.printf("Yaw: %.2f  ", imu.getYaw());
    Serial.printf("MagX: %.2f  ", imu.getMagX());
    Serial.printf("MagY: %.2f  ", imu.getMagY());
    Serial.printf("MagZ: %.2f  ", imu.getMagZ());
    // Serial.print(imu.getTemperature(), 2);
    // Serial.print("    AccX: ");
    // Serial.print(imu.getAccX(), 2);
    // Serial.print("    AccY: ");
    // Serial.print(imu.getAccY(), 2);
    // Serial.print("    AccZ: ");
    // Serial.print(imu.getAccZ(), 2);

    Serial.println();
}

inline void calibrateImuAccelGyro(MPU9250 &imu)
{
    Serial.println("Accel Gyro calibration will start in 3sec.");
    Serial.println("Please leave the device still on the flat plane.");
    delay(3000);
    imu.calibrateAccelGyro();
    Serial.println("Accel Gyro Calibration Complete.");
}

inline void calibrateImuMag(MPU9250 &imu, uint8_t LedPin)
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