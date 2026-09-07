#pragma once

#include <MPU9250.h>

inline void print_roll_pitch_yaw(MPU9250 imu)
{
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
    // Serial.println(imu.getAccZ(), 2);

    Serial.println();
}

inline void calibrate_imu(MPU9250 imu)
{
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    imu.verbose(true);
    delay(5000);
    imu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    imu.calibrateMag();

    imu.verbose(false);
}