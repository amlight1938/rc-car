#pragma once

#include "Arduino.h"
#include <MPU9250.h>

class Imu
{
public:
    Imu();

    void print();

    void calibrateImuAccelGyro();

    void calibrateImuMag(uint8_t LedPin);

    MPU9250 getMpu();
    // returns the mpu9250 object

private:
    MPU9250 imu;

};