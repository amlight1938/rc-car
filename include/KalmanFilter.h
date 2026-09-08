#pragma once

#include "Arduino.h"
#include <ArduinoEigen.h>

class KalmanFilter
{
public:
    KalmanFilter();

    void iterate(double theta);

    double getX();
    double getY();
    double getTheta();

private:
    static constexpr double R = 4;

    static constexpr uint8_t H[1][4] = {
        {0, 0, 1, 0}};

    double theta;
    double X;
    double Y;
};