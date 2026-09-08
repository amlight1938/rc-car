#pragma once

#include "Arduino.h"

class Motor
{
public:
    Motor(uint8_t plusPin, uint8_t minusPin, uint8_t speedPin);

    void control(double stickY, double stickX);

    void stop();

private:
    static constexpr double DEADZONE = 0.15;
    static constexpr double MOTOR_TURN_GAIN = 0.5;

    void spin(bool forward, uint8_t speedValue);

    bool isStopped();

    uint8_t _plusPin;
    uint8_t _minusPin;
    uint8_t _speedPin;
    uint8_t _lastSpeedValue;
};