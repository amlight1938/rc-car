#pragma once

#include <cstdint>

class Motor
{
public:
    Motor(uint8_t plusPin, uint8_t minusPin, uint8_t speedPin);

    void spin(bool forward, uint8_t speedValue);

    void stop();

    bool isStopped();

    static constexpr double DEADZONE = 0.08;

private:
    uint8_t _plusPin;
    uint8_t _minusPin;
    uint8_t _speedPin;
    uint8_t _lastSpeedValue;
};