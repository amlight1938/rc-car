#include "Motor.h"
#include "Arduino.h"

Motor::Motor(uint8_t plusPin, uint8_t minusPin, uint8_t speedPin)
        : _plusPin(plusPin), _minusPin(minusPin), _speedPin(speedPin)
{
    pinMode(_plusPin, OUTPUT);
    pinMode(_minusPin, OUTPUT);
    pinMode(_speedPin, OUTPUT);
}

void Motor::spin(bool forward, uint8_t speedValue)
{
    _lastSpeedValue = speedValue;

    if (speedValue == 0)
    {
        analogWrite(_speedPin, speedValue);
        digitalWrite(_plusPin, 0);
        digitalWrite(_minusPin, 0);
        return;
    }

    if (forward)
    {
        digitalWrite(_plusPin, 1);
        digitalWrite(_minusPin, 0);
    }else
    {
        digitalWrite(_plusPin, 0);
        digitalWrite(_minusPin, 1);
    }

    analogWrite(_speedPin, speedValue);
}

void Motor::stop()
{
    spin(true, 0);
}

bool Motor::isStopped()
{
    return _lastSpeedValue == 0;
}
