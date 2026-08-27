#include "Motor.h"
#include "Arduino.h"

Motor::Motor(uint8_t plusPin, uint8_t minusPin, uint8_t speedPin)
    : _plusPin(plusPin), _minusPin(minusPin), _speedPin(speedPin)
{
    pinMode(_plusPin, OUTPUT);
    pinMode(_minusPin, OUTPUT);
    pinMode(_speedPin, OUTPUT);
}

void Motor::control(double stickY, double stickX)
{
    // adding in deadzones for both stick controls
    if (-DEADZONE <= stickX && stickX <= DEADZONE)
    {
        stickX = 0.0;
    }
    if (-DEADZONE <= stickY && stickY <= DEADZONE)
    {
        stickY = 0.0;
    }

    // turning the motors off and performing an early exit if the sticks are in a neutral position
    if (stickY == 0.0 && stickX == 0.0)
    {
        if (!this->isStopped())
        {
            this->stop();
            Serial.println("stopping");
        }
        return;
    }

    // combining the two inputs to determine motor speed
    // scaling this speed by 1000 to losslessly convert to an int for mapping
    int speedControl = (stickY + (MOTOR_TURN_GAIN * stickX)) * 1000;

    // extracting the direction the motor should spin
    bool forward = speedControl > 0;

    // mapping the controller joystick inputs (0 to 1) to a PWM output
    speedControl = map(std::abs(speedControl), 0, 1000, 0, 255);

    this->spin(forward, speedControl);
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
    }
    else
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
