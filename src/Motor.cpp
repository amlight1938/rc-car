#include "Motor.h"
#include "Arduino.h"


Motor::Motor(uint8_t plusPin, uint8_t minusPin, uint8_t speedPin)
    : _plusPin(plusPin), _minusPin(minusPin), _speedPin(speedPin)
{   
    pinMode(_plusPin, OUTPUT);
    pinMode(_minusPin, OUTPUT);
    pinMode(_speedPin, OUTPUT);
}

void Motor::control(double leftStickY, double rightStickX)
{
    if ( -DEADZONE <= leftStickY && leftStickY <= DEADZONE){
        leftStickY = 0.0;
    }
    if ( -DEADZONE <= rightStickX && rightStickX <= DEADZONE){
        rightStickX = 0.0;
    }
    
    if (leftStickY == 0.0 && rightStickX == 0.0){
        if (!this->isStopped()){
            this->stop();
        }
        return;
    }
    
    double speedControl = leftStickY + (MOTOR_TURN_GAIN * rightStickX);
    bool forward = speedControl > 0;
    int max_output = (std::abs(leftStickY) > 0)*1000 + (std::abs(rightStickX) > 0)*MOTOR_TURN_GAIN*1000;
    // map() internally converts doubles to long and will truncate decimals. This is okay for human control
    speedControl = map(std::abs(speedControl * 1000), 0, max_output, 0, 255); 
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
