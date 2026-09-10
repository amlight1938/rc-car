#pragma once

#include "Arduino.h"
#include <ESP32Encoder.h>

class Encoder
{
public:
    Encoder(uint8_t L_ENCODER_PHASE_A_PIN, uint8_t L_ENCODER_PHASE_B_PIN, uint8_t R_ENCODER_PHASE_A_PIN, uint8_t R_ENCODER_PHASE_B_PIN);

    double getVelocity(unsigned long &prevTime, unsigned long &currentTime);
    // gets the average velocity of the two motors with 70mm diameter wheels since the previous getVelocity request
    // velocity is sent as meters/second

    ESP32Encoder getLeftEncoder();
    // returns the left encoder object

    ESP32Encoder getRightEncoder();
    // returns the right encoder object

private:
    ESP32Encoder leftEncoder;
    ESP32Encoder rightEncoder;

    double leftMotorCount;
    double rightMotorCount;
    double prevLeftMotorCount;
    double prevRightMotorCount;
    double triggerRate;
    double velocity;
};