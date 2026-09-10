#include "Encoder.h"

Encoder::Encoder(uint8_t L_ENCODER_PHASE_A_PIN, uint8_t L_ENCODER_PHASE_B_PIN, uint8_t R_ENCODER_PHASE_A_PIN, uint8_t R_ENCODER_PHASE_B_PIN)
{
    // starting the count for each motor encoder
    leftEncoder.attachFullQuad(L_ENCODER_PHASE_A_PIN, L_ENCODER_PHASE_B_PIN); // 4x decoding
    leftEncoder.setCount(0);
    rightEncoder.attachFullQuad(R_ENCODER_PHASE_A_PIN, R_ENCODER_PHASE_B_PIN); // 4x decoding
    rightEncoder.setCount(0);

    leftMotorCount = 0;
    rightMotorCount = 0;
}

double Encoder::getVelocity(unsigned long &prevTime, unsigned long &currentTime)
{
    // reading out the encoder values for the left and right motors
    prevLeftMotorCount = leftMotorCount;
    prevRightMotorCount = rightMotorCount;
    leftMotorCount = -leftEncoder.getCount();  // left motor .getCount decreases as the car moves forward
    rightMotorCount = rightEncoder.getCount(); // right motor .getCount increase as the car moves forward

    // calculating number of triggers/millisecond then scaling to triggers/second
    triggerRate = 1000 * (((leftMotorCount - prevLeftMotorCount) + (rightMotorCount - prevRightMotorCount) / 2) / (currentTime - prevTime));

    // motor tests indicate 2772 triggers/axle rotation
    // wheel diameter is 0.070m, therefore 0.2199m circumference
    // 2772 triggers/0.2199m or 12605 (trig/m)
    // (m/s) = (trig/s) / (trig/m)
    velocity = triggerRate / 12605;

    return velocity;
}

ESP32Encoder Encoder::getLeftEncoder()
{
    return leftEncoder;
}

ESP32Encoder Encoder::getRightEncoder()
{
    return rightEncoder;
}