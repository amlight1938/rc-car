#pragma once

// declaring pins for the LED outputs and standby control to arm/disarm the motors
const uint8_t STBY_PIN = 26;
const uint8_t BT_LED_PIN = 12;

// Motor pins
const uint8_t L_MOTOR_PLUS_PIN = 27;
const uint8_t L_MOTOR_MINUS_PIN = 14;
const uint8_t L_MOTOR_SPEED_PIN = 13;

const uint8_t R_MOTOR_PLUS_PIN = 33;
const uint8_t R_MOTOR_MINUS_PIN = 25;
const uint8_t R_MOTOR_SPEED_PIN = 32;

// Encoder pins
const uint8_t L_ENCODER_PHASE_A_PIN = 18;
const uint8_t L_ENCODER_PHASE_B_PIN = 19;
const uint8_t R_ENCODER_PHASE_A_PIN = RX2; //GPIO 16
const uint8_t R_ENCODER_PHASE_B_PIN = TX2; //GPIO 17

//I2C pins are 21 for SDA(I2C Data) and 22 for SCL(I2C Clock) for the ESP32
