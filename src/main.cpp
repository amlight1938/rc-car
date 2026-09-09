#include <Arduino.h>
#include <BLEGamepadClient.h>
#include <MPU9250.h>
#include <ESP32Encoder.h>
#include <ArduinoEigen.h>
#include "ImuFunctions.h"
#include "Motor.h"
#include "Pins.h"

// the purpose of this task is to replace arduino's default loop() with a FreeRTOS task for multiple reasons:
// Deterministic timing - allows us to execute on a guarantee schedule wheras delay() or millis() scheduing drifts
// Priority - this allows us to assign priority to each task (logging, control)
// Specify core use - specify which core (0,1) to run this task on so we can separate it from bluetooth task
void controlTask(void *params)
{
    Serial.begin(115200);

    // instantiating the bluetooth controller and motors
    XboxController controller;
    controller.begin();

    // setting up pins as Inputs or Outputs
    pinSetup();

    // instantiating the motors from the motor class
    Motor leftMotor(L_MOTOR_PLUS_PIN, L_MOTOR_MINUS_PIN, L_MOTOR_SPEED_PIN);
    Motor rightMotor(R_MOTOR_PLUS_PIN, R_MOTOR_MINUS_PIN, R_MOTOR_SPEED_PIN);

    // instantiating the motor encoders from the encoder class
    ESP32Encoder leftEncoder;
    ESP32Encoder rightEncoder;

    // starting the count for each motor encoder
    leftEncoder.attachFullQuad(L_ENCODER_PHASE_A_PIN, L_ENCODER_PHASE_B_PIN); // 4x decoding
    leftEncoder.setCount(0);
    rightEncoder.attachFullQuad(R_ENCODER_PHASE_A_PIN, R_ENCODER_PHASE_B_PIN); // 4x decoding
    rightEncoder.setCount(0);

    // setting up the standby state for the motor controller
    bool stbyState = LOW;
    unsigned long stbyDelay = 500;
    unsigned long lastStbyToggle = 0;

    // setting up the blinking LED for the period the esp32 is searching for a bluetooth controller
    unsigned long pairingBlinkDuration = 500;    // 500 ms on, 500 off
    unsigned long autonomousBlinkDuration = 200; // 200 ms on, 200 off
    bool btLedState = LOW;
    unsigned long lastBlinkSwitch = 0;

    unsigned long prevTime = millis();
    double leftMotorCount = 0;
    double rightMotorCount = 0;
    double prevLeftMotorCount;
    double prevRightMotorCount;
    double triggerRate;
    double velocity;

    // list of states the car can enter
    enum State
    {
        PAIRING,
        DISARMED,
        MANUAL,
        AUTONOMOUS,
        CALIBRATION
    };

    State currentState = PAIRING; // start in pairing mode
    State lastState;

    // setting up I2C and starting the IMU
    Wire.begin();
    MPU9250 imu;
    connectImu(imu);

    // set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        XboxControlsState controllerState;
        unsigned long currentTime = millis();

        // enter pairing mode if controller disconnects
        if (!controller.isConnected())
        {
            currentState = PAIRING;
        }
        else
        {
            // gathering the controller inputs if controller is connected
            controller.read(&controllerState);
        }

        // performing unique actions based on the cars state
        switch (currentState)
        {
        case PAIRING:
            // leaving the pairing state when a controller connects
            if (controller.isConnected())
            {
                lastState = currentState;
                currentState = DISARMED;
                break;
            }

            // stopping the motors if the controller disconnects
            if (lastState != PAIRING)
            {
                stbyState = LOW;
                digitalWrite(STBY_PIN, stbyState);
                leftMotor.stop();
                rightMotor.stop();
            }

            // causing the bluetooth LED to blink while searching for a controller
            // blinking will occur at a frequency based on pairingBlinkDuration
            if (currentTime - lastBlinkSwitch > pairingBlinkDuration)
            {
                lastBlinkSwitch = currentTime;
                digitalWrite(BT_LED_PIN, btLedState);
                btLedState = !btLedState;
            }

            break;

        case DISARMED:
            // turning the bluetooth LED ON and the standby LED OFF when entering the state
            if (lastState != DISARMED)
            {
                btLedState = HIGH;
                digitalWrite(BT_LED_PIN, btLedState);
                stbyState = LOW;
                digitalWrite(STBY_PIN, stbyState);
                leftMotor.stop();
                rightMotor.stop();
            }

            // switching to MANUAL based on controller inputs
            if (controllerState.leftBumper == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = MANUAL; // always go straight to manual once stby button is toggled on
                break;
            }

            // switching to CALIBRATION based on controller inputs
            if (controllerState.dpadDown == true && controllerState.buttonA == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = CALIBRATION;
                break;
            }

            break;

        case MANUAL:
            // turning the bluetooth LED ON and the standby LED O when entering the state
            if (lastState != MANUAL)
            {
                stbyState = HIGH;
                digitalWrite(STBY_PIN, stbyState);
                // need to turn the btLED on (at least for the time being for indication)
                btLedState = HIGH;
                digitalWrite(BT_LED_PIN, btLedState);
            }

            // switching to DISARMED based on controller inputs
            if (controllerState.leftBumper == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = DISARMED;
                break;
            }

            // switching to AUTONOMOUS based on controller inputs
            if (controllerState.buttonY == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = AUTONOMOUS;
                break;
            }

            // powering the motors with manual control
            leftMotor.control(controllerState.leftStickY, controllerState.rightStickX);
            rightMotor.control(controllerState.leftStickY, -controllerState.rightStickX); // negative right stick to account for chirality differences for the motors

            // reading out the encoder values for the left and right motors
            prevLeftMotorCount = leftMotorCount;
            prevRightMotorCount = rightMotorCount;
            leftMotorCount = -leftEncoder.getCount(); //left motor .getCount decreases as the car moves forward
            rightMotorCount = rightEncoder.getCount(); //right motor .getCount increase as the car moves forward

            // calculating number of triggers/millisecond then scaling to triggers/second
            triggerRate = 1000 * (((leftMotorCount - prevLeftMotorCount) + (rightMotorCount - prevRightMotorCount) / 2) / (currentTime - prevTime));

            // motor tests indicate 2772 triggers/axle rotation
            // wheel diameter is 0.070m, therefore 0.2199m circumference
            // 2772 triggers/0.2199m or 12605 (trig/m)
            // (m/s) = (trig/s) / (trig/m)
            velocity = triggerRate / 12605;

            Serial.printf("Count %ld  ", leftMotorCount);
            Serial.printf("Trig Rate %.3f  ", triggerRate);
            Serial.printf("Vel %.3f", velocity);
            Serial.println();

            break;

        case AUTONOMOUS:
            // turning the both motors off when entering the state
            if (lastState != AUTONOMOUS)
            {
                leftMotor.stop();
                rightMotor.stop();
            }

            // causing the bluetooth LED to blink quickly while in AUTONOMOUS mode
            // blinking will occur at a frequency based on autonomousBlinkDuration
            if (currentTime - lastBlinkSwitch > autonomousBlinkDuration)
            {
                lastBlinkSwitch = currentTime;
                digitalWrite(BT_LED_PIN, btLedState);
                btLedState = !btLedState;
            }

            // switching to DISARMED based on controller inputs
            if (controllerState.leftBumper == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = DISARMED;
                break;
            }

            // switching to MANUAL based on controller inputs
            if (controllerState.buttonY == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = MANUAL;
                break;
            }

            // future autonomous capabilities go here

            // imu readout testing
            if (imu.update())
            {
                printImuData(imu);
            }

            break;

        case CALIBRATION:
            // turning the both motors off when entering the state
            leftMotor.stop();
            rightMotor.stop();

            // turning the bluetooth LED and Standby  off when calbrating
            btLedState = LOW;
            digitalWrite(BT_LED_PIN, btLedState);
            stbyState = LOW;
            digitalWrite(STBY_PIN, stbyState);

            // calibrating the IMUs Accelerometer and Gyroscope
            calibrateImuAccelGyro(imu);

            // calibrating the IMUs Magnetometer
            calibrateImuMag(imu, CAL_LED_PIN);

            // returning to DISARMED after calibration is complete
            lastState = currentState;
            currentState = DISARMED;
            break;
        }

        // tracking the time at the previous loop for velocity calculations
        prevTime = currentTime;

        // schedule when the task should wake up and run another loop
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}

void setup()
{
    // create and run the task
    // task, human readable name, stack size allocation, input parameters, priority, returned task handle, core ID
    xTaskCreatePinnedToCore(controlTask, "controlTask", 4000, NULL, 2, NULL, 0);
}

void loop()
{
    // kill the arduino default loop task since we are running a deterministic time scheduler task
    vTaskDelete(NULL);
}
