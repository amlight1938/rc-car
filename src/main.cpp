#include <Arduino.h>
#include <BLEGamepadClient.h>
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

    Motor leftMotor(L_MOTOR_PLUS_PIN, L_MOTOR_MINUS_PIN, L_MOTOR_SPEED_PIN);
    Motor rightMotor(R_MOTOR_PLUS_PIN, R_MOTOR_MINUS_PIN, R_MOTOR_SPEED_PIN);

    // initializing all utilized pins as outputs or inputs
    pinMode(STBY_PIN, OUTPUT);
    pinMode(BT_LED_PIN, OUTPUT);

    // setting up the standby state for the motor controller
    bool stbyState = LOW;
    unsigned long stbyDelay = 500;
    unsigned long lastStbyToggle = 0;

    // setting up the blinking LED for the period the esp32 is searching for a bluetooth controller
    unsigned long pairingBlinkDuration = 500;    // 500 ms on, 500 off
    unsigned long autonomousBlinkDuration = 200; // 200 ms on, 200 off
    bool btLedState = LOW;
    unsigned long lastBlinkSwitch = 0;

    // list of states the car can enter
    enum State
    {
        PAIRING,
        DISARMED,
        MANUAL,
        AUTONOMOUS,
    };

    State currentState = PAIRING; // start in pairing mode
    State lastState;

    // set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        XboxControlsState s;
        unsigned long currentTime = millis();

        // enter pairing mode if controller disconnects
        if (!controller.isConnected())
        {
            currentState = PAIRING;
        }
        else
        {
            // gathering the controller inputs if controller is connected
            controller.read(&s);
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
            if (s.leftBumper == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = MANUAL; // always go straight to manual once stby button is toggled on
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
            if (s.leftBumper == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = DISARMED;
                break;
            }

            // switching to AUTONOMOUS based on controller inputs
            if (s.buttonY == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = AUTONOMOUS;
                break;
            }

            // powering the motors with manual control
            leftMotor.control(s.leftStickY, s.rightStickX);
            rightMotor.control(s.leftStickY, -s.rightStickX); // negative right stick to account for chirality differences for the motors

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
            if (s.leftBumper == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = DISARMED;
                break;
            }

            // switching to MANUAL based on controller inputs
            if (s.buttonY == true && currentTime - lastStbyToggle > stbyDelay)
            {
                lastStbyToggle = currentTime;
                lastState = currentState;
                currentState = MANUAL;
                break;
            }

            // future autonomous capabilities go here

            break;
        }

        // schedule the when the task should wake up and run another loop
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
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