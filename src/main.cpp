#include <Arduino.h>
#include <BLEGamepadClient.h>
#include "Motor.h"
#include "Pins.h"

// instantiating the bluetooth controller and motors
XboxController controller;
Motor leftMotor(L_MOTOR_PLUS_PIN, L_MOTOR_MINUS_PIN, L_MOTOR_SPEED_PIN);
Motor rightMotor(R_MOTOR_PLUS_PIN, R_MOTOR_MINUS_PIN, R_MOTOR_SPEED_PIN);

// the purpose of this task is to replace arduino's default loop() with a FreeRTOS task for multiple reasons:
// Deterministic timing - allows us to execute on a guarantee schedule wheras delay() or millis() scheduing drifts
// Priority - this allows us to assign priority to each task (logging, control)
// Specify core use - specify which core (0,1) to run this task on so we can separate it from bluetooth task
void controlTask(void *params)
{
    Serial.begin(115200);

    // initializing all utilized pins as outputs or inputs
    controller.begin();
    pinMode(STBY_PIN, OUTPUT);
    pinMode(BT_LED_PIN, OUTPUT);

    // setting up the standby state for the motor controller
    bool stbyState = LOW;
    int stbyDelay = 500;
    int lastStbyToggle = -stbyDelay;

    // setting up the blinking LED for the period the esp32 is searching for a bluetooth controller
    bool blinkState = LOW;

    // set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {

        if (controller.isConnected())
        {
            // keeping the bluetooth LED high once connected
            digitalWrite(BT_LED_PIN, HIGH);

            // gathering the controller inputs
            XboxControlsState s;
            controller.read(&s);

            // toggling the standby LED and motor controller based on controller inputs
            if (s.leftBumper == true && millis() > (stbyDelay + lastStbyToggle)) {
                digitalWrite(STBY_PIN, stbyState);
                stbyState = !stbyState;
                lastStbyToggle = millis();
            }

            // mapping the controller joystick inputs (0 to 1) to a PWM output
            int speedControlLeft = s.leftStickY * 1000;
            int speedControlRight = s.rightStickY * 1000;
            speedControlLeft = map(std::abs(speedControlLeft), 0, 1000, 0, 255);
            speedControlRight = map(std::abs(speedControlRight), 0, 1000, 0, 255);

            // including a Motor::DEADZONE of 0.08 in each direction
            // powering the left motor based on controller inputs
            if (s.leftStickY < -Motor::DEADZONE)
            {
                leftMotor.spin(false, speedControlLeft);
            }else if (s.leftStickY > Motor::DEADZONE)
            {
                leftMotor.spin(true, speedControlLeft);
            }else if (!leftMotor.isStopped())
            {
                Serial.println("left stopping");
                leftMotor.stop();
            }

            // powering the right motor based on controller inputs
            if (s.rightStickY < -Motor::DEADZONE)
            {
                rightMotor.spin(false, speedControlRight);
            }else if (s.rightStickY > Motor::DEADZONE)
            {
                rightMotor.spin(true, speedControlRight);
            }else if (!rightMotor.isStopped())
            {
                Serial.println("right stopping");
                rightMotor.stop();
            }
        }else
        {
            Serial.println("controller not connected");

            // causing the bluetooth LED to blink while searching for a controller
            // blinking will occur at a frequency based on the task schedule period
            digitalWrite(BT_LED_PIN, blinkState);
            blinkState = !blinkState;
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