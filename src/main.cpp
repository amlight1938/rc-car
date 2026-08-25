#include <Arduino.h>
#include <BLEGamepadClient.h>

class Motor
{
public:
    Motor(uint8_t plusPin, uint8_t minusPin, uint8_t speedPin)
        : _plusPin(plusPin), _minusPin(minusPin), _speedPin(speedPin)
    {
        pinMode(_plusPin, OUTPUT);
        pinMode(_minusPin, OUTPUT);
        pinMode(_speedPin, OUTPUT);
    }

    void spin(bool forward, uint8_t speedValue)
    {
        _lastSpeedValue = speedValue;

        if (speedValue == 0)
        {
            analogWrite(_speedPin, speedValue);
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

    void stop()
    {
        spin(true, 0);
    }

    bool isStopped()
    {
        return _lastSpeedValue == 0;
    }

private:
    uint8_t _plusPin;
    uint8_t _minusPin;
    uint8_t _speedPin;
    uint8_t _lastSpeedValue;
};

// instantiating the bluetooth controller and motors
XboxController controller;
Motor leftMotor(27, 14, 13);
Motor rightMotor(25, 33, 32);

// declaring pins for the LED outputs and standby control to arm/disarm the motors
const int stbyPin = 26;
const int bluetoothLedPin = 12;

// the purpose of this task is to replace arduino's default loop() with a FreeRTOS task for multiple reasons:
// Deterministic timing - allows us to execute on a guarantee schedule wheras delay() or millis() scheduing drifts
// Priority - this allows us to assign priority to each task (logging, control)
// Specify core use - specify which core (0,1) to run this task on so we can separate it from bluetooth task
void controlTask(void *params)
{
    Serial.begin(115200);

    // initializing all utilized pins as outputs or inputs
    controller.begin();
    pinMode(stbyPin, OUTPUT);
    pinMode(bluetoothLedPin, OUTPUT);

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
            // gathering the controller inputs
            XboxControlsState s;
            controller.read(&s);

            // toggling the standby LED and motor controller based on controller inputs
            if (s.leftBumper = true && millis() > (stbyDelay + lastStbyToggle)) {
                digitalWrite(stbyPin, stbyState);
                stbyState = !stbyState;
                lastStbyToggle = millis();
            }

            // mapping the controller joystick inputs (0 to 1) to a PWM output
            int speedControlLeft = s.leftStickY * 1000;
            int speedControlRight = s.rightStickY * 1000;
            speedControlLeft = map(std::abs(speedControlLeft), 0, 1000, 0, 255);
            speedControlRight = map(std::abs(speedControlRight), 0, 1000, 0, 255);

            // including a deadzone of 0.08 in each direction
            static const double deadzone = 0.08;

            // powering the left motor based on controller inputs
            if (s.leftStickY < -deadzone)
            {
                leftMotor.spin(false, speedControlLeft);
            }else if (s.leftStickY > deadzone)
            {
                leftMotor.spin(true, speedControlLeft);
            }else if (!leftMotor.isStopped())
            {
                Serial.println("left stopping");
                leftMotor.stop();
            }

            // powering the right motor based on controller inputs
            if (s.rightStickY < -deadzone)
            {
                rightMotor.spin(false, speedControlRight);
            }else if (s.rightStickY > deadzone)
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
            digitalWrite(bluetoothLedPin, blinkState);
            blinkState = !blinkState;
        }

        // schedule the when the task should wake up and run another loop
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
    }
}

void setup()
{
    // create and run the task
    //  task, human readable name, stack size allocation, input parameters, priority, returned task handle, core ID
    xTaskCreatePinnedToCore(controlTask, "controlTask", 4000, NULL, 2, NULL, 0);
}

void loop()
{
    // kill the arduino default loop task since we are running a deterministic time scheduler task
    vTaskDelete(NULL);
}