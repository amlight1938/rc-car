#include <Arduino.h>

const int leftMotorPin = 3;
const int rightMotorPin = 4;

//used for bluetooth failsafe
unsigned long lastCommandReceivedTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 1000;

// the purpose of this task is to replace arduino's default loop() with a FreeRTOS task for multiple reasons:
// Deterministic timing - allows us to execute on a guarantee schedule wheras delay() or millis() scheduing drifts
// Priority - this allows us to assign priority to each task (logging, control)
// Specify core use - specify which core (0,1) to run this task on so we can separate it from bluetooth task
void controlTask(void* params){
    Serial.begin(115200);
    pinMode(leftMotorPin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);

    //set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
    TickType_t lastWakeTime = xTaskGetTickCount();

    while(true){
        int leftMotorValue = 5; //some bluetooth library call
        int rightMotorValue = 5; //some bluetooth library call
        //update lastCommandReceivedTime = millis() when bluetooth call is successful
        
        //failsafe to make sure motors dont continue with their last reading if bluetooth loses connection
        int leftOut = 0;
        int rightOut = 0;
        if (millis() - lastCommandReceivedTime >= COMMAND_TIMEOUT_MS){
            leftOut = map(leftMotorValue, 0, 4095, 0, 255);
            rightOut = map(rightMotorValue, 0, 4095, 0, 255);
        }

        //update pin outs for motors
        analogWrite(leftMotorPin, leftOut);
        analogWrite(rightMotorPin, rightOut);

        //print only every 10 iterations (100ms)
        static int counter = 0;
        if (++counter >= 10){
            Serial.print("Left motor input: ");
            Serial.println(leftMotorValue);
            Serial.print("Right motor input: ");
            Serial.println(rightMotorValue);
            counter = 0;
        }

        // schedule the when the task should wake up and run another loop
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
    }
}

void setup() {
    //create and run the task
    // task, human readable name, stack size allocation, input parameters, priority, returned task handle, core ID
    xTaskCreatePinnedToCore(controlTask, "controlTask", 4000, NULL, 2, NULL, 0); 
}

void loop() {
    // kill the arduino default loop task since we are running a deterministic time scheduler task
    vTaskDelete(NULL);
}
