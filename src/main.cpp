#include <Arduino.h>

const int leftMotorPinForward = A1;
const int leftMotorPinReverse = A2;

//used for bluetooth failsafe
unsigned long lastCommandReceivedTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 1000;

// the purpose of this task is to replace arduino's default loop() with a FreeRTOS task for multiple reasons:
// Deterministic timing - allows us to execute on a guarantee schedule wheras delay() or millis() scheduing drifts
// Priority - this allows us to assign priority to each task (logging, control)
// Specify core use - specify which core (0,1) to run this task on so we can separate it from bluetooth task
void controlTask(void* params){
    Serial.begin(115200);
    pinMode(leftMotorPinForward, OUTPUT);
    pinMode(leftMotorPinReverse, OUTPUT);

    //set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
    TickType_t lastWakeTime = xTaskGetTickCount();

    while(true){
        static int totalLoops = 0;
        int leftMotorOutputForward = 0;
        int leftMotorOutputReverse = 0;
        if(totalLoops <= 1000){
            leftMotorOutputForward = 0;
            leftMotorOutputReverse = 0;
        } else if(totalLoops > 1000 && totalLoops <= 2000){
            leftMotorOutputForward = 50;
            leftMotorOutputReverse = 0;
        } else if(totalLoops > 2000 && totalLoops <= 3000){
            leftMotorOutputForward = 0;
            leftMotorOutputReverse = 50;
        } else {
            totalLoops = 0;
        }
        
        totalLoops++;

        //update pin outs for motors
        analogWrite(leftMotorPinForward, leftMotorOutputForward);
        analogWrite(leftMotorPinReverse, leftMotorOutputReverse);

        //print only every 10 iterations (100ms)
        static int counter = 0;
        if (++counter >= 10){
            Serial.print("Left motor forward input: ");
            Serial.println(leftMotorOutputForward);
            Serial.print("Left motor reverse input: ");
            Serial.println(leftMotorOutputReverse);
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
