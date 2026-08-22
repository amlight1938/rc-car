#include <Arduino.h>



const int speedControlPin = 4;

class Motor {
  public:
    Motor(uint8_t plusPin, uint8_t minusPin, uint8_t speedPin)
      : _plusPin(plusPin), _minusPin(minusPin), _speedPin(speedPin) {
    }

    void begin() {
      pinMode(_plusPin, OUTPUT);
      pinMode(_minusPin, OUTPUT);
      pinMode(_speedPin, OUTPUT);
    }

    void spin(uint8_t plusValue, uint8_t minusValue, uint8_t speedValue) {
      digitalWrite(_plusPin, plusValue);
      digitalWrite(_minusPin, minusValue);
      analogWrite(_speedPin, speedValue);
    }

    void stop() {
      spin(0, 0, 0);
    }

  private:
    uint8_t _plusPin;
    uint8_t _minusPin;
    uint8_t _speedPin;
};

//used for bluetooth failsafe
unsigned long lastCommandReceivedTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 1000;

Motor leftMotor(27, 14, 13);
Motor rightMotor(25, 33, 32);

const int leftMotorPinForward = 27;
const int leftMotorPinReverse = 14;
const int leftMotorPinSpeed = 13;

const int rightMotorPinForward = 25;
const int rightMotorPinReverse = 33;
const int rightMotorPinSpeed = 32;

// the purpose of this task is to replace arduino's default loop() with a FreeRTOS task for multiple reasons:
// Deterministic timing - allows us to execute on a guarantee schedule wheras delay() or millis() scheduing drifts
// Priority - this allows us to assign priority to each task (logging, control)
// Specify core use - specify which core (0,1) to run this task on so we can separate it from bluetooth task
void controlTask(void* params){
    Serial.begin(115200);
    pinMode(leftMotorPinForward, OUTPUT);
    pinMode(leftMotorPinReverse, OUTPUT);
    pinMode(leftMotorPinSpeed, OUTPUT);

    pinMode(rightMotorPinForward, OUTPUT);
    pinMode(rightMotorPinReverse, OUTPUT);
    pinMode(rightMotorPinSpeed, OUTPUT);
    
    pinMode(speedControlPin, INPUT);

    //set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
    TickType_t lastWakeTime = xTaskGetTickCount();


        
        totalLoops++;

        //update pin outs for motors
        analogWrite(leftMotorPinForward, leftMotorOutputForward);
        analogWrite(leftMotorPinReverse, leftMotorOutputReverse);
        analogWrite(leftMotorPinSpeed, leftMotorOutputSpeed);

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
