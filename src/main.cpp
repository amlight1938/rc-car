#include <Arduino.h>

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

	void spin(uint8_t plusValue, uint8_t minusValue, uint8_t speedValue)
	{
		digitalWrite(_plusPin, plusValue);
		digitalWrite(_minusPin, minusValue);
		analogWrite(_speedPin, speedValue);
	}

	void stop()
	{
		spin(0, 0, 0);
	}

private:
	uint8_t _plusPin;
	uint8_t _minusPin;
	uint8_t _speedPin;
};

const int speedControlPin = 4;

Motor leftMotor(27, 14, 13);
Motor rightMotor(25, 33, 32);

// the purpose of this task is to replace arduino's default loop() with a FreeRTOS task for multiple reasons:
// Deterministic timing - allows us to execute on a guarantee schedule wheras delay() or millis() scheduing drifts
// Priority - this allows us to assign priority to each task (logging, control)
// Specify core use - specify which core (0,1) to run this task on so we can separate it from bluetooth task
void controlTask(void *params)
{
	Serial.begin(115200);

	pinMode(speedControlPin, INPUT);

	// set lastWakeTime once outside the loop and it will be set going forward by vTaskDelayUntil()
	TickType_t lastWakeTime = xTaskGetTickCount();

	while (true)
	{
		int speedControl = analogRead(speedControlPin);
		Serial.print("Speed Input = ");
		Serial.print(speedControl);
		speedControl = map(speedControl, 0, 4095, 0, 255);
		Serial.print("   Speed Output = ");
		Serial.println(speedControl);

		leftMotor.spin(1, 0, speedControl);
		rightMotor.spin(1, 0, speedControl);

		// schedule the when the task should wake up and run another loop
		// vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
		delay(5);
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