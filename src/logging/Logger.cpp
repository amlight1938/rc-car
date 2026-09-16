#include "logging/Logger.h"

Logger::Logger(){
	bool result = connectToWifi();
	if (!result){
		Serial.println("Restarting ESP32...");
		delay(1000);
		ESP.restart();
	}
	logQueue = xQueueCreateStatic(QUEUE_LENGTH, QUEUE_ITEM_SIZE, ucQueueStorageArea, &xStaticQueue);
}

Logger& Logger::getInstance(){
	static Logger instance;
	return instance;
}

void Logger::createTask(){
	xTaskCreatePinnedToCore(loggingTask, "loggingTask", 4000, NULL, 1, NULL, 0);
}

void Logger::loggingTask(void* params){
	Logger& logger = Logger::getInstance();
	while(true){
		bool result = logger.sendUdp();
		Serial.printf("[%lu] UDP send: %s\n", millis(), result ? "OK" : "FAILEDDDDDDDDDD");
	}
}

bool Logger::sendUdp(){
	//take item off queue
	char buffer[QUEUE_ITEM_SIZE];
	xQueueReceive(logQueue, buffer, portMAX_DELAY); //wait indefinitely for something to come to the queue

	//send it over UDP to receiveing IP
	udp.beginPacket(RECEIVING_IP_ADDRESS, RECEIVING_PORT);
	udp.print(buffer);
	return udp.endPacket();
}

void Logger::log(const std::string& message, bool rateLimit){
	// skip log to rate limit the logging if boolean is true and we are not on the start of the rate limit cycle
	if (rateLimit && rateLimitIterationCounter != 0){
		return;
	}

	char buffer[QUEUE_ITEM_SIZE];
	snprintf(buffer, QUEUE_ITEM_SIZE, "%s", message.c_str()); //%s so it doesnt accidentally interpret message as a format string
	xQueueSend(logQueue, buffer, 0);
}

bool Logger::connectToWifi(){
	unsigned long timeout = 10000; 

	//Serial printing and not logging yet since wifi isn't connected
	Serial.print("Connecting to WiFi.");
	//try 3 times 10 seconds each
	for (size_t i = 0; i < 3; i++){
		if (i > 0){
			Serial.println("Timed out. Retrying...");
		}

		WiFi.disconnect(true); //forget previous networks
		delay(500);
		WiFi.mode(WIFI_STA); //set to connection mode
		// WiFi.setSleep(false); //make sure it doesnt miss any wifi pings

		WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

		unsigned long startTime = millis();
		while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeout){
			delay(500);
			Serial.print(".");
		}

		if (WiFi.status() == WL_CONNECTED) {
            break;  // exit early on success, don't burn remaining attempts
        }
	}

	// WiFi.setSleep(true); // set it back because it needs to yield time to BLE later on

	if (WiFi.status() == WL_CONNECTED){
		Serial.println("\nConnected to Wi-Fi");
		Serial.print("ESP32 IP address: ");
		Serial.println(WiFi.localIP());
		delay(2000); //time to allow wifi connection to fully settle
		return true;
	}else{
		Serial.println("Failed to connect to WiFi");
		return false;
	}
}

void Logger::incrementRateLimitCounter(){ 
	if (rateLimitIterationCounter == RATE_LIMIT_COUNT_MAX - 1){
		rateLimitIterationCounter = 0;
	}else{
		rateLimitIterationCounter++; 
	}
}

//defined outside the class becuase static members that are runtime dependent cant be defined inside the class 
uint8_t Logger::ucQueueStorageArea[Logger::QUEUE_LENGTH * Logger::QUEUE_ITEM_SIZE];
StaticQueue_t Logger::xStaticQueue;
QueueHandle_t Logger::logQueue;