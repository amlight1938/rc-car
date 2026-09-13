#include "Logger.h"

Logger::Logger(){
	setupWifi();
	logQueue = xQueueCreateStatic(QUEUE_LENGTH, QUEUE_ITEM_SIZE, ucQueueStorageArea, &xStaticQueue);
}

Logger& Logger::getInstance(){
	static Logger instance;
	return instance;
}

void Logger::startTask(){
	xTaskCreatePinnedToCore(loggingTask, "loggingTask", 4000, NULL, 1, NULL, 0);
}

void Logger::loggingTask(void* params){
	Logger& logger = Logger::getInstance();
	while(true){
		logger.sendUdp();
		Serial.printf("High water mark: %u\n", uxTaskGetStackHighWaterMark(NULL));
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

void Logger::log(const std::string& message){
	char buffer[QUEUE_ITEM_SIZE];
	snprintf(buffer, QUEUE_ITEM_SIZE, "%s", message.c_str()); //%s so it doesnt accidentally interpret message as a format string
	xQueueSend(logQueue, buffer, 0);
}

void Logger::setupWifi(){
	//Serial printing and not logging yet since wifi isn't connected
	Serial.print("Connecting.");
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	while (WiFi.status() != WL_CONNECTED){
		delay(500);
		Serial.print(".");
	}
	Serial.println("\nConnected to Wi-Fi");
	Serial.print("ESP32 IP address: ");
  	Serial.println(WiFi.localIP());
}

//defined outside the class becuase static members that are runtime dependent cant be defined inside the class 
uint8_t Logger::ucQueueStorageArea[Logger::QUEUE_LENGTH * Logger::QUEUE_ITEM_SIZE];
StaticQueue_t Logger::xStaticQueue;
QueueHandle_t Logger::logQueue;