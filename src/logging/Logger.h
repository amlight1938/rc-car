#pragma once
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <WiFiUdp.h>
#include <WiFi.h>
#include "WifiCredentials.h"

// a singleton class pattern - we want there to only ever be one instance of this class for the entirety of the car's lifecycle
class Logger {
	private:
		Logger();
		~Logger() = default;

		bool connectToWifi();

		bool sendUdp();

		//defines the task that logging is running on
		static void loggingTask(void* params);

		//Queue items
		static const uint16_t QUEUE_LENGTH = 1000; // the total number of items that can be on the queue at once. New logs are dropped if queue is full
		static const uint8_t QUEUE_ITEM_SIZE = 50; // the max item size of each message in bytes
		static uint8_t ucQueueStorageArea[QUEUE_LENGTH * QUEUE_ITEM_SIZE]; //the actual space in memeory for the queue to keep its item, passed into the xQueueCreateStatic function
		static StaticQueue_t xStaticQueue; // how the queue tracks its state, used internally by FreeRTOS and passed into the xQueueCreateStatic function
		static QueueHandle_t logQueue; // the actual queue that we can use push and pop log messages to and from 

		WiFiUDP udp; //the object that sends the data over wifi

		// used to log every RATE_LIMIT_COUNT_MAX control loop cycles so we dont overload udp sends
		uint8_t rateLimitIterationCounter = 0;
		uint8_t const RATE_LIMIT_COUNT_MAX = 10; // will log at a rate of 1 log per RATE_LIMIT_COUNT_MAX * control loop fixed rate

	public:
		// singleton class pattern requires we delete the copy, move, and assignment ctors so there's no backdoor to create a second instance of Logger
		Logger(const Logger&) = delete;
		Logger& operator=(const Logger&) = delete;
		Logger(Logger&&) = delete;
		Logger& operator=(Logger&&) = delete;

		// called by other classes to get the logger. Returns a reference to the logger instead of a new logging object. Static so it can be called without a logger object present
		static Logger& getInstance();

		// called by other classes to add messages to the queue to be sent via UDP
		void log(const std::string& message, bool rateLimit = false);

		// registers the task to be scheduled by FreeRTOS
		void createTask();

		// formats the logs to be of size QUEUE_ITEM_SIZE because the logQueue items maximum size is QUEUE_ITEM_SIZE
		// template becuase there is a variable amount of arguments passed in dpeending on message
		template<typename... Args>
		static std::string formatLog(const char* fmt, Args... args) {
			char buffer[QUEUE_ITEM_SIZE];
			snprintf(buffer, sizeof(buffer), fmt, args...);
			return std::string(buffer);
		}

		void incrementRateLimitCounter();
};
