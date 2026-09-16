#pragma once
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <WiFiUdp.h>
#include <WiFi.h>
#include "WifiCredentials.h"

//a singleton class
class Logger {
	private:
		Logger();
		~Logger() = default;

		bool connectToWifi();

		bool sendUdp();

		static void loggingTask(void* params);

		//Queue items
		static const uint16_t QUEUE_LENGTH = 1000;
		static const uint8_t QUEUE_ITEM_SIZE = 50; //bytes
		static uint8_t ucQueueStorageArea[QUEUE_LENGTH * QUEUE_ITEM_SIZE];
		static StaticQueue_t xStaticQueue;
		static QueueHandle_t logQueue;

		WiFiUDP udp;

		// used to log every RATE_LIMIT_COUNT_MAX control loop cycles so we dont overload udp sends
		uint8_t rateLimitIterationCounter = 0;
		uint8_t const RATE_LIMIT_COUNT_MAX = 10; // will log at a rate of 1 log per RATE_LIMIT_COUNT_MAX * control loop fixed rate

	public:
		Logger(const Logger&) = delete;
		Logger& operator=(const Logger&) = delete;
		Logger(Logger&&) = delete;
		Logger& operator=(Logger&&) = delete;

		static Logger& getInstance();

		void log(const std::string& message, bool rateLimit = false);

		void createTask();

		template<typename... Args>
		static std::string formatLog(const char* fmt, Args... args) {
			char buffer[QUEUE_ITEM_SIZE];
			snprintf(buffer, sizeof(buffer), fmt, args...);
			return std::string(buffer);
		}

		void incrementRateLimitCounter();
};
