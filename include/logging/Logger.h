#pragma once
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <WiFiUdp.h>
#include <WiFi.h>
#include "WifiCredentials.h"

class Logger {
	private:
		Logger();
		~Logger() = default;

		void setupWifi();

		bool sendUdp();

		static void loggingTask(void* params);

		//Queue items
		static const uint16_t QUEUE_LENGTH = 1000;
		static const uint8_t QUEUE_ITEM_SIZE = 50; //bytes
		static uint8_t ucQueueStorageArea[QUEUE_LENGTH * QUEUE_ITEM_SIZE];
		static StaticQueue_t xStaticQueue;
		static QueueHandle_t logQueue;

		WiFiUDP udp;

	public:
		Logger(const Logger&) = delete;
		Logger& operator=(const Logger&) = delete;
		Logger(Logger&&) = delete;
		Logger& operator=(Logger&&) = delete;

		static Logger& getInstance();

		void log(const std::string& message);

		void startTask();

		template<typename... Args>
		static std::string formatLog(const char* fmt, Args... args) {
			char buffer[QUEUE_ITEM_SIZE];
			snprintf(buffer, sizeof(buffer), fmt, args...);
			return std::string(buffer);
		}

};
