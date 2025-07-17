#ifndef DHT_ESP32_H
#define DHT_ESP32_H

#include "DHTesp.h"
#include <Ticker.h>
#include <Arduino.h>

extern TaskHandle_t tempTaskHandle;
extern bool tasksEnabled;
extern int dhtPin;

extern float currentTemperature;
extern float currentHumidity;


bool initTemp();              // 초기화 함수
bool getTemperature();        // 온도 읽는 함수
void triggerGetTemp();        // Ticker 콜백
void tempTask(void *pvParameters);  // 태스크 함수

#endif
