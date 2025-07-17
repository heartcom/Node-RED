#include "DHT_ESP32.h"

DHTesp dht;
Ticker tempTicker;
TaskHandle_t tempTaskHandle = NULL;
bool tasksEnabled = false;
int dhtPin = 17;

float currentTemperature = 0.0;
float currentHumidity = 0.0;

bool initTemp() {
  dht.setup(dhtPin, DHTesp::DHT11);
  Serial.println("DHT initiated");

  xTaskCreatePinnedToCore(tempTask, "tempTask", 4000, NULL, 5, &tempTaskHandle, 1);
  if (tempTaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } else {
    tempTicker.attach(5, triggerGetTemp);
  }
  return true;
}

void triggerGetTemp() {
  if (tempTaskHandle != NULL) {
    xTaskResumeFromISR(tempTaskHandle);
  }
}

void tempTask(void *pvParameters) {
  Serial.println("tempTask loop started");
  while (1) {
    if (tasksEnabled) {
      getTemperature();
    }
    vTaskSuspend(NULL);
  }
}

bool getTemperature() {
  TempAndHumidity newValues = dht.getTempAndHumidity();
  if (dht.getStatus() != 0) {
    Serial.println("DHT11 error status: " + String(dht.getStatusString()));
    return false;
  }
  //Serial.println(" T:" + String(newValues.temperature) + " H:" + String(newValues.humidity));

  currentTemperature = newValues.temperature;
  currentHumidity = newValues.humidity;
  
  return true;
}
