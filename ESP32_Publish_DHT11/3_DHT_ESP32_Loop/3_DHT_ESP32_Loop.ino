#include "DHT_ESP32.h"

void setup() {
  Serial.begin(115200);
  initTemp();
  tasksEnabled = true;
}

void loop() {  
  Serial.println(" T:" + String(currentTemperature) + " H:" + String(currentHumidity));
  delay(1000);
}
