#include "DHT_ESP32.h"

void setup() {
  Serial.begin(115200);
  initTemp();
  tasksEnabled = true;
}

void loop() {  
  Serial.println(" T:" + String(newValues.temperature) + " H:" + String(newValues.humidity));
  delay(1000);
}
