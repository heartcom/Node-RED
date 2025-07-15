#include "DHT_ESP32.h"

void setup() {
  Serial.begin(115200);
  initTemp();
  tasksEnabled = true;
}

void loop() {  
  delay(1000);
}
