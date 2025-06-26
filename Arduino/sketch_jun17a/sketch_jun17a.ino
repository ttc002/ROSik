#include <Arduino.h>

volatile unsigned long idleCounter = 0;
hw_timer_t *timer = NULL;

void IRAM_ATTR onTimer(){
  idleCounter++;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  timer = timerBegin(1000); // 1 кГц (1 мс)
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1, true, 1); // исправлено: 4 аргумента!
  timerStart(timer);
}

void loop() {
  static unsigned long prevIdleCounter = 0;
  static unsigned long lastMillis = 0;

  float cpuTemp = temperatureRead();
  uint32_t totalRAM = ESP.getHeapSize();
  uint32_t freeRAM  = ESP.getFreeHeap();
  uint32_t usedRAM  = totalRAM - freeRAM;

  unsigned long currIdleCounter = idleCounter;
  unsigned long idleTicks = currIdleCounter - prevIdleCounter;
  prevIdleCounter = currIdleCounter;

  float cpuLoad = 100.0 * (1.0 - (float)idleTicks / 1000.0);

  if (millis() - lastMillis > 1000) {
    lastMillis = millis();
    Serial.println("=== ESP32 Task Manager ===");
    Serial.printf("CPU Temperature: %.2f °C\n", cpuTemp);
    Serial.printf("CPU Load (relative): %.1f %%\n", cpuLoad < 0 ? 0 : (cpuLoad > 100 ? 100 : cpuLoad));
    Serial.printf("RAM Used: %lu / %lu bytes (%.1f %%)\n", usedRAM, totalRAM, 100.0 * usedRAM / totalRAM);
    Serial.println("==========================\n");
  }

  delay(100);
  
}