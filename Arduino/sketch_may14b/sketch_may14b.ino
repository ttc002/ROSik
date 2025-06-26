#include <SPI.h>
#include <SD.h>

#define SD_CS D1  // Пин CS для SD-карты

void setup() {
  Serial.begin(115200);
  
  // Инициализация SPI
  SPI.begin();  // Явно инициализируем SPI
  if (!SD.begin(SD_CS)) {
    Serial.println("Ошибка инициализации SD-карты!");
    while (true);  // Ожидание, пока не будет исправлено
  } else {
    Serial.println("SD карта успешно инициализирована.");
  }
}
void loop(){}