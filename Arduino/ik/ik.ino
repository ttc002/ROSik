#include <IRremote.hpp>  // именно .hpp
#define IR_RECEIVE_PIN 15

void setup() {
  Serial.begin(115200);
  delay(200);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);  // Инициализация
  Serial.println("IR Receiver ready");
}

void loop() {
  if (IrReceiver.decode()) {
    IrReceiver.printIRResultShort(&Serial);  // Краткий вывод
    IrReceiver.resume();  // Готов к следующему сигналу
  }
}
