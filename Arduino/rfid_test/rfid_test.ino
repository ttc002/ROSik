#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10
#define RST_PIN 9

MFRC522 mfrc522(SS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Init RC522 on Arduino Uno"));

  SPI.begin();           
  mfrc522.PCD_Init();    

  byte version = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.print(F("RC522 Version: 0x"));
  Serial.println(version, HEX);

  if (version == 0x00 || version == 0xFF) {
    Serial.println(F("ERROR: RC522 not responding!"));
  } else {
    Serial.println(F("RC522 detected. Try scanning a card."));
  }
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial()) return;

  Serial.print(F("Card UID: "));
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
  }
  Serial.println();
  delay(1000);
}
