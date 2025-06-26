
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "fonts.h"
// Настройка дисплея
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 17
#define SD_CS 4

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// SD карта


// Глобальный JSON объект
StaticJsonDocument<16384> fontIndex;



// Точка вывода текста
int cursor_x = 0;
int cursor_y = 20;


#include <WiFi.h>  // для ESP32

const char *ssid = "lelka";
const char *password = "78914040";

void setup() {
  Serial.begin(115200);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);

  if (!SD.begin(SD_CS)) {
    Serial.println("❌ SD не инициализирована");
    while (1);
  }
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  // ← здесь IP-адрес ESP32
  tft.setCursor(45, 145);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println(WiFi.localIP());
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
}

void processCommand(String command) {
  command.trim();
  if (command.equalsIgnoreCase("PING")) {
    Serial.println("DISPLAY");
  }

  else if (command.startsWith("TEXT")) {
    // <TEXT> <FINT DIR> <X> <Y> <HEX COLOR>

    String text, dir, color;
    uint8_t x, y;
    parseTextNumCommand(command, &text, &dir, &x, &y, &color);
    cursor_x = x;
    cursor_y = y;
    fontIndex.clear();
    loadFontIndex(dir);


    drawText(text.c_str(), dir, hexToRGB565(color));
  }

  else if (command.equalsIgnoreCase("CLEAR_SCREEN")) {
    tft.fillScreen(ST77XX_BLACK);

  } else {
    Serial.println("ERROR: Unknown command");
  }
}

void parseTextNumCommand(String command, String *text, String *dir, uint8_t *x, uint8_t *y, String *color) {
  int index = command.indexOf(' ');
  if (index == -1) {
    Serial.println("ERROR: Invalid command");
    return;
  }
  String params = command.substring(index + 1);
  int idx1 = params.indexOf(' ');
  int idx2 = params.indexOf(' ', idx1 + 1);
  int idx3 = params.indexOf(' ', idx2 + 1);
  int idx4 = params.indexOf(' ', idx3 + 1);
  if (idx1 == -1 || idx2 == -1 || idx3 == -1 || idx4 == -1) {
    Serial.println("ERROR: Invalid parameters");
    return;
  }
  String aStr = params.substring(0, idx1);
  String bStr = params.substring(idx1 + 1, idx2);
  String cStr = params.substring(idx2 + 1, idx3);
  String dStr = params.substring(idx3 + 1, idx4);
  String eStr = params.substring(idx4 + 1);

  *text = aStr;
  *dir = bStr;
  *x = (uint8_t)cStr.toInt();
  *y = (uint8_t)dStr.toInt();
  *color = eStr;
}




