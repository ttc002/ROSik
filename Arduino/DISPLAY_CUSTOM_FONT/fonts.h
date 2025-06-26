
#include <SPI.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#ifndef fonts
#define fonts

void loadFontIndex(String FONT_DIR);
void drawChar(uint16_t codepoint, String FONT_DIR, uint16_t color) ;
void drawText(const char* text, String FONT_DIR, uint16_t color);
uint16_t hexToRGB565(const String& hexColor);

extern StaticJsonDocument<16384> fontIndex;
extern Adafruit_ST7735 tft;
extern int cursor_x;
extern int cursor_y;

#endif
