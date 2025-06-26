#include "fonts.h"
void loadFontIndex(String FONT_DIR) {
  String path = String(FONT_DIR) + "/index.json";
  File file = SD.open(path.c_str());
  if (!file) {
    Serial.println("❌ Не удалось открыть index.json");
    return;
  }

  DeserializationError error = deserializeJson(fontIndex, file);
  if (error) {
    Serial.print("❌ Ошибка JSON: ");
    Serial.println(error.c_str());
  }

  file.close();
}

void drawChar(uint16_t codepoint, String FONT_DIR, uint16_t color) {
  if (!fontIndex.containsKey(String(codepoint))) {
    Serial.print("❌ Нет символа: ");
    Serial.println(codepoint);
    return;
  }

  JsonObject meta = fontIndex[String(codepoint)];
  String filename = meta["file"];
  int width = meta["width"];
  int height = meta["height"];
  int baseline = meta["baseline"];


  String path = String(FONT_DIR) + "/" + filename;
  File file = SD.open(path.c_str());
  if (!file) {
    Serial.print("❌ Не удалось открыть: ");
    Serial.println(path);
    return;
  }

  int byte_width = (width + 7) / 8;
  uint8_t row[byte_width];

  int y0 = cursor_y - baseline;

  for (int y = 0; y < height; y++) {
    file.read(row, byte_width);
    for (int x = 0; x < width; x++) {
      int byte_index = x / 8;
      int bit_index = 7 - (x % 8);
      bool pixel_on = row[byte_index] & (1 << bit_index);
      if (pixel_on) {
        tft.drawPixel(cursor_x + x, y0 + y, color);
      }
    }
  }

  cursor_x += width + 1;
  file.close();
}

void drawText(const char* text, String FONT_DIR, uint16_t color) {
  while (*text) {
    uint16_t codepoint;

    if ((uint8_t)*text < 0x80) {
      // ASCII символ
      uint8_t bit = (uint8_t)*text;
      if (bit == 0x20) {
        cursor_x += 7;
        text++;
        continue;
      } else {
        codepoint = (uint8_t)*text++;
      }
    } else if (((uint8_t)*text & 0xE0) == 0xC0 && *(text + 1)) {
      // Двухбайтовый UTF-8 символ
      uint8_t high = (uint8_t)*text;
      uint8_t low = (uint8_t)*(text + 1);

      if (high == 0xD0) {
        if (low >= 0x90 && low <= 0xBF) {
          // А–п: U+0410–U+043F
          codepoint = 0x0410 + (low - 0x90);
        } else if (low == 0x81) {
          // Ё: U+0401
          codepoint = 0x0401;
        } else {
          codepoint = '?';
        }
      } else if (high == 0xD1) {
        if (low >= 0x80 && low <= 0x8F) {
          // р–я: U+0440–U+044F
          codepoint = 0x0440 + (low - 0x80);
        } else if (low == 0x91) {
          // ё: U+0451
          codepoint = 0x0451;
        } else {
          codepoint = '?';
        }
      } else {
        codepoint = '?';
      }
      text += 2;
    } else {
      // Неподдерживаемый символ
      codepoint = '?';
      text++;
    }

    drawChar(codepoint, FONT_DIR, color);
  }
}

uint16_t hexToRGB565(const String& hexColor) {
  String hex = hexColor;
  if (hex.startsWith("#")) hex.remove(0, 1);
  if (hex.length() != 6) return 0;

  uint8_t r = strtoul(hex.substring(0, 2).c_str(), nullptr, 16);
  uint8_t g = strtoul(hex.substring(2, 4).c_str(), nullptr, 16);
  uint8_t b = strtoul(hex.substring(4, 6).c_str(), nullptr, 16);

  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
