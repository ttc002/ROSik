#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <PNGdec.h>

// Пины
#define TFT_CS    D2
#define TFT_DC    D1
#define TFT_RST   D0
#define SD_CS     D8

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
PNG png;

// Смещение (если нужно)
const int offsetX = 0;
const int offsetY = 0;

int drawPNGCallback(PNGDRAW *pDraw) {
  uint16_t lineBuf[pDraw->iWidth];
  png.getLineAsRGB565(pDraw, lineBuf, PNG_RGB565_BIG_ENDIAN, 0xFFFF);
  tft.drawRGBBitmap(offsetX, pDraw->y + offsetY, lineBuf, pDraw->iWidth, 1);
  return 1;
}

// Стандартные функции для работы с SD через File*
void *myOpen(const char *filename, int32_t *size) {
  File f = SD.open(filename);
  if (!f) return NULL;
  *size = f.size();
  File *pf = new File(f);  // выделяем копию, чтобы сохранить состояние
  return (void *)pf;
}

void myClose(void *handle) {
  File *pf = (File *)handle;
  if (pf) {
    pf->close();
    delete pf;
  }
}

int32_t myRead(PNGFILE *handle, uint8_t *buffer, int32_t length) {
  File *pf = (File *)handle->fHandle;
  return pf->read(buffer, length);
}

int32_t mySeek(PNGFILE *handle, int32_t position) {
  File *pf = (File *)handle->fHandle;
  return pf->seek(position) ? position : -1;
}

void setup() {
  Serial.begin(115200);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed!");
    while (1);
  }

  const char *filename = "/test.png";

  int rc = png.open(
    filename,
    myOpen,
    myClose,
    myRead,
    mySeek,
    drawPNGCallback
  );

  if (rc != PNG_SUCCESS) {
    Serial.print("PNG decode failed: ");
    Serial.println(rc);
    return;
  }

  Serial.println("PNG decode start");
  png.decode(NULL, 0);
  png.close();
  Serial.println("PNG decode done");
}

void loop() {
  // ничего
}
