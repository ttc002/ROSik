#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif



#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#define TFT_CS 5
#define TFT_DC 4
#define TFT_RST 27

#define BCLK_PIN 0
#define LRC_PIN 21
#define DATA_PIN 22

const int IMG_WIDTH = 160;
const int IMG_HEIGHT = 128;
uint16_t buf[IMG_WIDTH * IMG_HEIGHT];
const int music_port = 2408;
const int image_port = 4210;
const char* ssid = "robotx";
const char* password = "78914040";


unsigned long idleCounter = 0;
unsigned long lastMillis = 0;
float cpuLoad = 0;


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

WiFiUDP udp;
WiFiServer server(image_port);
WiFiClient client;

void setup_i2s() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 22050,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = BCLK_PIN,
    .ws_io_num = LRC_PIN,
    .data_out_num = DATA_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

void setup() {
  Serial.begin(115200);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  setup_i2s();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting...");
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  udp.begin(music_port);

  lastMillis = millis();
}

void loop() {
  client = server.available();
  if (client) {
    Serial.println("✅ Client connected");
    int total = 0;
    uint8_t* byte_buf = (uint8_t*)buf;
    int size = IMG_WIDTH * IMG_HEIGHT * 2;

    while (total < size) {
      if (client.available()) {
        int r = client.read(byte_buf + total, size - total);
        if (r > 0) total += r;
      }
    }

    Serial.println("✅ Image fully received!");
    onImageReceived();
    client.stop();
    Serial.println("✅ Image Shown!");
  }

  uint8_t buffer[1024];
  int len = udp.parsePacket();
  if (len > 0) {
    int n = udp.read(buffer, sizeof(buffer));
    size_t bytes_written;
    i2s_write(I2S_NUM_0, buffer, n, &bytes_written, portMAX_DELAY);
  }

  float cpuTemp = (temprature_sens_read() - 32) / 1.8;  // Конвертация в °C
  uint32_t totalRAM = ESP.getHeapSize();
  uint32_t freeRAM = ESP.getFreeHeap();
  uint32_t usedRAM = totalRAM - freeRAM;

  idleCounter++;
  if (millis() - lastMillis >= 1000) {
    unsigned long count = idleCounter;
    idleCounter = 0;

    // 100% - если loop пустой; если вы туда что-то нагружаете - будет меньше
    static unsigned long maxIdle = 0;
    if (count > maxIdle) maxIdle = count;
    cpuLoad = 100.0 * (1.0 - (float)count / maxIdle);

    Serial.println("=== ESP32 Task Manager ===");
    Serial.printf("CPU Temperature: %.2f °C\n", cpuTemp);
    Serial.printf("CPU Load (relative): %.1f %%\n", cpuLoad < 0 ? 0 : (cpuLoad > 100 ? 100 : cpuLoad));
    Serial.printf("RAM Used: %lu / %lu bytes (%.1f %%)\n", usedRAM, totalRAM, 100.0 * usedRAM / totalRAM);
    Serial.println("==========================\n");

    lastMillis = millis();
  }
}
/*
void onImageReceived() {
  for (int y = 0; y < IMG_HEIGHT; y++) {
    for (int x = 0; x < IMG_WIDTH; x++) {
      uint16_t color = buf[y * IMG_WIDTH + x];
      //uint16_t pixel = (packetBuffer[i + 1] << 8) | packetBuffer[i];

      tft.drawPixel(x, y, color);
    }
  }
}
*/
void onImageReceived() {
  tft.startWrite();  // Включаем ускоренный режим записи
  for (int y = 0; y < IMG_HEIGHT; y++) {
    tft.setAddrWindow(0, y, IMG_WIDTH, 1);                  // Устанавливаем окно строки
    tft.writePixels(&buf[y * IMG_WIDTH], IMG_WIDTH, true);  // true — big-endian (обычно для 16-битных цветов)
  }
  tft.endWrite();  // Выключаем ускоренный режим записи
}