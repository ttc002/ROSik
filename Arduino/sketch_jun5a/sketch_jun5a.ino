#include <WiFi.h>
#include <WiFiServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

const char* ssid = "lelka";
const char* password = "78914040";
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 17
#define SD_CS 4

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);



WiFiServer server(4210);
WiFiClient client;

const int IMG_WIDTH = 160;
const int IMG_HEIGHT = 128;
uint16_t buf[IMG_WIDTH * IMG_HEIGHT];

void setup() {
  Serial.begin(115200);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting...");
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("📡 TCP server ready on port 4210");
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
  }
}

void onImageReceived() {


  for (int y = 0; y < IMG_HEIGHT; y++) {
    for (int x = 0; x < IMG_WIDTH; x++) {
      uint16_t color = buf[y * IMG_WIDTH + x];
      //uint16_t pixel = (packetBuffer[i + 1] << 8) | packetBuffer[i];

      tft.drawPixel(x, y, color);
    }
  }
}
