#include <WiFi.h>
#include <WiFiServer.h>
#include "driver/i2s.h"

#define BCLK_PIN 0
#define LRC_PIN 21
#define DATA_PIN 22

const char* ssid = "lelka";
const char* password = "78914040";


void setup() {
  Serial.begin(115200);
  setup_i2s();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting...");
  }
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("TCP client connected.");
    uint8_t buffer[1024];
    while (client.connected()) {
      int n = client.read(buffer, sizeof(buffer));
      if (n > 0) {
        Serial.print("Received TCP audio bytes: ");
        Serial.println(n);
        // Debug: print first few bytes
        for (int i = 0; i < n && i < 8; ++i) {
          Serial.print(buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        size_t bytes_written;
        i2s_write(I2S_NUM_0, buffer, n, &bytes_written, portMAX_DELAY);
      }
      delay(1);
    }
    client.stop();
    Serial.println("TCP client disconnected.");
  }

}

