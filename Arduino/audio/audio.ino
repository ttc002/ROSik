#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"

#define BCLK_PIN 2
#define LRC_PIN 13
#define DATA_PIN 22

const char* ssid = "lelka";
const char* password = "78914040";
const int udp_port = 12345;

WiFiUDP udp;

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println("WiFi connected: " + WiFi.localIP().toString());
}

void setup_i2s() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
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
  setup_wifi();
  setup_i2s();
  udp.begin(udp_port);
}

void loop() {
  uint8_t buffer[1024];
  int len = udp.parsePacket();
  if (len > 0) {
    int n = udp.read(buffer, sizeof(buffer));
    size_t bytes_written;
    i2s_write(I2S_NUM_0, buffer, n, &bytes_written, portMAX_DELAY);
  }
}
