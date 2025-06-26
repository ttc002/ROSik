/****************************************************************
 *  ESP32  –  Twist-PID + Encoder-Alignment
 *
 *  • КОМАНДНЫЙ СЛОЙ
 *    /setSpeed?l=&r=      – целевые скорости колёс, мм/с
 *    /setCoeff?kp&ki&kd&kff=
 *    /state               – полное состояние (duty, enc, speed, target, odom)
 *    /resetEnc /resetOdom
 *
 *  • ДВИГАТЕЛИ
 *    GPIO 33/32  – левый H-мост   (A-канал = вперёд)
 *    GPIO 26/25  – правый H-мост
 *
 *  • СЧЁТЧИКИ PCNT
 *    UNIT0 = 18 (A) / 19 (B)  – правый энкодер
 *    UNIT1 = 34 (A) / 35 (B)  – левый  энкодер
 *
 *  • ОДОМЕТРИЯ
 *    Ø 44 мм, база 96 мм, 2930 тиков / оборот
 *
 *  • PID
 *    kp/ki/kd – обычный PID c feed-forward kff
 *    P-контур "прямо" (`kAlign`) — сравнивает абсолютный ход левого и правого
 *    колеса и корректирует target, если робот должен ехать строго прямо
 *    (или крутиться на месте)
 ****************************************************************/

#include <WiFi.h>
#include <WiFiServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "driver/pcnt.h"
#include <math.h>
#include "driver/i2s.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#define L_A 33
#define L_B 32
#define R_A 25
#define R_B 26
#define ENC_R_A 34
#define ENC_R_B 35
#define ENC_L_A 15
#define ENC_L_B 2

#define BAT ADC1_CHANNEL_3  //GPIO 39

#define BCLK_PIN 0
#define LRC_PIN 21
#define DATA_PIN 22

#define TFT_CS 5
#define TFT_DC 4
#define TFT_RST 27

uint32_t bat_timer = 0;
int bat_check_time = 1000;
float voltage = 0;

const int IMG_WIDTH = 160;
const int IMG_HEIGHT = 128;
uint16_t buf[IMG_WIDTH * IMG_HEIGHT];


/* ────── Wi-Fi ───── */
constexpr char SSID[] = "robotx";
constexpr char PASS[] = "78914040";
const int music_port = 12345;
const int image_port = 4210;

/* ────── mechanics ───── */
constexpr float WHEEL_D = 0.044f;  // m
constexpr float BASE_L = 0.096f;   // m
constexpr int TICKS_REV = 2930;
constexpr float MM_PER_TICK = WHEEL_D * M_PI * 1000.0f / TICKS_REV;

volatile uint8_t dutyLA, dutyLB, dutyRA, dutyRB;
volatile int32_t encTotL = 0, encTotR = 0, prevEncL = 0, prevEncR = 0;
volatile float speedL = 0, speedR = 0, tgtL = 0, tgtR = 0;
volatile float odomX = 0, odomY = 0, odomTh = 0;
volatile float kp = 2.0f, ki = 3.5f, kd = 0.01f, kff = 0.75f;

bool alignMode = false;  // включён ли «режим прямо»
float alignSign = 1;     //  +1 прямой ход, −1 разворот
int32_t alignRefL = 0, alignRefR = 0;
constexpr float kAlign = 1;  // (мм ⇒ мм/с)

inline void pwm(uint8_t pin, uint8_t d) {
  analogWrite(pin, d);
  switch (pin) {
    case L_A: dutyLA = d; break;
    case L_B: dutyLB = d; break;
    case R_A: dutyRA = d; break;
    case R_B: dutyRB = d; break;
  }
}
inline void stopMotors() {
  pwm(L_A, 0);
  pwm(L_B, 0);
  pwm(R_A, 0);
  pwm(R_B, 0);
}

inline void pcntInit(pcnt_unit_t u, gpio_num_t a, gpio_num_t b) {
  pcnt_config_t c{};
  c.unit = u;
  c.channel = PCNT_CHANNEL_0;
  c.pulse_gpio_num = a;
  c.ctrl_gpio_num = b;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;
  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;
  c.counter_h_lim = 32767;
  c.counter_l_lim = -32768;
  pcnt_unit_config(&c);
  pcnt_set_filter_value(u, 100);
  pcnt_filter_enable(u);
  pcnt_counter_clear(u);
  pcnt_counter_resume(u);
}
inline int16_t snap(pcnt_unit_t u) {
  int16_t v;
  pcnt_get_counter_value(u, &v);
  pcnt_counter_clear(u);
  return v;
}

/* ────── API ───── */
AsyncWebServer server(80);
WiFiServer musicServer(music_port);
WiFiServer imageServer(image_port);
WiFiClient imageClient;


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);


void routes() {
  server.on("/state", HTTP_GET, [](auto* r) {
    char js[600];
    snprintf(js, sizeof(js),
             "{\"duty\":{\"L_A\":%u,\"L_B\":%u,\"R_A\":%u,\"R_B\":%u,\"voltage\":%.3f},"
             "\"enc\":{\"left\":%ld,\"right\":%ld},"
             "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
             "\"target\":{\"left\":%.1f,\"right\":%.1f},"
             "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"th\":%.3f}}",
             dutyLA, dutyLB, dutyRA, dutyRB, voltage,
             encTotL, encTotR, speedL, speedR, tgtL, tgtR,
             odomX, odomY, odomTh);
    r->send(200, "application/json", js);
  });

  /* ---------- setSpeed ---------- */
  server.on("/setSpeed", HTTP_GET, [](auto* r) {
    float newL = tgtL, newR = tgtR;
    if (r->hasParam("l")) newL = r->getParam("l")->value().toFloat();
    if (r->hasParam("r")) newR = r->getParam("r")->value().toFloat();
    tgtL = newL;
    tgtR = newR;

    /* — включаем alignment, если |vL| ≈ |vR| и не ноль — */
    if (fabs(fabs(newL) - fabs(newR)) < 1 && fabs(newL) > 1) {
      alignMode = true;
      alignSign = (newL * newR >= 0) ? 1.0f : -1.0f;
      alignRefL = encTotL;
      alignRefR = encTotR;
    } else alignMode = false;

    r->send(200, "text/plain", "ok");
  });

  server.on("/setCoeff", HTTP_GET, [](auto* r) {
    if (r->hasParam("kp")) kp = r->getParam("kp")->value().toFloat();
    if (r->hasParam("ki")) ki = r->getParam("ki")->value().toFloat();
    if (r->hasParam("kd")) kd = r->getParam("kd")->value().toFloat();
    if (r->hasParam("kff")) kff = r->getParam("kff")->value().toFloat();
    r->send(200, "text/plain", "ok");
  });

  server.on("/resetEnc", HTTP_GET, [](auto* r) {
    encTotL = encTotR = prevEncL = prevEncR = 0;
    speedL = speedR = 0;
    alignMode = false;
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    r->send(200, "text/plain", "enc reset");
  });

  server.on("/resetOdom", HTTP_GET, [](auto* r) {
    odomX = odomY = odomTh = 0;
    r->send(200, "text/plain", "odom reset");
  });

  server.on("/", HTTP_GET, [](auto* r) {
    r->send(200, "text/plain",
            "/state /setSpeed /setCoeff /resetEnc /resetOdom");
  });
}

/* ───── PWM map (A-вперёд) ───── */
inline void mapPWM(float val, uint8_t& a, uint8_t& b) {
  if (val >= 0) {
    a = uint8_t(constrain(val, 0, 255));
    b = 0;
  } else {
    a = 0;
    b = uint8_t(constrain(-val, 0, 255));
  }
}

/* ───── PID ───── */
void pidTask() {
  static uint32_t tPrev = millis();
  static float iL = 0, iR = 0, ePrevL = 0, ePrevR = 0;
  static float lastTgtL = 0, lastTgtR = 0;

  float dt = (millis() - tPrev) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  tPrev = millis();
  if (tgtL != lastTgtL || fabs(tgtL) < 1) {
    iL = 0;
    ePrevL = 0;
  }
  if (tgtR != lastTgtR || fabs(tgtR) < 1) {
    iR = 0;
    ePrevR = 0;
  }
  lastTgtL = tgtL;
  lastTgtR = tgtR;

  /* ----- alignment correction (P-only) ----- */
  float corr = 0;
  if (alignMode) {
    int32_t dL = encTotL - alignRefL, dR = encTotR - alignRefR;
    float diff = (dL - alignSign * dR) * MM_PER_TICK;  // мм
    corr = kAlign * diff;                              // мм/с
  }
  float tgtCorrL = tgtL - corr;
  float tgtCorrR = tgtR + alignSign * corr;

  /* ----- обычный PID ----- */
  float eL = tgtCorrL - speedL, eR = tgtCorrR - speedR;
  iL += eL * dt;
  iR += eR * dt;
  const float ILIM = 300;
  iL = constrain(iL, -ILIM, ILIM);
  iR = constrain(iR, -ILIM, ILIM);
  float dL = (eL - ePrevL) / dt, dR = (eR - ePrevR) / dt;
  ePrevL = eL;
  ePrevR = eR;
  float outL = kp * eL + ki * iL + kd * dL + kff * tgtCorrL;
  float outR = kp * eR + ki * iR + kd * dR + kff * tgtCorrR;
  outL = constrain(outL, -255, 255);
  outR = constrain(outR, -255, 255);

  uint8_t aL, bL, aR, bR;
  mapPWM(outL, aL, bL);
  mapPWM(outR, aR, bR);
  pwm(L_A, aL);
  pwm(L_B, bL);
  pwm(R_A, aR);
  pwm(R_B, bR);
}

void setup_i2s_streaming() {
  i2s_config_t i2s_config_tx = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 22050,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = BCLK_PIN,
    .ws_io_num = LRC_PIN,
    .data_out_num = DATA_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config_tx, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setup_adc_bat() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(BAT, ADC_ATTEN_DB_11);
}
float check_bat() {
  float voltage = 0;
  int i = 0;
  for (i = 0; i <= 10; i++) {
    int raw = adc1_get_raw(BAT);
    voltage += raw * 0.00235f;  // калибруйте под свою схему!
    delay(1);
  }
  return voltage / i;
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

/* ───── SETUP / LOOP ───── */
void setup() {
  Serial.begin(115200);
  setup_i2s_streaming();  // Для музыки
  setup_adc_bat();        // Для чтения BAT

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  pinMode(L_A, OUTPUT);
  pinMode(L_B, OUTPUT);
  pinMode(R_A, OUTPUT);
  pinMode(R_B, OUTPUT);
  stopMotors();
  pcntInit(PCNT_UNIT_0, (gpio_num_t)ENC_R_A, (gpio_num_t)ENC_R_B);
  pcntInit(PCNT_UNIT_1, (gpio_num_t)ENC_L_A, (gpio_num_t)ENC_L_B);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }
  Serial.println("\nIP " + WiFi.localIP().toString());
  routes();
  server.begin();
  musicServer.begin();
  imageServer.begin();

}


void loop() {
  static uint32_t t10 = 0, t20 = 0, tPID = 0;
  uint32_t ms = millis();

  if (ms - t10 >= 10) {
    t10 = ms;
    int16_t dR = snap(PCNT_UNIT_0), dL = snap(PCNT_UNIT_1);
    encTotR += dR;
    encTotL += dL;
    float sR = dR * MM_PER_TICK / 1000, sL = dL * MM_PER_TICK / 1000;
    float ds = 0.5f * (sR + sL), dth = (sR - sL) / BASE_L, mid = odomTh + 0.5f * dth;
    odomX += ds * cosf(mid);
    odomY += ds * sinf(mid);
    odomTh += dth;
    if (odomTh > M_PI) odomTh -= 2 * M_PI;
    if (odomTh < -M_PI) odomTh += 2 * M_PI;
  }

  if (ms - t20 >= 20) {
    float dt = (ms - t20) * 0.001f;
    t20 = ms;
    speedL = (encTotL - prevEncL) * MM_PER_TICK / dt;
    speedR = (encTotR - prevEncR) * MM_PER_TICK / dt;
    prevEncL = encTotL;
    prevEncR = encTotR;
  }

  if (ms - tPID >= 20) {
    tPID = ms;
    pidTask();
  }

  if (ms - bat_timer >= bat_check_time) {
    voltage = check_bat();
  }


  WiFiClient client = musicServer.available();
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



  imageClient = imageServer.available();
  if (imageClient) {
    Serial.println("✅ Client connected");
    int total = 0;
    uint8_t* byte_buf = (uint8_t*)buf;
    int size = IMG_WIDTH * IMG_HEIGHT * 2;

    while (total < size) {
      if (imageClient.available()) {
        int r = imageClient.read(byte_buf + total, size - total);
        if (r > 0) total += r;
      }
    }

    Serial.println("✅ Image fully received!");
    onImageReceived();
    imageClient.stop();
  }
}
