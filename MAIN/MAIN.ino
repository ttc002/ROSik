#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <driver/uart.h>
#include "esp_task_wdt.h"
#include "driver/pcnt.h"
#include <math.h>
#include "driver/i2s.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
/* ---------- Wi-Fi ---------- */
constexpr char WIFI_SSID[] = "robotx";
constexpr char WIFI_PASS[] = "78914040";

/* ---------- Пины и параметры робота ---------- */
// Двигатели (H-мосты)//
#define L_A 33
#define L_B 32
#define R_A 25
#define R_B 26

/* ── пины энкодеров ─────────────────────────────────────────*/
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

Adafruit_ST7735 tft2 = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7735 tft = Adafruit_ST7735(13, 12, 14);

uint32_t bat_timer = 0;
int bat_check_time = 1000;
float voltage = 0;

const int IMG_WIDTH = 160;
const int IMG_HEIGHT = 128;
uint16_t buf[IMG_WIDTH * IMG_HEIGHT];

const int music_port = 2408;
const int image_port = 4210;

// Описание механики
constexpr float WHEEL_D = 0.044f;                                    // диаметр колеса, м
constexpr float BASE_L = 0.100f;                                     // база (расстояние между колёсами), м
constexpr int TICKS_REV = 2930;                                      // количество тиков энкодера на оборот колеса
constexpr float MM_PER_TICK = WHEEL_D * M_PI * 1000.0f / TICKS_REV;  // мм за один тик

/* ---------- Глобальные переменные состояния ---------- */
volatile uint8_t dutyLA = 0, dutyLB = 0, dutyRA = 0, dutyRB = 0;  // текущие ШИМ для H-мостов
volatile int32_t encTotL = 0, encTotR = 0;                        // накопленные тики энкодера
volatile int32_t prevEncL = 0, prevEncR = 0;
volatile float speedL = 0.0f, speedR = 0.0f;                   // измеренные скорости, мм/с
volatile float tgtL = 0.0f, tgtR = 0.0f;                       // целевые скорости, мм/с
volatile float odomX = 0.0f, odomY = 0.0f, odomTh = 0.0f;      // одометрия: положение робота (м, м, рад)
volatile float kp = 1.0f, ki = 0.8f, kd = 0.02f, kff = 0.25f;  // PID коэффициенты

volatile uint32_t lastCmdMs = 0;

// Режим выравнивания (для прямолинейного движения)
bool alignMode = false;
float alignSign = 1.0f;
int32_t alignRefL = 0, alignRefR = 0;
constexpr float kAlign = 1.0f;  // коэффициент P-контроллера выравнивания (мм -> мм/с)

/* ---------- Настройки лидара ---------- */
#define LIDAR_RX_PIN 17  // lidar TX -> ESP RX
#define LIDAR_TX_PIN 16  // lidar RX (не обязательно использовать)
#define LIDAR_BAUD 115200

static const uint8_t HDR[4] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t BODY_LEN = 32;      // байт в теле пакета лидара (8 точек по 4 байта)
static const uint8_t INTENSITY_MIN = 2;  // минимальное значение интенсивности для учёта точки
static const float MAX_SPREAD_DEG = 20.0;
#define FRAME_LEN 20   // длина упакованных данных на каждую порцию (2 байта нач.угол, 2 байта кон.угол, 8*2 байта дистанции)
#define MAX_FRAMES 64  // макс. количество порций на один полный оборот (64*20 ≈ 1280 байт)
static uint8_t scanBuf[MAX_FRAMES * FRAME_LEN];
static uint8_t *wr = scanBuf;
static uint8_t frameCount = 0;
static float prevStartAngle = -1;

/* ---------- Статистика ---------- */
volatile uint32_t stat_rx = 0;  // принятых 20-байтных пакетов от LDS
volatile uint32_t stat_tx = 0;  // переданных полных сканов по WS


const int graphWidth = 160;
const int graphHeight = 90;
const int graphX = 0;
const int graphY = 30;

#define GRAPH_WIDTH 160
#define GRAPH_HEIGHT 100
#define GRAPH_X 0
#define GRAPH_Y 0

uint8_t cpuLoadHistory[GRAPH_WIDTH] = { 0 };

unsigned long idleCounter = 0;
unsigned long lastMillis = 0;
float cpuLoad = 0;








/* ---------- Вспомогательные функции ---------- */
inline float decodeAngle(uint16_t raw) {
  // Декодирует угол (двухбайтное значение) из формата LDS
  float a = (raw - 0xA000) / 64.0f;
  if (a < 0) a += 360.0f;
  else if (a >= 360) a -= 360.0f;
  return a;
}
bool readBytes(HardwareSerial &serial, uint8_t *dst, size_t n, uint32_t timeout = 300) {
  uint32_t t0 = millis();
  for (size_t i = 0; i < n; ++i) {
    while (!serial.available()) {
      if (millis() - t0 > timeout) return false;
      vTaskDelay(1);
      esp_task_wdt_reset();
    }
    dst[i] = serial.read();
  }
  return true;
}
bool waitLidarHeader(HardwareSerial &serial) {
  uint8_t pos = 0;
  uint32_t t0 = millis();
  while (true) {
    if (serial.available()) {
      if (uint8_t(serial.read()) == HDR[pos]) {
        if (++pos == 4) return true;
      } else {
        pos = 0;
      }
    }
    if (millis() - t0 > 200) return false;
    esp_task_wdt_reset();
  }
}
inline uint16_t crc16(uint16_t crc, uint8_t v) {
  crc ^= v;
  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

/* ---------- Настройка WebSocket и HTTP ---------- */
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocketClient *wsClient = nullptr;
WiFiServer musicServer(music_port);
WiFiServer imageServer(image_port);
WiFiClient imageClient;
WiFiClient client;

// Обработчик событий WebSocket
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    // Новый клиент подключился
    if (wsClient && wsClient->status() == WS_CONNECTED) {
      // Закрываем предыдущего клиента, если был
      wsClient->close();
    }
    wsClient = client;
    //wsClient->printf("[WS] Connected (id=%u)\n", client->id());
    wsClient->client()->setNoDelay(true);  // отключаем алгоритм Нэгла для минимальной задержки
    Serial.printf("[WS] Client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    if (client == wsClient) {
      wsClient = nullptr;
      Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
  } else if (type == WS_EVT_DATA) {
    // NEW: входящее сообщение от клиента (ROS2), может содержать команду на движение
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->opcode == WS_BINARY && len == 4) {
      // Ожидаем 4-байтовое бинарное сообщение: [int16_t left_mm_s, int16_t right_mm_s]
      int16_t left = data[0] | (data[1] << 8);
      int16_t right = data[2] | (data[3] << 8);
      // Устанавливаем новые целевые скорости колес
      tgtL = (float)left;
      tgtR = (float)right;
      lastCmdMs = millis();  // ← обновили «пинг»
      // Включаем режим выравнивания, если |vL|≈|vR| и не ноль (робот едет прямо или крутится на месте)
      if (fabs(fabs(tgtL) - fabs(tgtR)) < 1.0f && fabs(tgtL) > 1.0f) {
        alignMode = true;
        alignSign = (tgtL * tgtR >= 0) ? 1.0f : -1.0f;
        alignRefL = encTotL;
        alignRefR = encTotR;
      } else {
        alignMode = false;
      }
      // Можно отправить подтверждение или лог (не обязательно)
      Serial.printf("[WS] Cmd: left=%d, right=%d\n", left, right);
    }
    // Если нужно обработать текстовые сообщения или другие бинарные команды, добавить тут
  }
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
    voltage += raw * 0.00242f;  // калибруйте под свою схему!
    delay(1);
  }
  return voltage / i;
}


void setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
  digitalWrite(TFT_DC, LOW);
  digitalWrite(TFT_CS, LOW);
  SPI.transfer(0x2A);  // CASET
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer(0x00);
  SPI.transfer(x0);
  SPI.transfer(0x00);
  SPI.transfer(x1);

  digitalWrite(TFT_DC, LOW);
  SPI.transfer(0x2B);  // RASET
  digitalWrite(TFT_DC, HIGH);
  SPI.transfer(0x00);
  SPI.transfer(y0);
  SPI.transfer(0x00);
  SPI.transfer(y1);

  digitalWrite(TFT_DC, LOW);
  SPI.transfer(0x2C);  // RAMWR (запись блока)
  digitalWrite(TFT_DC, HIGH);
  // CS держать LOW до конца передачи блока!
}


void onImageReceived() {
  digitalWrite(TFT_CS, LOW);  // Активируем дисплей

  setAddrWindow(0, 0, IMG_WIDTH - 1, IMG_HEIGHT - 1);  // Адресное окно
  digitalWrite(TFT_DC, HIGH);                          // DC=1 — данные

  for (int i = 0; i < IMG_WIDTH * IMG_HEIGHT; i++) {
    uint16_t color = buf[i];
    SPI.transfer(color >> 8);  // старший байт

    SPI.transfer(color & 0xFF);  // младший байт
  }

  digitalWrite(TFT_CS, HIGH);  // Деактивируем дисплей
  // digitalWrite(TFT_DC, LOW); // Обычно не требуется, если только сразу не будете слать команду
}

// Настройка HTTP-роутов
void setupRoutes() {
  server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Формируем JSON с текущим состоянием робота
    char json[512];
    snprintf(json, sizeof(json),
             "{\"duty\":{\"L_A\":%u,\"L_B\":%u,\"R_A\":%u,\"R_B\":%u},"
             "\"enc\":{\"left\":%ld,\"right\":%ld},"
             "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
             "\"target\":{\"left\":%.1f,\"right\":%.1f},"
             "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"th\":%.3f}}",
             dutyLA, dutyLB, dutyRA, dutyRB,
             encTotL, encTotR,
             speedL, speedR, tgtL, tgtR,
             odomX, odomY, odomTh);
    request->send(200, "application/json", json);
  });
  server.on("/setSpeed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("l")) tgtL = request->getParam("l")->value().toFloat();
    if (request->hasParam("r")) tgtR = request->getParam("r")->value().toFloat();
    lastCmdMs = millis();  // ← обновили «пинг»
    // Управление alignMode аналогично, как выше
    if (fabs(fabs(tgtL) - fabs(tgtR)) < 1.0f && fabs(tgtL) > 1.0f) {
      alignMode = true;
      alignSign = (tgtL * tgtR >= 0) ? 1.0f : -1.0f;
      alignRefL = encTotL;
      alignRefR = encTotR;
    } else {
      alignMode = false;
    }
    request->send(200, "text/plain", "ok");
  });
  server.on("/setCoeff", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("kp")) kp = request->getParam("kp")->value().toFloat();
    if (request->hasParam("ki")) ki = request->getParam("ki")->value().toFloat();
    if (request->hasParam("kd")) kd = request->getParam("kd")->value().toFloat();
    if (request->hasParam("kff")) kff = request->getParam("kff")->value().toFloat();
    request->send(200, "text/plain", "ok");
  });
  server.on("/resetEnc", HTTP_GET, [](AsyncWebServerRequest *request) {
    encTotL = encTotR = 0;
    prevEncL = prevEncR = 0;
    speedL = speedR = 0;
    alignMode = false;
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    request->send(200, "text/plain", "enc reset");
  });
  server.on("/resetOdom", HTTP_GET, [](AsyncWebServerRequest *request) {
    odomX = odomY = odomTh = 0;
    request->send(200, "text/plain", "odom reset");
  });
  // Тестовая главная страница (необязательно)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain",
                  "Endpoints: /state /setSpeed /setCoeff /resetEnc /resetOdom");
  });
}

/* ---------- Управление моторами (PWM) ---------- */
inline void setPWM(uint8_t pin, uint8_t value) {
  analogWrite(pin, value);
  // Обновляем глобальные duty-переменные для мониторинга
  switch (pin) {
    case L_A: dutyLA = value; break;
    case L_B: dutyLB = value; break;
    case R_A: dutyRA = value; break;
    case R_B: dutyRB = value; break;
  }
}
inline void stopMotors() {
  setPWM(L_A, 0);
  setPWM(L_B, 0);
  setPWM(R_A, 0);
  setPWM(R_B, 0);
}

/* ---------- Инициализация счетчиков PCNT ---------- */
void pcntInit(pcnt_unit_t unit, gpio_num_t pulse_pin, gpio_num_t ctrl_pin) {
  pcnt_config_t cfg = {};
  cfg.unit = unit;
  cfg.channel = PCNT_CHANNEL_0;
  cfg.pulse_gpio_num = pulse_pin;
  cfg.ctrl_gpio_num = ctrl_pin;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DEC;
  cfg.lctrl_mode = PCNT_MODE_REVERSE;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = -32768;
  pcnt_unit_config(&cfg);
  pcnt_set_filter_value(unit, 100);
  pcnt_filter_enable(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}
inline int16_t readEncoder(pcnt_unit_t unit) {
  int16_t count = 0;
  pcnt_get_counter_value(unit, &count);
  pcnt_counter_clear(unit);
  return count;
}

/* ---------- PID-регулятор скорости ---------- */
void updatePID() {
  static uint32_t prevMillis = millis();
  static float iTermL = 0, iTermR = 0;
  static float prevErrorL = 0, prevErrorR = 0;
  static float lastTgtL = 0, lastTgtR = 0;

  uint32_t now = millis();
  float dt = (now - prevMillis) * 0.001f;
  if (dt < 0.001f) dt = 0.001f;
  prevMillis = now;

  // Сброс интегральной части при изменении целевой скорости или переходе через 0
  if (tgtL != lastTgtL || fabs(tgtL) < 1.0f) {
    iTermL = 0;
    prevErrorL = 0;
  }
  if (tgtR != lastTgtR || fabs(tgtR) < 1.0f) {
    iTermR = 0;
    prevErrorR = 0;
  }
  lastTgtL = tgtL;
  lastTgtR = tgtR;

  // Выравнивание (P-регулятор) – корректирует цель при езде строго прямо или повороте на месте
  float corr = 0.0f;
  if (alignMode) {
    int32_t dL = encTotL - alignRefL;
    int32_t dR = encTotR - alignRefR;
    float diff_mm = (float)(dL - alignSign * dR) * MM_PER_TICK;  // разница пройденного пути (мм)
    corr = kAlign * diff_mm;                                     // мм/с коррекция
  }
  float tgtCorrL = tgtL - corr;
  float tgtCorrR = tgtR + alignSign * corr;

  // PID для левого и правого колеса
  float errorL = tgtCorrL - speedL;
  float errorR = tgtCorrR - speedR;
  iTermL += errorL * dt;
  iTermR += errorR * dt;
  // ограничиваем накопление интегральной ошибки
  const float I_LIMIT = 300.0f;
  if (iTermL > I_LIMIT) iTermL = I_LIMIT;
  if (iTermL < -I_LIMIT) iTermL = -I_LIMIT;
  if (iTermR > I_LIMIT) iTermR = I_LIMIT;
  if (iTermR < -I_LIMIT) iTermR = -I_LIMIT;
  float dTermL = (errorL - prevErrorL) / dt;
  float dTermR = (errorR - prevErrorR) / dt;
  prevErrorL = errorL;
  prevErrorR = errorR;
  // Управляющее воздействие
  float outputL = kp * errorL + ki * iTermL + kd * dTermL + kff * tgtCorrL;
  float outputR = kp * errorR + ki * iTermR + kd * dTermR + kff * tgtCorrR;
  // Ограничиваем PWM
  if (outputL > 255) outputL = 255;
  if (outputL < -255) outputL = -255;
  if (outputR > 255) outputR = 255;
  if (outputR < -255) outputR = -255;
  // Задаём PWM на моторах (A-вперёд, B-назад)
  uint8_t pwmLA, pwmLB, pwmRA, pwmRB;
  if (outputL >= 0) {
    pwmLA = (uint8_t)outputL;
    pwmLB = 0;
  } else {
    pwmLA = 0;
    pwmLB = (uint8_t)(-outputL);
  }
  if (outputR >= 0) {
    pwmRA = (uint8_t)outputR;
    pwmRB = 0;
  } else {
    pwmRA = 0;
    pwmRB = (uint8_t)(-outputR);
  }
  setPWM(L_A, pwmLA);
  setPWM(L_B, pwmLB);
  setPWM(R_A, pwmRA);
  setPWM(R_B, pwmRB);
}

/* ---------- Задача чтения Лидара (поток на Core 1) ---------- */
void lidarTask(void *param) {
  esp_task_wdt_add(NULL);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  uint8_t body[BODY_LEN];
  while (true) {

    esp_task_wdt_reset();
    vTaskDelay(1);
    if (!waitLidarHeader(Serial2)) {
      continue;
    }
    if (!readBytes(Serial2, body, BODY_LEN)) {
      continue;
    }


    handleClient();

    // Распарсить порцию точек лидара
    float startDeg = decodeAngle(body[2] | (body[3] << 8));
    uint8_t offset = 4;
    uint16_t dist[8];
    uint8_t quality[8];
    for (int i = 0; i < 8; ++i) {
      dist[i] = body[offset] | (body[offset + 1] << 8);
      quality[i] = body[offset + 2];
      offset += 3;
    }
    float endDeg = decodeAngle(body[offset] | (body[offset + 1] << 8));
    if (endDeg < startDeg) endDeg += 360.0f;
    if (endDeg - startDeg > MAX_SPREAD_DEG) {
      // Пропускаем пакет, если слишком большой разрыв (ошибка)
      continue;
    }
    // Упаковываем 20 байт в общий буфер скана
    uint16_t s = (uint16_t)(startDeg * 100 + 0.5f);
    uint16_t e = (uint16_t)(endDeg * 100 + 0.5f);
    *wr++ = s & 0xFF;
    *wr++ = s >> 8;
    *wr++ = e & 0xFF;
    *wr++ = e >> 8;
    for (int i = 0; i < 8; ++i) {
      uint16_t d = (quality[i] >= INTENSITY_MIN) ? dist[i] : 0;
      *wr++ = d & 0xFF;
      *wr++ = d >> 8;
    }
    frameCount++;
    stat_rx++;
    if (frameCount >= MAX_FRAMES) {
      frameCount = 0;
      wr = scanBuf;
      prevStartAngle = -1;  // сброс для корректного «перескока» угла
      continue;             // переходим к следующему пакету
    }
    // Проверяем перескок через 0° (начало нового круга)
    if (prevStartAngle >= 0.0f && startDeg < prevStartAngle && frameCount >= 30) {
      size_t scanSize = frameCount * FRAME_LEN;
      // Вычисляем CRC для всего скана
      uint16_t crc = 0xFFFF;
      for (size_t i = 0; i < scanSize; ++i) {
        crc = crc16(crc, scanBuf[i]);
      }
      scanBuf[scanSize] = crc & 0xFF;
      scanBuf[scanSize + 1] = crc >> 8;
      // Отправляем по WebSocket, если клиент подключен
      if (wsClient && wsClient->canSend()) {
        wsClient->binary(scanBuf, scanSize + 2);
        stat_tx++;
      }
      // Сбрасываем буфер для следующего оборота
      wr = scanBuf;
      frameCount = 0;
    }
    prevStartAngle = startDeg;
  }
}

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 Lidar+Motor Bridge ===");
  setup_i2s_streaming();  // Для музыки
  setup_adc_bat();        // Для чтения BAT

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 5);
  tft.print("CPU LOAD");



  tft2.initR(INITR_BLACKTAB);
  tft2.setRotation(3);
  tft2.fillScreen(ST77XX_BLACK);
  // Wi-Fi подключение
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print('.');
  }
  Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  // Настройка GPIO
  pinMode(L_A, OUTPUT);
  pinMode(L_B, OUTPUT);
  pinMode(R_A, OUTPUT);
  pinMode(R_B, OUTPUT);
  stopMotors();
  // PCNT для энкодеров
  pcntInit(PCNT_UNIT_0, (gpio_num_t)ENC_R_A, (gpio_num_t)ENC_R_B);
  pcntInit(PCNT_UNIT_1, (gpio_num_t)ENC_L_A, (gpio_num_t)ENC_L_B);
  // Запуск веб-сервера и WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  setupRoutes();
  server.begin();
  musicServer.begin();
  imageServer.begin();
  // Запуск задачи лидара на ядре 0
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 4096, nullptr, 2, nullptr, 0);


  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);  // по умолчанию не выбран
  SPI.begin();
}

void handleClient() {
  if (!client) {  // Проверяем нового клиента
    client = musicServer.available();
    if (client) {
      client.setTimeout(1);  // Неблокирующий режим (1 мс)
      Serial.println("TCP client connected.");
    }
  }

  if (client && client.connected()) {
    int avail = client.available();
    if (avail > 0) {
      uint8_t buffer[1024];
      // Определяем размер для чтения (решаем проблему типов)
      size_t toRead = (avail < static_cast<int>(sizeof(buffer)))
                        ? avail
                        : sizeof(buffer);

      int n = client.readBytes(buffer, toRead);

      if (n > 0) {
        Serial.print("Received TCP audio bytes: ");
        Serial.println(n);

        // Отладочный вывод (первые 8 байт)
        for (int i = 0; i < n && i < 8; ++i) {
          Serial.print(buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        // Передача в I2S
        size_t bytes_written;
        i2s_write(I2S_NUM_0, buffer, n, &bytes_written, portMAX_DELAY);
      }
    }
  } else if (client) {
    // Клиент отключился
    client.stop();
    Serial.println("TCP client disconnected.");
    client = WiFiClient();  // Сбрасываем объект клиента
  }
}










/* ---------- LOOP ---------- */
void loop() {
  static uint32_t t10 = 0, t20 = 0, t2000 = 0;
  uint32_t now = millis();
  /* --- safety timeout: если 3000 ms нет команд, стоп --- */
  /*
  if (now - lastCmdMs > 3000) {
    if (tgtL != 0 || tgtR != 0) {  // меняем только один раз
      tgtL = tgtR = 0;
      alignMode = false;
      // мгновенно убираем PWM – чтобы робот тут же замер
      stopMotors();
      Serial.println("[SAFE] cmd timeout → STOP");
    }
  }*/
  // Каждые 10 мс: считываем энкодеры, обновляем одометрию
  if (now - t10 >= 10) {
    t10 = now;
    int16_t dR = readEncoder(PCNT_UNIT_0);
    int16_t dL = readEncoder(PCNT_UNIT_1);
    encTotR += dR;
    encTotL += dL;
    // Расстояние, пройденное каждым колесом за 10 мс (в метрах)
    float sR = dR * MM_PER_TICK / 1000.0f;
    float sL = dL * MM_PER_TICK / 1000.0f;
    // Обновляем одометрические координаты (в глобальной системе odom)
    float ds = 0.5f * (sR + sL);
    float dth = (sR - sL) / BASE_L;
    float midTh = odomTh + 0.5f * dth;
    odomX += ds * cosf(midTh);
    odomY += ds * sinf(midTh);
    odomTh += dth;
    // Нормализуем угол odomTh в [-pi, pi]
    if (odomTh > M_PI) odomTh -= 2 * M_PI;
    if (odomTh < -M_PI) odomTh += 2 * M_PI;
  }
  // Каждые 20 мс: вычисляем текущие скорости колес (мм/с)
  if (now - t20 >= 20) {
    float dt = (now - t20) * 0.001f;
    t20 = now;
    speedL = (encTotL - prevEncL) * MM_PER_TICK / dt;
    speedR = (encTotR - prevEncR) * MM_PER_TICK / dt;
    prevEncL = encTotL;
    prevEncR = encTotR;
  }
  // Каждые 20 мс: обновляем PID-регулятор и ШИМ моторов
  static uint32_t tPID = 0;
  if (now - tPID >= 20) {
    tPID = now;
    updatePID();
  }
  // Обслуживание клиентов WebSocket (освобождение памяти для отключившихся)
  ws.cleanupClients();
  // Каждые 2 с: выводим IP (для мониторинга, не обязательно)
  if (now - t2000 >= 2000) {
    t2000 = now;
    Serial.print(WiFi.localIP());
    Serial.print("    ");
    Serial.print("bat voltage");
    Serial.print(check_bat());
    Serial.println();
  }



  imageClient = imageServer.available();
  if (imageClient) {
    Serial.println("✅ Client connected");
    int total = 0;
    uint8_t *byte_buf = (uint8_t *)buf;
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
  /*
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();
    uint8_t load = getCPULoad();
    memmove(cpuLoadHistory, cpuLoadHistory + 1, GRAPH_WIDTH - 1);
    cpuLoadHistory[GRAPH_WIDTH - 1] = map(load, 0, 100, 0, GRAPH_HEIGHT);

    drawGraph();

    tft.fillRect(0, GRAPH_HEIGHT + 5, 160, 20, ST77XX_BLACK);
    tft.setCursor(10, GRAPH_HEIGHT + 10);
    tft.printf("CPU1 Load: %d%%", load);
  }*/















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