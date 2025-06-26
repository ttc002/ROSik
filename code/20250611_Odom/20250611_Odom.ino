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
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "driver/pcnt.h"
#include <math.h>

/* ────── GPIO ────── */
#define L_A 33
#define L_B 32
#define R_A 26
#define R_B 25
#define ENC_R_A 18
#define ENC_R_B 19
#define ENC_L_A 34
#define ENC_L_B 35

/* ────── Wi-Fi ───── */
constexpr char SSID[] = "robotx";
constexpr char PASS[] = "78914040";

/* ────── Механика ─── */
constexpr float WHEEL_D = 0.044f;              // m
constexpr float BASE_L  = 0.096f;              // m
constexpr int   TICKS_REV = 2930;
constexpr float MM_PER_TICK = WHEEL_D * M_PI * 1000.0f / TICKS_REV;

/* ────── Глобальные ─ */
volatile uint8_t dutyLA,dutyLB,dutyRA,dutyRB;
volatile int32_t encTotL=0, encTotR=0, prevEncL=0, prevEncR=0;
volatile float   speedL=0, speedR=0, tgtL=0, tgtR=0;
volatile float   odomX=0, odomY=0, odomTh=0;
volatile float   kp=1.0f, ki=0.8f, kd=0.02f, kff=0.25f;

/* --- Alignment P-контур --- */
bool     alignMode = false;        // включён ли «режим прямо»
float    alignSign = 1;            //  +1 прямой ход, −1 разворот
int32_t  alignRefL = 0, alignRefR = 0;
constexpr float kAlign = 1;   // (мм ⇒ мм/с)

/* ────── LEDC ───── */
inline void pwm(uint8_t pin,uint8_t d){ analogWrite(pin,d);
  switch(pin){case L_A:dutyLA=d;break;case L_B:dutyLB=d;break;
              case R_A:dutyRA=d;break;case R_B:dutyRB=d;break;} }
inline void stopMotors(){ pwm(L_A,0); pwm(L_B,0); pwm(R_A,0); pwm(R_B,0);}

/* ────── PCNT ───── */
inline void pcntInit(pcnt_unit_t u,gpio_num_t a,gpio_num_t b){
  pcnt_config_t c{}; c.unit=u;c.channel=PCNT_CHANNEL_0;
  c.pulse_gpio_num=a; c.ctrl_gpio_num=b;
  c.pos_mode=PCNT_COUNT_INC; c.neg_mode=PCNT_COUNT_DEC;
  c.lctrl_mode=PCNT_MODE_REVERSE; c.hctrl_mode=PCNT_MODE_KEEP;
  c.counter_h_lim=32767; c.counter_l_lim=-32768;
  pcnt_unit_config(&c); pcnt_set_filter_value(u,100); pcnt_filter_enable(u);
  pcnt_counter_clear(u); pcnt_counter_resume(u);}
inline int16_t snap(pcnt_unit_t u){int16_t v; pcnt_get_counter_value(u,&v); pcnt_counter_clear(u);return v;}

/* ────── API ───── */
AsyncWebServer server(80);

void routes(){
  server.on("/state",HTTP_GET,[](auto*r){
      char js[512];
      snprintf(js,sizeof(js),
        "{\"duty\":{\"L_A\":%u,\"L_B\":%u,\"R_A\":%u,\"R_B\":%u},"
        "\"enc\":{\"left\":%ld,\"right\":%ld},"
        "\"speed\":{\"left\":%.1f,\"right\":%.1f},"
        "\"target\":{\"left\":%.1f,\"right\":%.1f},"
        "\"odom\":{\"x\":%.3f,\"y\":%.3f,\"th\":%.3f}}",
        dutyLA,dutyLB,dutyRA,dutyRB,
        encTotL,encTotR, speedL,speedR, tgtL,tgtR,
        odomX,odomY,odomTh);
      r->send(200,"application/json",js);});

  /* ---------- setSpeed ---------- */
  server.on("/setSpeed",HTTP_GET,[](auto*r){
      float newL=tgtL, newR=tgtR;
      if(r->hasParam("l")) newL=r->getParam("l")->value().toFloat();
      if(r->hasParam("r")) newR=r->getParam("r")->value().toFloat();
      tgtL=newL; tgtR=newR;

      /* — включаем alignment, если |vL| ≈ |vR| и не ноль — */
      if(fabs(fabs(newL)-fabs(newR))<1 && fabs(newL)>1){
        alignMode=true;
        alignSign = (newL*newR>=0) ? 1.0f : -1.0f;
        alignRefL = encTotL; alignRefR = encTotR;
      }else alignMode=false;

      r->send(200,"text/plain","ok");
  });

  server.on("/setCoeff",HTTP_GET,[](auto*r){
      if(r->hasParam("kp"))  kp  = r->getParam("kp")->value().toFloat();
      if(r->hasParam("ki"))  ki  = r->getParam("ki")->value().toFloat();
      if(r->hasParam("kd"))  kd  = r->getParam("kd")->value().toFloat();
      if(r->hasParam("kff")) kff = r->getParam("kff")->value().toFloat();
      r->send(200,"text/plain","ok");});

  server.on("/resetEnc",HTTP_GET,[](auto*r){
      encTotL=encTotR=prevEncL=prevEncR=0;
      speedL=speedR=0; alignMode=false;
      pcnt_counter_clear(PCNT_UNIT_0); pcnt_counter_clear(PCNT_UNIT_1);
      r->send(200,"text/plain","enc reset");});

  server.on("/resetOdom",HTTP_GET,[](auto*r){
      odomX=odomY=odomTh=0; r->send(200,"text/plain","odom reset");});

  server.on("/",HTTP_GET,[](auto*r){
      r->send(200,"text/plain",
      "/state /setSpeed /setCoeff /resetEnc /resetOdom");});
}

/* ───── PWM map (A-вперёд) ───── */
inline void mapPWM(float val, uint8_t& a, uint8_t& b){
  if(val>=0){ a=uint8_t(constrain(val,0,255)); b=0; }
  else       { a=0; b=uint8_t(constrain(-val,0,255)); }
}

/* ───── PID ───── */
void pidTask(){
  static uint32_t tPrev=millis();
  static float iL=0,iR=0,ePrevL=0,ePrevR=0;
  static float lastTgtL=0,lastTgtR=0;

  float dt=(millis()-tPrev)*0.001f; if(dt<0.001f) dt=0.001f; tPrev=millis();
  if(tgtL!=lastTgtL||fabs(tgtL)<1){iL=0;ePrevL=0;}
  if(tgtR!=lastTgtR||fabs(tgtR)<1){iR=0;ePrevR=0;}
  lastTgtL=tgtL; lastTgtR=tgtR;

  /* ----- alignment correction (P-only) ----- */
  float corr=0;
  if(alignMode){
    int32_t dL=encTotL-alignRefL, dR=encTotR-alignRefR;
    float diff = (dL - alignSign*dR) * MM_PER_TICK;   // мм
    corr = kAlign * diff;                             // мм/с
  }
  float tgtCorrL = tgtL - corr;
  float tgtCorrR = tgtR + alignSign * corr;

  /* ----- обычный PID ----- */
  float eL=tgtCorrL-speedL, eR=tgtCorrR-speedR;
  iL+=eL*dt; iR+=eR*dt; const float ILIM=300;
  iL=constrain(iL,-ILIM,ILIM); iR=constrain(iR,-ILIM,ILIM);
  float dL=(eL-ePrevL)/dt, dR=(eR-ePrevR)/dt;
  ePrevL=eL; ePrevR=eR;
  float outL=kp*eL+ki*iL+kd*dL+kff*tgtCorrL;
  float outR=kp*eR+ki*iR+kd*dR+kff*tgtCorrR;
  outL=constrain(outL,-255,255); outR=constrain(outR,-255,255);

  uint8_t aL,bL,aR,bR; mapPWM(outL,aL,bL); mapPWM(outR,aR,bR);
  pwm(L_A,aL); pwm(L_B,bL); pwm(R_A,aR); pwm(R_B,bR);
}

/* ───── SETUP / LOOP ───── */
void setup(){
  Serial.begin(115200);
  pinMode(L_A,OUTPUT); pinMode(L_B,OUTPUT); pinMode(R_A,OUTPUT); pinMode(R_B,OUTPUT); stopMotors();
  pcntInit(PCNT_UNIT_0,(gpio_num_t)ENC_R_A,(gpio_num_t)ENC_R_B);
  pcntInit(PCNT_UNIT_1,(gpio_num_t)ENC_L_A,(gpio_num_t)ENC_L_B);
  WiFi.mode(WIFI_STA); WiFi.begin(SSID,PASS);
  while(WiFi.status()!=WL_CONNECTED){delay(300);Serial.print('.');}
  Serial.println("\nIP "+WiFi.localIP().toString()); routes(); server.begin();
}

void loop(){
  static uint32_t t10=0,t20=0,tPID=0; uint32_t ms=millis();

  if(ms-t10>=10){ t10=ms;
    int16_t dR=snap(PCNT_UNIT_0), dL=snap(PCNT_UNIT_1);
    encTotR+=dR; encTotL+=dL;
    float sR=dR*MM_PER_TICK/1000, sL=dL*MM_PER_TICK/1000;
    float ds=0.5f*(sR+sL), dth=(sR-sL)/BASE_L, mid=odomTh+0.5f*dth;
    odomX+=ds*cosf(mid); odomY+=ds*sinf(mid); odomTh+=dth;
    if(odomTh>M_PI)odomTh-=2*M_PI; if(odomTh<-M_PI)odomTh+=2*M_PI; }

  if(ms-t20>=20){ float dt=(ms-t20)*0.001f; t20=ms;
    speedL=(encTotL-prevEncL)*MM_PER_TICK/dt; speedR=(encTotR-prevEncR)*MM_PER_TICK/dt;
    prevEncL=encTotL; prevEncR=encTotR; }

  if(ms-tPID>=20){ tPID=ms; pidTask(); }
}
