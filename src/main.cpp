// main.cpp now delegates lighting to the LightEffects library
#include <Arduino.h>
#include "LightEffects.h"

// FIRE/FUSE pins and LEDC configuration
#define FIRE_PIN 1
#define FUSE_PIN 2
const int CHANNEL_FIRE = 0;
const int CHANNEL_FUSE = 1;
const int PWM_FREQ = 5000;
const int PWM_RES = 8;

const int CHRISTMAS_PINS[6] = {5,6,7,8,9,10};
LightEffects lights(CHRISTMAS_PINS);

unsigned long nowMs = 0;

// FIRE/FUSE state
unsigned long fireNextUpdate = 0;
const unsigned long FIRE_UPDATE_MIN = 20;
const unsigned long FIRE_UPDATE_MAX = 60;
float firePhase = 0.0f;

unsigned long fuseNextUpdate = 0;
const unsigned long FUSE_UPDATE_MIN = 10;
const unsigned long FUSE_UPDATE_MAX = 40;
bool fuseSparkActive = false;
unsigned long fuseSparkEnd = 0;

static inline unsigned long rndRange(unsigned long a, unsigned long b){
  return a + (unsigned long)random(0, (int)(b - a + 1));
}

void updateFire() {
  if (nowMs < fireNextUpdate) return;
  int base = 120;
  int variance = random(-60, 80);
  int sinv = (int)(20.0f * sin(firePhase));
  int val = base + variance + sinv;
  val = constrain(val, 0, 255);
  ledcWrite(CHANNEL_FIRE, val);
  firePhase += 0.12f + (random(0, 10) / 100.0f);
  if (firePhase > 10000.0f) firePhase = fmod(firePhase, 1000.0f);
  fireNextUpdate = nowMs + rndRange(FIRE_UPDATE_MIN, FIRE_UPDATE_MAX);
}

void updateFuse() {
  if (nowMs < fuseNextUpdate) return;
  int val;
  if (!fuseSparkActive && (random(0, 100) < 12)) {
    fuseSparkActive = true;
    fuseSparkEnd = nowMs + rndRange(80, 320);
  }
  if (fuseSparkActive) { val = random(160,255); if (nowMs >= fuseSparkEnd) fuseSparkActive = false; }
  else { val = random(8,40); }
  val = constrain(val,0,255);
  ledcWrite(CHANNEL_FUSE, val);
  fuseNextUpdate = nowMs + rndRange(FUSE_UPDATE_MIN, FUSE_UPDATE_MAX);
}

void setup(){
  delay(1500);
  Serial.begin(115200);
  Serial.println("LED effects starting...");

  // setup PWM for fire and fuse
  ledcSetup(CHANNEL_FIRE, PWM_FREQ, PWM_RES);
  ledcAttachPin(FIRE_PIN, CHANNEL_FIRE);
  ledcWrite(CHANNEL_FIRE, 0);
  fireNextUpdate = millis() + rndRange(FIRE_UPDATE_MIN, FIRE_UPDATE_MAX);

  ledcSetup(CHANNEL_FUSE, PWM_FREQ, PWM_RES);
  ledcAttachPin(FUSE_PIN, CHANNEL_FUSE);
  ledcWrite(CHANNEL_FUSE, 0);
  fuseNextUpdate = millis() + rndRange(FUSE_UPDATE_MIN, FUSE_UPDATE_MAX);

  randomSeed(analogRead(0));
  lights.begin();
}

unsigned long lastStatusLog = 0;
const unsigned long STATUS_LOG_INTERVAL = 2500;

void loop(){
  nowMs = millis();
  lights.tick();
  updateFire();
  updateFuse();
  if (nowMs - lastStatusLog >= STATUS_LOG_INTERVAL){
    Serial.println("Running effects: sequence / fire / fuse");
    lastStatusLog = nowMs;
  }
  yield();
}