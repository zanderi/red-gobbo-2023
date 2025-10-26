#include <Arduino.h>

const int CHRISTMAS_PINS[6] = {5, 6, 7, 9, 4, 3};
#ifndef LED_PIN
  #ifdef LED_BUILTIN
    #define LED_PIN LED_BUILTIN
  #else
    #define LED_PIN 8
  #endif
#endif

const int CHANNEL_CHRISTMAS_BASE = 2;
const int PWM_FREQ = 5000;
const int PWM_RES = 8;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Christmas pin test starting...");
  Serial.print("LED_PIN = "); Serial.println(LED_PIN);
  for (int i = 0; i < 6; ++i) {
    Serial.print("CHRISTMAS_PINS["); Serial.print(i); Serial.print("] = ");
    Serial.println(CHRISTMAS_PINS[i]);
  }

  // Check conflict
  for (int i = 0; i < 6; ++i) {
    if (CHRISTMAS_PINS[i] == LED_PIN) {
      Serial.println("WARNING: LED_PIN conflicts with Christmas pin -> change LED_PIN or CHRISTMAS_PINS");
    }
  }

  // Simple digital on/off test (drives ULN input HIGH to sink LEDs)
  Serial.println("Digital test: each pin HIGH for 600ms");
  for (int i = 0; i < 6; ++i) {
    pinMode(CHRISTMAS_PINS[i], OUTPUT);
    digitalWrite(CHRISTMAS_PINS[i], LOW); // ensure off
  }
  delay(200);
  for (int i = 0; i < 6; ++i) {
    Serial.print("Testing pin "); Serial.println(CHRISTMAS_PINS[i]);
    digitalWrite(CHRISTMAS_PINS[i], HIGH);
    delay(600);
    digitalWrite(CHRISTMAS_PINS[i], LOW);
    delay(200);
  }

  // PWM test via ledc
  Serial.println("PWM test: full ON (255) for 600ms via LEDC");
  for (int i = 0; i < 6; ++i) {
    int ch = CHANNEL_CHRISTMAS_BASE + i;
    ledcSetup(ch, PWM_FREQ, PWM_RES);
    ledcAttachPin(CHRISTMAS_PINS[i], ch);
    ledcWrite(ch, 0);
  }
  delay(200);
  for (int i = 0; i < 6; ++i) {
    int ch = CHANNEL_CHRISTMAS_BASE + i;
    Serial.print("PWM ON pin "); Serial.println(CHRISTMAS_PINS[i]);
    ledcWrite(ch, 255);
    delay(600);
    ledcWrite(ch, 0);
    delay(200);
  }

  Serial.println("Test done. Observe which LEDs lit and note any that behave oddly.");
}

void loop(){ /* nothing */ }