#pragma once
#include <Arduino.h>
#include "driver/gpio.h"

class LightEffects {
public:
  LightEffects(const int pins[6]);
  void begin();
  void tick();

  // Optional runtime controls
  void setModeDuration(unsigned long ms);

private:
  int _pins[6];

  // ISR-driven PWM state
  volatile uint8_t christmasDuty[6];
  volatile int christmasPwmPhase;
  hw_timer_t* christmasPwmTimer;

  // scheduling and state
  unsigned long christmasStartTimes[6];
  unsigned long christmasCooldown[6];
  int christmasNextIndex;
  unsigned long christmasNextStartAt;

  // twinkle state
  bool twinkle_active[6];
  unsigned long twinkle_startTs[6];
  unsigned long twinkle_durMs[6];
  unsigned long twinkle_nextEvalAt;

  // fade base
  unsigned long fadeBaseStart;

  // sequence manager
  enum SequenceMode { MODE_CHAIN = 0, MODE_TWINKLE = 1, MODE_FADE = 2 };
  SequenceMode currentMode;
  unsigned long modeStartMs;
  unsigned long modeTransitionStart;
  bool modeInTransition;
  unsigned long modeDurationMs;

  // internal helpers
  void chainChristmasLights();
  void twinkleLights();
  void fadeLights();
  void groupFade(uint8_t nextDuty[6], const int groupIdx[3], unsigned long baseStart, unsigned long phaseOffsetMs);
  void runSequenceManager();
  void resetTwinkle();
  void resetChain();
  void resetFade();

  static LightEffects* instance; // for ISR callback
  static void IRAM_ATTR onChristmasPwmTimerStatic();
  void IRAM_ATTR onChristmasPwmTimer();

  // low-level helper
  inline uint8_t scaleToIsr(uint8_t bri);
};
