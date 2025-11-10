#pragma once
#include <Arduino.h>
#include "driver/gpio.h"

// Darlington transistor array inverts the signal
// If USE_DARLINGTON is defined, we invert: HIGH->LOW, LOW->HIGH
#ifdef USE_DARLINGTON
  #define LED_ON  0
  #define LED_OFF 1
#else
  #define LED_ON  1
  #define LED_OFF 0
#endif

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

  // random light state (reuses christmasStartTimes/christmasCooldown arrays)
  int randomLastIndex;
  
  // sequence manager
  enum SequenceMode { MODE_CHAIN = 0, MODE_TWINKLE = 1, MODE_RANDOM = 2 };
  SequenceMode currentMode;
  unsigned long modeStartMs;
  unsigned long modeTransitionStart;
  bool modeInTransition;
  unsigned long modeDurationMs;

  // internal helpers
  void chainChristmasLights();
  void twinkleLights();
  void randomLights();
  void fadeLights();
  void groupFade(uint8_t nextDuty[6], const int groupIdx[3], unsigned long baseStart, unsigned long phaseOffsetMs);
  void runSequenceManager();
  void resetTwinkle();
  void resetChain();
  void resetRandom();
  void resetFade();

  static LightEffects* instance; // for ISR callback
  static void IRAM_ATTR onChristmasPwmTimerStatic();
  void IRAM_ATTR onChristmasPwmTimer();

  // low-level helper
  inline uint8_t scaleToIsr(uint8_t bri);
};
