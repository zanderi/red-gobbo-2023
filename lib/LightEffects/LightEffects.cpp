#include "LightEffects.h"
#include <math.h>

// Constants (internal to the library)
static const unsigned long UNUSED_TS = 0xFFFFFFFFUL;
static const unsigned long CHRISTMAS_PWM_US = 2000UL;
static const int CHRISTMAS_PWM_RES = 64;
static const unsigned long CHRISTMAS_FADE_MS = 300;
static const unsigned long CHRISTMAS_HOLD_MS = 300;
static const unsigned long CHRISTMAS_OVERLAP_MS = 100;
static const unsigned long CHRISTMAS_TOTAL_MS = (CHRISTMAS_FADE_MS + CHRISTMAS_HOLD_MS + CHRISTMAS_FADE_MS);
static const unsigned long CHRISTMAS_STEP_MS = (CHRISTMAS_TOTAL_MS - CHRISTMAS_OVERLAP_MS);
static const unsigned long CHRISTMAS_COOLDOWN_MS = 80;

// group fade defaults
static unsigned long GROUP_FADE_MS = 300;
static unsigned long GROUP_HOLD_MS = 300;
static unsigned long GROUP_GAP_MS  = 400;

// sequence defaults
static unsigned long MODE_DURATION_MS = 60000UL; // can be overridden with setModeDuration()
static const unsigned long TRANSITION_FADE_MS = 800UL;
static const uint8_t TRANSITION_BASELINE_BRI = 6;
static const unsigned long TRANSITION_HOLD_MS = 200UL;

// helper
static inline unsigned long rndRange(unsigned long a, unsigned long b){
  return a + (unsigned long)random(0, (int)(b - a + 1));
}

LightEffects* LightEffects::instance = nullptr;

LightEffects::LightEffects(const int pins[6]) {
  memcpy(_pins, pins, sizeof(_pins));
  christmasPwmTimer = nullptr;
  christmasPwmPhase = 0;
  christmasNextIndex = 0;
  christmasNextStartAt = 0;
  randomLastIndex = 0;
  fadeBaseStart = 0;
  currentMode = MODE_CHAIN;
  modeStartMs = 0;
  modeInTransition = false;
  modeTransitionStart = 0;
  modeDurationMs = MODE_DURATION_MS;

  // init arrays
  for (int i = 0; i < 6; ++i) {
    christmasDuty[i] = 0;
    christmasStartTimes[i] = UNUSED_TS;
    christmasCooldown[i] = 0;
    twinkle_active[i] = false;
    twinkle_startTs[i] = 0;
    twinkle_durMs[i] = 0;
    twinkle_peakBrightness[i] = 255;
    twinkle_cooldown[i] = 0;
  }
}

void LightEffects::setModeDuration(unsigned long ms) {
  modeDurationMs = ms;
}

inline uint8_t LightEffects::scaleToIsr(uint8_t bri) {
  return (uint8_t)((((unsigned int)bri) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
}

void LightEffects::begin() {
  // configure pins as outputs and ensure off
  for (int i = 0; i < 6; ++i) {
    pinMode(_pins[i], OUTPUT);
    gpio_set_level((gpio_num_t)_pins[i], LED_OFF);
    christmasDuty[i] = 0;
  }

  // schedule initial chain
  unsigned long now = millis();
  christmasStartTimes[0] = now;
  christmasNextIndex = 1 % 6;
  christmasNextStartAt = now + CHRISTMAS_STEP_MS;

  // init timer for software PWM
  unsigned long tick_us = CHRISTMAS_PWM_US / (unsigned long)CHRISTMAS_PWM_RES;
  if (tick_us < 1) tick_us = 1;
  christmasPwmTimer = timerBegin(0, 80, true);
  instance = this;
  timerAttachInterrupt(christmasPwmTimer, &LightEffects::onChristmasPwmTimerStatic, true);
  timerAlarmWrite(christmasPwmTimer, tick_us, true);
  timerAlarmEnable(christmasPwmTimer);
}

void LightEffects::tick() {
  runSequenceManager();
}

void IRAM_ATTR LightEffects::onChristmasPwmTimerStatic() {
  if (instance) instance->onChristmasPwmTimer();
}

void IRAM_ATTR LightEffects::onChristmasPwmTimer() {
  christmasPwmPhase++;
  if (christmasPwmPhase >= CHRISTMAS_PWM_RES) christmasPwmPhase = 0;
  for (int i = 0; i < 6; ++i) {
    uint8_t duty = christmasDuty[i];
    bool on = (christmasPwmPhase < duty);
    gpio_set_level((gpio_num_t)_pins[i], on ? LED_ON : LED_OFF);
  }
}

static inline uint8_t christmasBrightnessForElapsed(unsigned long elapsed) {
  if (elapsed >= CHRISTMAS_TOTAL_MS) return 0;
  if (elapsed < CHRISTMAS_FADE_MS) {
    return (uint8_t)((elapsed * 255UL) / CHRISTMAS_FADE_MS);
  }
  unsigned long t = elapsed - CHRISTMAS_FADE_MS;
  if (t < CHRISTMAS_HOLD_MS) return 255;
  unsigned long tout = t - CHRISTMAS_HOLD_MS;
  return (uint8_t)(((CHRISTMAS_FADE_MS - tout) * 255UL) / CHRISTMAS_FADE_MS);
}

void LightEffects::chainChristmasLights() {
  unsigned long now = millis();
  uint8_t nextDuty[6];
  for (int i = 0; i < 6; ++i) nextDuty[i] = christmasDuty[i];

  for (int i = 0; i < 6; ++i) {
    unsigned long ts = christmasStartTimes[i];
    if (ts == UNUSED_TS) { nextDuty[i] = 0; continue; }
    unsigned long elapsed = now - ts;
    if (elapsed >= CHRISTMAS_TOTAL_MS) {
      christmasStartTimes[i] = UNUSED_TS;
      christmasCooldown[i] = now + CHRISTMAS_COOLDOWN_MS;
      nextDuty[i] = 0;
      continue;
    }
    uint8_t bri = christmasBrightnessForElapsed(elapsed);
    if (bri < 6) bri = 0;
    uint8_t scaled = scaleToIsr(bri);
    nextDuty[i] = scaled;
  }

  if (now >= christmasNextStartAt) {
    int attempts = 0;
    int idx = christmasNextIndex;
    while (attempts < 6) {
      if (christmasStartTimes[idx] == UNUSED_TS && now >= christmasCooldown[idx]) {
        christmasStartTimes[idx] = now;
        christmasNextIndex = (idx + 1) % 6;
        christmasNextStartAt = now + CHRISTMAS_STEP_MS;
        nextDuty[idx] = 1;
        break;
      }
      idx = (idx + 1) % 6;
      attempts++;
    }
    if (attempts >= 6) christmasNextStartAt = now + 10;
  }

  noInterrupts();
  for (int i = 0; i < 6; ++i) christmasDuty[i] = nextDuty[i];
  interrupts();
}

void LightEffects::randomLights() {
  unsigned long now = millis();
  uint8_t nextDuty[6];
  for (int i = 0; i < 6; ++i) nextDuty[i] = christmasDuty[i];

  // Update currently active lights
  for (int i = 0; i < 6; ++i) {
    unsigned long ts = christmasStartTimes[i];
    if (ts == UNUSED_TS) { nextDuty[i] = 0; continue; }
    unsigned long elapsed = now - ts;
    if (elapsed >= CHRISTMAS_TOTAL_MS) {
      christmasStartTimes[i] = UNUSED_TS;
      christmasCooldown[i] = now + CHRISTMAS_COOLDOWN_MS;
      nextDuty[i] = 0;
      continue;
    }
    uint8_t bri = christmasBrightnessForElapsed(elapsed);
    if (bri < 6) bri = 0;
    uint8_t scaled = scaleToIsr(bri);
    nextDuty[i] = scaled;
  }

  // Start next random light (excluding sequential next)
  if (now >= christmasNextStartAt) {
    int attempts = 0;
    int sequentialNext = (randomLastIndex + 1) % 6;
    
    while (attempts < 12) { // Try up to 12 times to find a valid light
      int idx = random(0, 6);
      
      // Skip if it's the sequential next one
      if (idx == sequentialNext) {
        attempts++;
        continue;
      }
      
      // Check if this light is available
      if (christmasStartTimes[idx] == UNUSED_TS && now >= christmasCooldown[idx]) {
        christmasStartTimes[idx] = now;
        randomLastIndex = idx;
        christmasNextStartAt = now + CHRISTMAS_STEP_MS;
        nextDuty[idx] = 1;
        break;
      }
      attempts++;
    }
    if (attempts >= 12) christmasNextStartAt = now + 10;
  }

  noInterrupts();
  for (int i = 0; i < 6; ++i) christmasDuty[i] = nextDuty[i];
  interrupts();
}

void LightEffects::resetRandom() {
  unsigned long now = millis();
  for (int i = 0; i < 6; ++i) {
    christmasStartTimes[i] = UNUSED_TS;
    christmasCooldown[i] = 0;
  }
  randomLastIndex = random(0, 6);
  christmasStartTimes[randomLastIndex] = now;
  christmasNextStartAt = now + CHRISTMAS_STEP_MS;
}

void LightEffects::resetTwinkle() {
  for (int i = 0; i < 6; ++i) {
    twinkle_active[i] = false;
    twinkle_startTs[i] = 0;
    twinkle_durMs[i] = 0;
    twinkle_peakBrightness[i] = 255;
    twinkle_cooldown[i] = 0;
  }
  twinkle_nextEvalAt = 0;
}

void LightEffects::twinkleLights() {
  // Match chain effect timing: 300ms fade in + 300ms hold + 300ms fade out = 900ms total
  const unsigned long TWINKLE_DUR_MS = CHRISTMAS_TOTAL_MS; // 900ms to match chain effect
  const unsigned long FADE_IN_MS = CHRISTMAS_FADE_MS;      // 300ms
  const unsigned long HOLD_MS = CHRISTMAS_HOLD_MS;         // 300ms
  const unsigned long FADE_OUT_MS = CHRISTMAS_FADE_MS;     // 300ms
  const uint8_t START_CHANCE = 14;

  unsigned long now = millis();
  
  // Check if it's time to potentially start new lights
  bool canStartNew = false;
  if (twinkle_nextEvalAt == 0) { 
    twinkle_nextEvalAt = now + rndRange(250,400); 
  }
  if (now >= twinkle_nextEvalAt) {
    canStartNew = true;
    twinkle_nextEvalAt = now + rndRange(250,400);
  }

  uint8_t nextDuty[6];
  for (int i = 0; i < 6; ++i) nextDuty[i] = 0;

  // Count active lights and set cooldown for finished ones
  int activeCount = 0;
  for (int i = 0; i < 6; ++i) {
    if (twinkle_active[i]) {
      unsigned long elapsed = now - twinkle_startTs[i];
      if (elapsed >= TWINKLE_DUR_MS) {
        twinkle_active[i] = false;
        twinkle_cooldown[i] = now + rndRange(500, 1500); // 0.5-1.5 second cooldown
      } else {
        activeCount++;
      }
    }
  }

  // Ensure at least one light is always active (ignore cooldown for this)
  if (activeCount == 0) {
    int idx = random(0, 6);
    twinkle_active[idx] = true;
    twinkle_startTs[idx] = now;
    twinkle_durMs[idx] = TWINKLE_DUR_MS;
    twinkle_peakBrightness[idx] = random(128, 256); // Random brightness 50%-100%
    twinkle_cooldown[idx] = 0; // Clear cooldown since we need this light on
    activeCount = 1;
  }

  // Start new lights if it's time
  if (canStartNew) {
    for (int i = 0; i < 6; ++i) {
      // Only start lights that are not currently active and past their cooldown
      if (!twinkle_active[i] && now >= twinkle_cooldown[i]) {
        if (activeCount < 3 && (random(0,100) < START_CHANCE)) {
          twinkle_active[i] = true;
          twinkle_startTs[i] = now;
          twinkle_durMs[i] = TWINKLE_DUR_MS;
          twinkle_peakBrightness[i] = random(128, 256); // Random brightness 50%-100%
          activeCount++;
        }
      }
    }
  }

  // Update all lights every frame for smooth fading
  for (int i = 0; i < 6; ++i) {
    if (twinkle_active[i]) {
      unsigned long elapsed = now - twinkle_startTs[i];
      if (elapsed >= TWINKLE_DUR_MS) { 
        twinkle_active[i] = false; 
        nextDuty[i] = 0; 
        continue; 
      }
      
      // Calculate fade with random peak brightness
      uint8_t peakBri = twinkle_peakBrightness[i];
      uint8_t bri = 0;
      
      if (elapsed < FADE_IN_MS) {
        // Fade in phase
        bri = (uint8_t)((elapsed * (unsigned long)peakBri) / FADE_IN_MS);
      } else if (elapsed < (FADE_IN_MS + HOLD_MS)) {
        // Hold phase at peak brightness
        bri = peakBri;
      } else {
        // Fade out phase
        unsigned long fadeOutElapsed = elapsed - (FADE_IN_MS + HOLD_MS);
        bri = (uint8_t)((unsigned long)peakBri - ((fadeOutElapsed * (unsigned long)peakBri) / FADE_OUT_MS));
      }
      
      uint8_t scaled = scaleToIsr((uint8_t)constrain(bri, 0, 255));
      nextDuty[i] = scaled;
    } else {
      nextDuty[i] = 0;
    }
  }

  noInterrupts();
  for (int i = 0; i < 6; ++i) christmasDuty[i] = nextDuty[i];
  interrupts();
}

void LightEffects::resetChain() {
  unsigned long now = millis();
  for (int i = 0; i < 6; ++i) {
    christmasStartTimes[i] = UNUSED_TS;
    christmasCooldown[i] = 0;
  }
  christmasStartTimes[0] = now;
  christmasNextIndex = 1 % 6;
  christmasNextStartAt = now + CHRISTMAS_STEP_MS;
}

void LightEffects::resetFade() {
  fadeBaseStart = 0;
}

void LightEffects::groupFade(uint8_t nextDuty[6], const int groupIdx[3], unsigned long baseStart, unsigned long phaseOffsetMs) {
  const unsigned long FADE_MS = GROUP_FADE_MS;
  const unsigned long HOLD_MS = GROUP_HOLD_MS;
  const unsigned long TOTAL_MS = FADE_MS + HOLD_MS + FADE_MS;
  unsigned long now = millis();
  unsigned long elapsed = (now >= baseStart) ? (now - baseStart) : 0;
  unsigned long period = (TOTAL_MS + GROUP_GAP_MS) * 2;
  elapsed = (elapsed + phaseOffsetMs) % period;
  if (elapsed >= TOTAL_MS) return;
  uint8_t bri;
  if (elapsed < FADE_MS) bri = (uint8_t)((elapsed * 255UL) / FADE_MS);
  else if (elapsed < (FADE_MS + HOLD_MS)) bri = 255;
  else { unsigned long tout = elapsed - (FADE_MS + HOLD_MS); bri = (uint8_t)(((FADE_MS - tout) * 255UL) / FADE_MS); }
  if (bri < 6) bri = 0;
  uint8_t scaled = scaleToIsr(bri);
  for (int i = 0; i < 3; ++i) nextDuty[groupIdx[i]] = scaled;
}

void LightEffects::fadeLights() {
  const int A_idx[3] = {0,2,4};
  const int B_idx[3] = {1,3,5};
  unsigned long now = millis();
  if (fadeBaseStart == 0) fadeBaseStart = now;
  uint8_t nextDuty[6]; for (int i=0;i<6;++i) nextDuty[i]=0;
  const unsigned long FADE_MS = GROUP_FADE_MS;
  const unsigned long HOLD_MS = GROUP_HOLD_MS;
  const unsigned long TOTAL_MS = FADE_MS + HOLD_MS + FADE_MS;
  const unsigned long GAP_MS = GROUP_GAP_MS;
  const unsigned long CYCLE = (TOTAL_MS + GAP_MS) * 2;
  unsigned long t = (now - fadeBaseStart) % CYCLE;

  if (t < TOTAL_MS) {
    unsigned long e = t;
    uint8_t bri;
    if (e < FADE_MS) bri = (uint8_t)((e * 255UL) / FADE_MS);
    else if (e < FADE_MS + HOLD_MS) bri = 255;
    else bri = (uint8_t)(((FADE_MS - (e - (FADE_MS + HOLD_MS))) * 255UL) / FADE_MS);
    if (bri < 6) bri = 0;
    uint8_t scaled = scaleToIsr(bri);
    for (int i=0;i<3;++i) nextDuty[A_idx[i]] = scaled;
    for (int i=0;i<3;++i) nextDuty[B_idx[i]] = 0;
  } else if (t < TOTAL_MS + GAP_MS) {
    // off
  } else if (t < TOTAL_MS + GAP_MS + TOTAL_MS) {
    unsigned long e = t - (TOTAL_MS + GAP_MS);
    uint8_t bri;
    if (e < FADE_MS) bri = (uint8_t)((e * 255UL) / FADE_MS);
    else if (e < FADE_MS + HOLD_MS) bri = 255;
    else bri = (uint8_t)(((FADE_MS - (e - (FADE_MS + HOLD_MS))) * 255UL) / FADE_MS);
    if (bri < 6) bri = 0;
    uint8_t scaled = scaleToIsr(bri);
    for (int i=0;i<3;++i) nextDuty[B_idx[i]] = scaled;
    for (int i=0;i<3;++i) nextDuty[A_idx[i]] = 0;
  }

  noInterrupts();
  for (int i=0;i<6;++i) christmasDuty[i]=nextDuty[i];
  interrupts();
}

void LightEffects::runSequenceManager() {
  unsigned long now = millis();
  if (!modeInTransition) {
    if (modeStartMs == 0) {
      modeStartMs = now;
      if (currentMode == MODE_TWINKLE) resetTwinkle();
      else if (currentMode == MODE_CHAIN) resetChain();
      else if (currentMode == MODE_RANDOM) resetRandom();
    }
    if (currentMode == MODE_CHAIN) chainChristmasLights();
    else if (currentMode == MODE_TWINKLE) twinkleLights();
    else if (currentMode == MODE_RANDOM) randomLights();
  if ((now - modeStartMs) >= modeDurationMs) { modeInTransition = true; modeTransitionStart = now; }
  } else {
    unsigned long elapsed = now - modeTransitionStart;
    float frac = (elapsed >= TRANSITION_FADE_MS) ? 0.0f : (1.0f - (float)elapsed / (float)TRANSITION_FADE_MS);
    if (frac < 0.0f) frac = 0.0f;
    noInterrupts();
    for (int i = 0; i < 6; ++i) {
      uint8_t duty = christmasDuty[i];
      uint8_t bri255 = (uint8_t)(((unsigned int)duty * 255U) / (unsigned int)CHRISTMAS_PWM_RES);
      uint8_t out = (uint8_t)((float)bri255 * frac + 0.5f);
      christmasDuty[i] = (uint8_t)((((unsigned int)out) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
    }
    interrupts();

    if (elapsed >= (TRANSITION_FADE_MS + TRANSITION_HOLD_MS)) {
      noInterrupts(); for (int i=0;i<6;++i) christmasDuty[i]=0; interrupts();
      modeInTransition = false; modeStartMs = 0;
      // Cycle between chain, random, and twinkle effects
      if (currentMode == MODE_CHAIN) currentMode = MODE_RANDOM;
      else if (currentMode == MODE_RANDOM) currentMode = MODE_TWINKLE;
      else currentMode = MODE_CHAIN;
    } else if (elapsed >= TRANSITION_FADE_MS) {
      noInterrupts();
      for (int i=0;i<6;++i) {
        uint8_t out = TRANSITION_BASELINE_BRI;
        christmasDuty[i] = (uint8_t)((((unsigned int)out) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
      }
      interrupts();
    }
  }
}
