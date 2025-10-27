#include <Arduino.h>
#include "driver/gpio.h"   // for gpio_set_level

// Christmas LEDs: each LED cathode -> its own ULN2803 OUTx, ULN INs driven by these GPIOs
const int CHRISTMAS_PINS[6] = {5, 6, 7, 8, 9, 10}; // <-- updated per your wiring
#define FIRE_PIN 1           // fireplace LED (PWM)
#define FUSE_PIN 2           // fuse LED (PWM)

// ledc channel assignments:
// 0 = fire, 1 = fuse, 2..7 = christmas LEDs (one per LED)
const int CHANNEL_FIRE = 0;
const int CHANNEL_FUSE = 1;
const int CHANNEL_CHRISTMAS_BASE = 2; // channels 2..7
const int PWM_FREQ = 5000;
const int PWM_RES = 8; // 0..255

// --- software PWM parameters for the Christmas strand (ISR-driven) ---
// total PWM period in microseconds and resolution (number of steps)
const unsigned long CHRISTMAS_PWM_US = 2000UL; // 2ms period (~500Hz)
const int CHRISTMAS_PWM_RES = 64;             // 64-step resolution

// ISR-driven duty table (0..CHRISTMAS_PWM_RES-1)
volatile uint8_t christmasDuty[6] = {0,0,0,0,0,0};
volatile int christmasPwmPhase = 0;
hw_timer_t* christmasPwmTimer = NULL;

// --- runtime state ---
unsigned long nowMs = 0;

// status LED
unsigned long statusToggleNext = 0;
const unsigned long STATUS_INTERVAL = 500;
bool statusOn = false;

// Christmas chase state (every-other back-and-forth)
unsigned long christmasNextChange = 0;
const unsigned long CHRISTMAS_INTERVAL = 180; // ms between steps
const unsigned long CHRISTMAS_FADE = 300;     // fade duration in ms
int christmasPos = 0;      // 0..2 (positions within even/odd groups)
int christmasDir = 1;      // 1 forward, -1 backward
bool christmasPhaseEven = true; // true => even indices (0,2,4), false => odd (1,3,5)

// fading state
bool christmasFading = false;
unsigned long christmasFadeStart = 0;
uint8_t prevBrightness[6];
uint8_t targetBrightness[6];
// temp next pos/phase to commit after fade completes
int christmasNextPos = 0;
bool christmasNextPhaseEven = true;

// Fireplace flicker (non-blocking, restored behavior)
unsigned long fireNextUpdate = 0;
const unsigned long FIRE_UPDATE_MIN = 20;
const unsigned long FIRE_UPDATE_MAX = 60;
float firePhase = 0.0f;

// Fuse flicker (non-blocking, restored behavior)
unsigned long fuseNextUpdate = 0;
const unsigned long FUSE_UPDATE_MIN = 10;
const unsigned long FUSE_UPDATE_MAX = 40;
bool fuseSparkActive = false;
unsigned long fuseSparkEnd = 0;

static inline unsigned long rndRange(unsigned long a, unsigned long b){
  return a + (unsigned long)random(0, (int)(b - a + 1));
}

// compute mask for given pos/phase
static inline uint8_t christmasMaskFor(int pos, bool phaseEven) {
  if (pos < 0) pos = 0; if (pos > 2) pos = 2;
  if (phaseEven) {
    return (1u << (pos * 2)); // 0,2,4
  } else {
    return (1u << (1 + pos * 2)); // 1,3,5
  }
}

// Christmas timing/state helpers (needed by the brightness scheduler)
const unsigned long CHRISTMAS_FADE_MS    = 300;
const unsigned long CHRISTMAS_HOLD_MS    = 300;
const unsigned long CHRISTMAS_OVERLAP_MS = 100;
const unsigned long CHRISTMAS_TOTAL_MS   = (CHRISTMAS_FADE_MS + CHRISTMAS_HOLD_MS + CHRISTMAS_FADE_MS);
const unsigned long CHRISTMAS_STEP_MS    = (CHRISTMAS_TOTAL_MS - CHRISTMAS_OVERLAP_MS);

const unsigned long CHRISTMAS_COOLDOWN_MS = 80; // ms cooldown to avoid immediate reuse
static unsigned long christmasStartTimes[6];
static unsigned long christmasCooldown[6];
static const unsigned long UNUSED_TS = 0xFFFFFFFFUL;
// next-start scheduling helpers
static unsigned long christmasNextStartAt = 0;
static int christmasNextIndex = 0;

// Sequence manager state: run chain, twinkle, fade in sequence
enum SequenceMode { MODE_CHAIN = 0, MODE_TWINKLE = 1, MODE_FADE = 2 };
SequenceMode currentMode = MODE_CHAIN;
unsigned long modeStartMs = 0;
const unsigned long MODE_DURATION_MS = 15000UL; // 15 seconds per mode
bool modeInTransition = false;
unsigned long modeTransitionStart = 0;
const unsigned long TRANSITION_FADE_MS = 800UL; // fade-to-off duration between modes
// Instead of forcing absolute zero (which can show as brief ON due to PWM/resolution),
// fade to a small baseline brightness and hold, then clear to zero. This reduces
// visible blips on some hardware/ULN2803 combinations.
const uint8_t TRANSITION_BASELINE_BRI = 6; // 0..255 baseline while settling (approx 2%)
const unsigned long TRANSITION_HOLD_MS = 200UL; // hold baseline before final zero

// Twinkle state moved to file-scope so we can reset when switching modes
static bool twinkle_active[6] = {false,false,false,false,false,false};
static unsigned long twinkle_startTs[6] = {0,0,0,0,0,0};
static unsigned long twinkle_durMs[6] = {0,0,0,0,0,0};
static unsigned long twinkle_nextEvalAt = 0;

// Fade helper baseStart moved to file-scope so we can reset it when entering fade mode
static unsigned long fadeBaseStart = 0;

// Group fade parameters (used by fadeLights / groupFade)
// per user request, slow fade timing to ~300ms
const unsigned long GROUP_FADE_MS = 250;
const unsigned long GROUP_HOLD_MS = 250;
const unsigned long GROUP_GAP_MS  = 400;

// brightness helper used by chainChristmasLights()
static inline uint8_t christmasBrightnessForElapsed(unsigned long elapsed) {
  if (elapsed >= CHRISTMAS_TOTAL_MS) return 0;
  if (elapsed < CHRISTMAS_FADE_MS) {
    return (uint8_t)((elapsed * 255UL) / CHRISTMAS_FADE_MS);         // fade in
  }
  unsigned long t = elapsed - CHRISTMAS_FADE_MS;
  if (t < CHRISTMAS_HOLD_MS) return 255;                            // hold
  unsigned long tout = t - CHRISTMAS_HOLD_MS;                       // fade out
  return (uint8_t)(((CHRISTMAS_FADE_MS - tout) * 255UL) / CHRISTMAS_FADE_MS);
}

// wrapper to match calls in code (writeChristmasBrightness used throughout)
// forward-declare low-level writer so wrapper can call it before its definition
static inline void writeChristmasBrightnessRaw(uint8_t idx, uint8_t val);
static inline void writeChristmasBrightness(uint8_t idx, uint8_t val) {
  writeChristmasBrightnessRaw(idx, val);
}

// low-level writer: scale 0..255 into 0..(CHRISTMAS_PWM_RES-1) and update atomically
static inline void writeChristmasBrightnessRaw(uint8_t idx, uint8_t val) {
  if (idx >= 6) return;
  uint8_t scaled = (uint8_t)((((unsigned int)val) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
  noInterrupts();
  christmasDuty[idx] = scaled;
  interrupts();
}

// prototype for the new strands functions
void fadeLights();
void groupFade(uint8_t nextDuty[6], const int groupIdx[3], unsigned long baseStart, unsigned long phaseOffsetMs);
void twinkleLights();
void runSequenceManager();
void resetTwinkle();
void resetChain();
void resetFade();
// forward declarations for ISR and low-level writer used before their definitions
void IRAM_ATTR onChristmasPwmTimer();
static inline void writeChristmasBrightnessRaw(uint8_t idx, uint8_t val);

// Non-blocking update: handle active fades and schedule next LED starts
void chainChristmasLights() {
  unsigned long now = millis();

  // prepare shadow duties to compute all updates without partially updating the active table
  uint8_t nextDuty[6];
  for (int i = 0; i < 6; ++i) nextDuty[i] = christmasDuty[i]; // start from current

  // 1) Update all six outputs and free finished slots first (use same 'now')
  for (int i = 0; i < 6; ++i) {
    unsigned long ts = christmasStartTimes[i];
    if (ts == UNUSED_TS) {
      nextDuty[i] = 0;
      continue;
    }
    unsigned long elapsed = now - ts;
    if (elapsed >= CHRISTMAS_TOTAL_MS) {
      // finished, free slot, ensure off and set short cooldown to avoid immediate reuse
      christmasStartTimes[i] = UNUSED_TS;
      christmasCooldown[i] = now + CHRISTMAS_COOLDOWN_MS;
      nextDuty[i] = 0;
      continue;
    }
  uint8_t bri = christmasBrightnessForElapsed(elapsed);
  // avoid tiny non-zero values that create perceptible blips
  if (bri < 6) bri = 0;
  // scale 0..255 -> 0..(CHRISTMAS_PWM_RES-1)
  uint8_t scaled = (uint8_t)((((unsigned int)bri) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
  nextDuty[i] = scaled;
  }

  // 2) Now schedule next LED if scheduled time reached (will not pick cooled slots)
  if (now >= christmasNextStartAt) {
    // try to find a free slot (starting from christmasNextIndex)
    int attempts = 0;
    int idx = christmasNextIndex;
    while (attempts < 6) {
      if (christmasStartTimes[idx] == UNUSED_TS && now >= christmasCooldown[idx]) {
        // found usable free slot
        christmasStartTimes[idx] = now;
        christmasNextIndex = (idx + 1) % 6;
        christmasNextStartAt = now + CHRISTMAS_STEP_MS;
        // ensure newly started LED has at least a minimal duty for the first cycle
        nextDuty[idx] = 1;
        break;
      }
      idx = (idx + 1) % 6;
      attempts++;
    }
    if (attempts >= 6) {
      // all slots busy or in cooldown; postpone slightly
      christmasNextStartAt = now + 10;
    }
  }

  // Atomically copy nextDuty into christmasDuty to avoid mid-frame inconsistencies
  noInterrupts();
  for (int i = 0; i < 6; ++i) christmasDuty[i] = nextDuty[i];
  interrupts();
}

// Restored fireplace flicker dynamics (non-blocking)
void updateFire() {
  if (nowMs < fireNextUpdate) return;

  int base = 120;
  int variance = random(-60, 80);
  int sinv = (int)(20.0f * sin(firePhase));
  int val = base + variance + sinv;
  val = constrain(val, 0, 255);
  ledcWrite(CHANNEL_FIRE, val);

  // schedule next small update
  firePhase += 0.12f + (random(0, 10) / 100.0f);
  if (firePhase > 10000.0f) firePhase = fmod(firePhase, 1000.0f);
  fireNextUpdate = nowMs + rndRange(FIRE_UPDATE_MIN, FIRE_UPDATE_MAX);
}

// Restored fuse flicker dynamics (non-blocking)
void updateFuse() {
  if (nowMs < fuseNextUpdate) return;

  int val;
  // start a spark occasionally if none active
  if (!fuseSparkActive && (random(0, 100) < 12)) {
    fuseSparkActive = true;
    fuseSparkEnd = nowMs + rndRange(80, 320); // spark lasts a short while
  }

  if (fuseSparkActive) {
    // produce quick variable sparks while active
    val = random(160, 255);
    // end spark when time elapsed
    if (nowMs >= fuseSparkEnd) {
      fuseSparkActive = false;
    }
  } else {
    // idle tiny flicker/glow
    val = random(8, 40);
  }

  val = constrain(val, 0, 255);
  ledcWrite(CHANNEL_FUSE, val);

  fuseNextUpdate = nowMs + rndRange(FUSE_UPDATE_MIN, FUSE_UPDATE_MAX);
}

void led_test() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Christmas pin test starting...");
  Serial.print("LED_PIN = "); Serial.println(LED_BUILTIN);
  for (int i = 0; i < 6; ++i) {
    Serial.print("CHRISTMAS_PINS["); Serial.print(i); Serial.print("] = ");
    Serial.println(CHRISTMAS_PINS[i]);
  }

  // Check conflict
  for (int i = 0; i < 6; ++i) {
    if (CHRISTMAS_PINS[i] == LED_BUILTIN) {
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

  // remove LEDC PWM test for christmas pins to avoid LEDC vs software PWM conflict
  Serial.println("Skipping LEDC PWM test for Christmas pins (using software PWM)");
  Serial.println("Test done. Observe which LEDs lit and note any that behave oddly.");
}

void setup() {
  delay(1500);                 // allow USB to enumerate
  Serial.begin(115200);
  // led_test();
  
  Serial.println("LED effects starting...");

  // --- DO NOT allocate LEDC channels for christmas pins ---
  // setup only software PWM: configure christmas pins as digital outputs
  for (int i = 0; i < 6; ++i) {
    pinMode(CHRISTMAS_PINS[i], OUTPUT);
    gpio_set_level((gpio_num_t)CHRISTMAS_PINS[i], 0);
    prevBrightness[i] = 0;
    targetBrightness[i] = 0;
    christmasDuty[i] = 0;
  }

  christmasPos = 0;
  christmasDir = 1;
  christmasPhaseEven = true;
  christmasNextChange = millis() + CHRISTMAS_INTERVAL;
  christmasFading = false;

  // setup PWM channels for FIRE and FUSE only (LED C channels 0 and 1)
  ledcSetup(CHANNEL_FIRE, PWM_FREQ, PWM_RES);
  ledcAttachPin(FIRE_PIN, CHANNEL_FIRE);
  ledcWrite(CHANNEL_FIRE, 0);
  fireNextUpdate = millis() + rndRange(FIRE_UPDATE_MIN, FIRE_UPDATE_MAX);

  ledcSetup(CHANNEL_FUSE, PWM_FREQ, PWM_RES);
  ledcAttachPin(FUSE_PIN, CHANNEL_FUSE);
  ledcWrite(CHANNEL_FUSE, 0);
  fuseNextUpdate = millis() + rndRange(FUSE_UPDATE_MIN, FIRE_UPDATE_MAX);

  statusToggleNext = millis() + STATUS_INTERVAL;
  randomSeed(analogRead(0));

  // initialize schedule
  for (int i = 0; i < 6; ++i) {
    christmasStartTimes[i] = UNUSED_TS;
    christmasCooldown[i] = 0;
  }
  unsigned long now = millis();
  christmasStartTimes[0] = now;
  christmasNextIndex = 1 % 6;
  christmasNextStartAt = now + CHRISTMAS_STEP_MS;
  // initialize ISR-driven PWM timer for Christmas outputs
  {
    // tick in microseconds per PWM step
    unsigned long tick_us = CHRISTMAS_PWM_US / (unsigned long)CHRISTMAS_PWM_RES;
    if (tick_us < 1) tick_us = 1;
    // timerBegin(timerNumber, prescaler, countUp) - prescaler 80 -> 1 MHz tick (1us)
    christmasPwmTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(christmasPwmTimer, &onChristmasPwmTimer, true);
    timerAlarmWrite(christmasPwmTimer, tick_us, true);
    timerAlarmEnable(christmasPwmTimer);
  }
}

unsigned long lastStatusLog = 0;
const unsigned long STATUS_LOG_INTERVAL = 2500;


// Software PWM applier for Christmas outputs (call each loop)
// ISR-driven PWM: keep ISR small and fast
void IRAM_ATTR onChristmasPwmTimer() {
  // advance phase
  christmasPwmPhase++;
  if (christmasPwmPhase >= CHRISTMAS_PWM_RES) christmasPwmPhase = 0;
  // update outputs
  for (int i = 0; i < 6; ++i) {
    uint8_t duty = christmasDuty[i];
    // since duty range is 0..(RES-1), compare directly
    bool on = (christmasPwmPhase < duty);
    gpio_set_level((gpio_num_t)CHRISTMAS_PINS[i], on ? 1 : 0);
  }
}

void loop() {
  nowMs = millis();

  // status LED toggle (onboard)
  /* if (nowMs >= statusToggleNext) {
    statusToggleNext += STATUS_INTERVAL;
    statusOn = !statusOn;
    digitalWrite(LED_BUILTIN, statusOn ? HIGH : LOW);
  } */

  // run sequence manager which runs chain / twinkle / fade in order
  runSequenceManager();
  updateFire();
  updateFuse();
  // PWM now driven in ISR; nothing to call here

  // occasional status print
  if (nowMs - lastStatusLog >= STATUS_LOG_INTERVAL) {
    Serial.println("Running effects: christmas (fade chase) / fire / fuse");
    lastStatusLog = nowMs;
  }

  // yield instead of delay to keep PWM updates responsive
  yield();
}

// New helper: groupFade
// Used by fadeLights(): fades a group of three indices together; 'invert' offsets phase by half cycle
void groupFade(uint8_t nextDuty[6], const int groupIdx[3], unsigned long baseStart, unsigned long phaseOffsetMs) {
  // use global GROUP_FADE_MS / GROUP_HOLD_MS and GROUP_GAP_MS
  const unsigned long FADE_MS = GROUP_FADE_MS;
  const unsigned long HOLD_MS = GROUP_HOLD_MS;
  const unsigned long TOTAL_MS = FADE_MS + HOLD_MS + FADE_MS;

  unsigned long now = millis();
  unsigned long elapsed = (now >= baseStart) ? (now - baseStart) : 0;
  unsigned long period = (TOTAL_MS + GROUP_GAP_MS) * 2; // full A+gap+B+gap sequence
  // apply phase offset and wrap into period
  elapsed = (elapsed + phaseOffsetMs) % period;
  // if we're outside the active TOTAL_MS window, leave outputs off
  if (elapsed >= TOTAL_MS) return;

  uint8_t bri;
  if (elapsed < FADE_MS) {
    bri = (uint8_t)((elapsed * 255UL) / FADE_MS);
  } else if (elapsed < (FADE_MS + HOLD_MS)) {
    bri = 255;
  } else {
    unsigned long tout = elapsed - (FADE_MS + HOLD_MS);
    bri = (uint8_t)(((FADE_MS - tout) * 255UL) / FADE_MS);
  }
  if (bri < 6) bri = 0;
  uint8_t scaled = (uint8_t)((((unsigned int)bri) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
  for (int i = 0; i < 3; ++i) {
    nextDuty[groupIdx[i]] = scaled;
  }
}

void fadeLights() {
  // group indices
  const int A_idx[3] = {0,2,4}; // pins 5,7,4
  const int B_idx[3] = {1,3,5}; // pins 6,9,3
  unsigned long now = millis();
  if (fadeBaseStart == 0) fadeBaseStart = now;

  uint8_t nextDuty[6];
  for (int i = 0; i < 6; ++i) nextDuty[i] = 0;

  const unsigned long FADE_MS = GROUP_FADE_MS;
  const unsigned long HOLD_MS = GROUP_HOLD_MS;
  const unsigned long TOTAL_MS = FADE_MS + HOLD_MS + FADE_MS;
  const unsigned long GAP_MS = GROUP_GAP_MS;
  const unsigned long CYCLE = (TOTAL_MS + GAP_MS) * 2; // A + gap + B + gap

  unsigned long t = (now - fadeBaseStart) % CYCLE;

  if (t < TOTAL_MS) {
    // A active
    unsigned long e = t; // 0..TOTAL_MS
    uint8_t bri;
    if (e < FADE_MS) bri = (uint8_t)((e * 255UL) / FADE_MS);
    else if (e < FADE_MS + HOLD_MS) bri = 255;
    else bri = (uint8_t)(((FADE_MS - (e - (FADE_MS + HOLD_MS))) * 255UL) / FADE_MS);
    if (bri < 6) bri = 0;
    uint8_t scaled = (uint8_t)((((unsigned int)bri) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
    for (int i = 0; i < 3; ++i) nextDuty[A_idx[i]] = scaled;
    // ensure B is off
    for (int i = 0; i < 3; ++i) nextDuty[B_idx[i]] = 0;
  } else if (t < TOTAL_MS + GAP_MS) {
    // gap after A: all off
    // nextDuty already zeroed
  } else if (t < TOTAL_MS + GAP_MS + TOTAL_MS) {
    // B active
    unsigned long e = t - (TOTAL_MS + GAP_MS);
    uint8_t bri;
    if (e < FADE_MS) bri = (uint8_t)((e * 255UL) / FADE_MS);
    else if (e < FADE_MS + HOLD_MS) bri = 255;
    else bri = (uint8_t)(((FADE_MS - (e - (FADE_MS + HOLD_MS))) * 255UL) / FADE_MS);
    if (bri < 6) bri = 0;
    uint8_t scaled = (uint8_t)((((unsigned int)bri) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
    for (int i = 0; i < 3; ++i) nextDuty[B_idx[i]] = scaled;
    // ensure A is off
    for (int i = 0; i < 3; ++i) nextDuty[A_idx[i]] = 0;
  } else {
    // gap after B: all off
  }

  // atomically copy
  noInterrupts();
  for (int i = 0; i < 6; ++i) christmasDuty[i] = nextDuty[i];
  interrupts();
}

// New top-level twinkleLights: random short twinkles across all 6 LEDs.
// Non-blocking: call from loop() when you want the twinkle effect active.
// reset twinkle state (call when entering twinkle mode)
void resetTwinkle() {
  for (int i = 0; i < 6; ++i) {
    twinkle_active[i] = false;
    twinkle_startTs[i] = 0;
    twinkle_durMs[i] = 0;
  }
  twinkle_nextEvalAt = 0;
}

// Twinkle lights: uses file-scope twinkle_* state so it can be reset on mode changes
void twinkleLights() {
  // Tunable: faster, independent twinkles
  const unsigned long MIN_DUR_MS = 120;   // minimum twinkle length (ms)
  const unsigned long MAX_DUR_MS = 800;   // maximum twinkle length (ms)
  const uint8_t START_CHANCE = 14;        // percent chance to start per check per LED
  // We'll randomize the delay between start-evaluations to 250..400ms per user request

  unsigned long now = millis();
  // If this is the first call after a reset, schedule the first evaluation and return
  // so we don't immediately start a burst of twinkles. Keeps strand LOW initially.
  if (twinkle_nextEvalAt == 0) {
    twinkle_nextEvalAt = now + rndRange(250, 400);
    return;
  }
  if (now < twinkle_nextEvalAt) return; // wait until randomized next evaluation time
  // schedule next evaluation randomly between 250 and 400 ms
  twinkle_nextEvalAt = now + rndRange(250, 400);

  // compute next duty table in the resolution the ISR expects (0..CHRISTMAS_PWM_RES-1)
  uint8_t nextDuty[6];
  for (int i = 0; i < 6; ++i) nextDuty[i] = 0;

  // First, clear finished twinkles and count active ones
  int activeCount = 0;
  for (int i = 0; i < 6; ++i) {
    if (twinkle_active[i]) {
      unsigned long elapsed = now - twinkle_startTs[i];
      if (elapsed >= twinkle_durMs[i]) {
        twinkle_active[i] = false;
      } else {
        activeCount++;
      }
    }
  }

  // Then evaluate potential starts and compute envelopes
  for (int i = 0; i < 6; ++i) {
    // If LED is inactive, we may start it only if we haven't hit the cap
    if (!twinkle_active[i]) {
      if (activeCount < 3 && (random(0, 100) < START_CHANCE)) {
        twinkle_active[i] = true;
        twinkle_startTs[i] = now;
        twinkle_durMs[i] = rndRange(MIN_DUR_MS, MAX_DUR_MS);
        activeCount++;
      }
    } else {
      // active: allow random restart even when active (doesn't increase activeCount)
      if (random(0, 100) < START_CHANCE) {
        twinkle_startTs[i] = now;
        twinkle_durMs[i] = rndRange(MIN_DUR_MS, MAX_DUR_MS);
      }
    }

    if (twinkle_active[i]) {
      unsigned long elapsed = now - twinkle_startTs[i];
      if (elapsed >= twinkle_durMs[i]) {
        // finished twinkle
        twinkle_active[i] = false;
        nextDuty[i] = 0;
        continue;
      }
      // triangular envelope: up then down
  float progress = (float)elapsed / (float)twinkle_durMs[i]; // 0..1
      float tri;
      if (progress < 0.5f) tri = (progress / 0.5f);      // 0..1 up
      else tri = (1.0f - (progress - 0.5f) / 0.5f);      // 1..0 down
      int bri = (int)(tri * 255.0f + 0.5f);
      if (bri < 8) bri = 0; // cutoff to avoid micro-blips
      // scale to pwm resolution
      uint8_t scaled = (uint8_t)((((unsigned int)constrain(bri,0,255)) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
      nextDuty[i] = scaled;
    } else {
      nextDuty[i] = 0;
    }
  }

  // atomic update of the ISR duty table
  noInterrupts();
  for (int i = 0; i < 6; ++i) christmasDuty[i] = nextDuty[i];
  interrupts();
}

// Reset chain state (prepare for chain mode)
void resetChain() {
  unsigned long now = millis();
  for (int i = 0; i < 6; ++i) {
    christmasStartTimes[i] = UNUSED_TS;
    christmasCooldown[i] = 0;
  }
  // start the scheduler immediately with the first slot
  christmasStartTimes[0] = now;
  christmasNextIndex = 1 % 6;
  christmasNextStartAt = now + CHRISTMAS_STEP_MS;
}

// Reset fade mode state
void resetFade() {
  fadeBaseStart = 0; // will be set on first call to fadeLights()
}

// Sequence manager: call from loop() to run current mode and handle transitions
void runSequenceManager() {
  unsigned long now = millis();

  if (!modeInTransition) {
    // start the mode if not already started
    if (modeStartMs == 0) {
      modeStartMs = now;
      // reset mode-specific state
      if (currentMode == MODE_TWINKLE) resetTwinkle();
      else if (currentMode == MODE_CHAIN) resetChain();
      else if (currentMode == MODE_FADE) resetFade();
    }

    // run the active mode
    if (currentMode == MODE_CHAIN) {
      chainChristmasLights();
    } else if (currentMode == MODE_TWINKLE) {
      twinkleLights();
    } else if (currentMode == MODE_FADE) {
      fadeLights();
    }

    // check for mode timeout
    if ((now - modeStartMs) >= MODE_DURATION_MS) {
      modeInTransition = true;
      modeTransitionStart = now;
    }
  } else {
    // fade all LEDs to off over TRANSITION_FADE_MS
    unsigned long elapsed = now - modeTransitionStart;
    float frac = (elapsed >= TRANSITION_FADE_MS) ? 0.0f : (1.0f - (float)elapsed / (float)TRANSITION_FADE_MS);
    if (frac < 0.0f) frac = 0.0f;

    // apply fade to all christmas outputs (read current ISR duty and scale down)
    noInterrupts();
    for (int i = 0; i < 6; ++i) {
      uint8_t duty = christmasDuty[i];
      // convert to 0..255
      uint8_t bri255 = (uint8_t)(((unsigned int)duty * 255U) / (unsigned int)CHRISTMAS_PWM_RES);
      uint8_t out = (uint8_t)((float)bri255 * frac + 0.5f);
      // write scaled value (this will scale again inside writeChristmasBrightnessRaw)
      christmasDuty[i] = (uint8_t)((((unsigned int)out) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
    }
    interrupts();

    if (elapsed >= (TRANSITION_FADE_MS + TRANSITION_HOLD_MS)) {
      // final clear to zero after baseline hold
      noInterrupts();
      for (int i = 0; i < 6; ++i) christmasDuty[i] = 0;
      interrupts();

      // advance to next mode
      modeInTransition = false;
      modeStartMs = 0;
      // rotate mode
      if (currentMode == MODE_CHAIN) currentMode = MODE_TWINKLE;
      else if (currentMode == MODE_TWINKLE) currentMode = MODE_FADE;
      else currentMode = MODE_CHAIN;
    } else if (elapsed >= TRANSITION_FADE_MS) {
      // we're in baseline hold window: set a small non-zero baseline brightness
      float fracHold = 0.0f; // baseline fraction is fixed (we use TRANSITION_BASELINE_BRI)
      noInterrupts();
      for (int i = 0; i < 6; ++i) {
        // convert baseline 0..255 to ISR resolution
        uint8_t out = TRANSITION_BASELINE_BRI;
        christmasDuty[i] = (uint8_t)((((unsigned int)out) * (unsigned int)CHRISTMAS_PWM_RES) / 256U);
      }
      interrupts();
    }
  }
}