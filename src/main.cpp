#include <Arduino.h>
#include "driver/gpio.h"   // for gpio_set_level

// Christmas LEDs: each LED cathode -> its own ULN2803 OUTx, ULN INs driven by these GPIOs
const int CHRISTMAS_PINS[6] = {5, 6, 7, 9, 4, 3}; // <-- updated per your wiring
#define FIRE_PIN 1           // fireplace LED (PWM)
#define FUSE_PIN 2           // fuse LED (PWM)

// ledc channel assignments:
// 0 = fire, 1 = fuse, 2..7 = christmas LEDs (one per LED)
const int CHANNEL_FIRE = 0;
const int CHANNEL_FUSE = 1;
const int CHANNEL_CHRISTMAS_BASE = 2; // channels 2..7
const int PWM_FREQ = 5000;
const int PWM_RES = 8; // 0..255

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

// Remove CHANNEL_CHRISTMAS_BASE usage (we'll use software PWM)
#undef CHANNEL_CHRISTMAS_BASE

// Software PWM settings for Christmas outputs (ISR-driven)
// Choose a PWM period and resolution such that ISR rate is reasonable.
// Period = CHRISTMAS_PWM_US microseconds, resolution = CHRISTMAS_PWM_RES steps.
const unsigned long CHRISTMAS_PWM_US = 2000; // 2 ms period (~500 Hz)
const uint8_t CHRISTMAS_PWM_RES = 64;       // 64-level resolution -> ISR ~ 32 kHz
volatile uint8_t christmasDuty[6]; // 0..(CHRISTMAS_PWM_RES-1) target duty for each Christmas LED
volatile uint8_t christmasPwmPhase = 0;
hw_timer_t * christmasPwmTimer = NULL;
// ISR prototype (defined later) - declare before use in setup
void IRAM_ATTR onChristmasPwmTimer();

// helper to write duty atomically (old writeChristmasBrightness replaced)
static inline void writeChristmasBrightnessRaw(uint8_t idx, uint8_t val) {
  christmasDuty[idx] = val;
}

// --- Add missing Christmas timing / state helpers (fixes undefined symbols) ---
const unsigned long CHRISTMAS_FADE_MS    = 300;
const unsigned long CHRISTMAS_HOLD_MS    = 300;
const unsigned long CHRISTMAS_OVERLAP_MS = 100;
const unsigned long CHRISTMAS_TOTAL_MS   = (CHRISTMAS_FADE_MS + CHRISTMAS_HOLD_MS + CHRISTMAS_FADE_MS); // 900
const unsigned long CHRISTMAS_STEP_MS    = (CHRISTMAS_TOTAL_MS - CHRISTMAS_OVERLAP_MS); // 800

const unsigned long CHRISTMAS_COOLDOWN_MS = 80; // used to avoid immediate reuse of a slot
static unsigned long christmasStartTimes[6];
static unsigned long christmasCooldown[6];
static const unsigned long UNUSED_TS = 0xFFFFFFFFUL;
// next-start scheduling helpers
static unsigned long christmasNextStartAt = 0;
static int christmasNextIndex = 0;

// brightness helper used by updateChristmas()
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
static inline void writeChristmasBrightness(uint8_t idx, uint8_t val) {
  writeChristmasBrightnessRaw(idx, val);
}

// prototype for the new strands functions
void fadeLights();
void twinkleLights(uint8_t nextDuty[6], const int groupIdx[3], unsigned long baseStart, bool invert);

// Non-blocking update: handle active fades and schedule next LED starts
void updateChristmas() {
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

  // run all three effects concurrently
  // replaced updateChristmas() with fadeLights() per user request
  fadeLights();
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

// New function: fadeLights (renamed from toggleStrands)
// Uses twinkleLights() to make two groups twinkle in alternate phase
void twinkleLights(uint8_t nextDuty[6], const int groupIdx[3], unsigned long baseStart, bool invert) {
  const unsigned long FADE_MS = 300;
  const unsigned long HOLD_MS = 300;
  const unsigned long TOTAL_MS = FADE_MS + HOLD_MS + FADE_MS; // 900

  unsigned long now = millis();
  unsigned long elapsed = (now >= baseStart) ? (now - baseStart) : 0;
  if (invert) {
    // shift by half period to invert phase
    elapsed = (elapsed + (TOTAL_MS / 2)) % TOTAL_MS;
  } else {
    elapsed = elapsed % TOTAL_MS;
  }

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
  static unsigned long baseStart = 0;
  if (baseStart == 0) baseStart = now;

  uint8_t nextDuty[6];
  for (int i = 0; i < 6; ++i) nextDuty[i] = 0;

  // group A normal phase
  twinkleLights(nextDuty, A_idx, baseStart, false);
  // group B inverted phase so they alternate
  twinkleLights(nextDuty, B_idx, baseStart, true);

  // atomically copy
  noInterrupts();
  for (int i = 0; i < 6; ++i) christmasDuty[i] = nextDuty[i];
  interrupts();
}