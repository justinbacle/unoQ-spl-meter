#include "Arduino_LED_Matrix.h"
#include <Wire.h>

// ════════════════════════════════════════════════════════════
// ── Build configuration  (edit here) ────────────────────────
// ════════════════════════════════════════════════════════════

#define DEMO_MODE 0           // 1 = synthetic sweep, 0 = live mic input
#define TEST_PWM_D9   1       // 1 = drive D9 with 500 Hz PWM for A0 bench test, 0 = normal
#define TEST_PWM_DUTY 127U    // analogWrite value (0-255)
#define TEST_PWM_RES  8U      // analogWriteResolution bits

// ── Microphone ──────────────────────────────────────────────
#define NUM_MICS             2
#define MIC_SENSITIVITY_DBV  (-42.0f)  // SPH8878LR5H-1: -42 dBV/Pa
#define CAL_OFFSET           (0.0f)    // per-unit trim (dB)

// ── Sampling ────────────────────────────────────────────────
#define SAMPLE_RATE  48000UL  // Hz
#define BLOCK_SIZE   480      // samples per block → 10 ms
#define ADC_MID      8192     // 14-bit midpoint (2^13)

// ── Display ─────────────────────────────────────────────────
#define DISPLAY_FLIPPED    true
#define ROWS               8
#define COLS               13
#define FONT_ROW           1    // digit top row  (5 px tall, baseline row 5)
#define LETTER_ROW         2    // letter top row (4 px tall, baseline row 5)
#define DIGIT_BRIGHTNESS   7    // 0-7 grayscale: digits at full brightness
#define LETTER_BRIGHTNESS  4    // 0-7 grayscale: mode letter dimmer

// ── Leq window ──────────────────────────────────────────────
#define LEQ_PERIOD  6000U     // blocks  →  6000 × 10 ms = 60 s

// ── Clip warning ────────────────────────────────────────────
#define CLIP_FLASH_MS  200U   // half-period of clip-flash blink (ms)
#define CLIP_FLASH_BG  2U     // brightness for off-pixels during clip flash (0-7); content stays readable

// ── Nav switch (Qwiic PCA9554) ──────────────────────────────
#define NAV_ADDR  0x20        // PCA9554 default I2C address

// ════════════════════════════════════════════════════════════
// ── Types ───────────────────────────────────────────────────
// ════════════════════════════════════════════════════════════

enum WeightMode { MODE_A = 0, MODE_C = 1, MODE_LEQ = 2 };

// Direct-Form-II transposed biquad.
// typedef is required so the Arduino preprocessor can prototype functions
// that take a Biquad& parameter without misinterpreting the signature.
typedef struct { float b0, b1, b2, a1, a2, s1, s2; } Biquad;

// ════════════════════════════════════════════════════════════
// ── Global state ─────────────────────────────────────────────
// ════════════════════════════════════════════════════════════

static const uint8_t MIC_PINS[NUM_MICS] = { A0, A1 };

// Which input to measure: 0..NUM_MICS-1 = single mic, NUM_MICS = average all
static uint8_t    activeMic   = 0;
static WeightMode currentMode = MODE_A;

// Audio buffers
static int16_t sampleBuf[BLOCK_SIZE];
static float   weightBuf[BLOCK_SIZE];

// DC-removal EMA state  (α ≈ 0.99869 → corner ~10 Hz at 48 kHz)
static const float DC_ALPHA = 0.99869f;
static float dcState = 0.0f;

// A-weighting cascade — 3 biquads, Fs=48 kHz, IEC 61672 analog poles
//   stage 0: ~20.6 Hz  double pole
//   stage 1: ~107.7 Hz + ~737.9 Hz poles
//   stage 2: ~12194 Hz double pole
static Biquad aFilt[3] = {
  {1,-2,1,-1.99462f,0.99462f,0,0},
  {1,-2,1,-1.89392f,0.89521f,0,0},
  {1,-2,1,-0.22456f,0.01261f,0,0},
};

// C-weighting cascade — 2 biquads (outer poles only)
static Biquad cFilt[2] = {
  {1,-2,1,-1.99462f,0.99462f,0,0},
  {1,-2,1,-0.22456f,0.01261f,0,0},
};

// Leq accumulator
static float    leqEnergySum  = 0.0f;
static uint16_t leqBlockCount = 0;
static float    leqDB         = 0.0f;

// Navigation switch (Qwiic PCA9554 via Wire1 — no external library needed)
static bool    navPresent   = false;
static uint8_t navLastState = 0xFF; // 0xFF = all buttons released

// Last displayed integer dB — promoted from loop() so nav switch can reset it
static int  lastDisplayed = -1;

// ADC clipping flag — set in collectBlock(), read in commitFrame()
static bool clipDetected  = false;

// Display
static ArduinoLEDMatrix matrix;
static uint8_t frameBuf[ROWS][COLS];

// 3×5 digit font
static const uint8_t DIGIT[10][5] = {
  {0b111,0b101,0b101,0b101,0b111},
  {0b010,0b110,0b010,0b010,0b111},
  {0b111,0b001,0b111,0b100,0b111},
  {0b111,0b001,0b011,0b001,0b111},
  {0b101,0b101,0b111,0b001,0b001},
  {0b111,0b100,0b111,0b001,0b111},
  {0b011,0b100,0b111,0b101,0b111},
  {0b111,0b001,0b010,0b010,0b010},
  {0b111,0b101,0b111,0b101,0b111},
  {0b111,0b101,0b111,0b001,0b011},
};

// 3×4 mode letters: A, C, L
static const uint8_t LETTER[3][4] = {
  {0b010,0b101,0b111,0b101}, // A
  {0b011,0b100,0b100,0b011}, // C
  {0b100,0b100,0b100,0b111}, // L
};

#if DEMO_MODE
static int  simDB   = 35;
static bool sweepUp = true;
#endif

// Expected SPL for the D9 test signal, computed once in setup()
#if TEST_PWM_D9
static float testExpectedDB = 0.0f;
#endif

// Runtime calibration offset (dB) — auto-set by TEST_PWM_D9 calibration, 0 otherwise
static float runtimeCalOffset = 0.0f;

// ════════════════════════════════════════════════════════════
// ── Audio pipeline ───────────────────────────────────────────
// ════════════════════════════════════════════════════════════

static void collectBlock() {
  const uint32_t tickUs = 1000000UL / SAMPLE_RATE;
  uint32_t tNext = micros();
  clipDetected = false;
  for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
    if (activeMic < NUM_MICS) {
      int16_t raw   = (int16_t)analogRead(MIC_PINS[activeMic]);
      sampleBuf[i]  = raw - ADC_MID;
      if (raw <= 100 || raw >= 16283) clipDetected = true;
    } else {
      int32_t sum = 0;
      for (uint8_t m = 0; m < NUM_MICS; m++) {
        int16_t raw = (int16_t)analogRead(MIC_PINS[m]);
        if (raw <= 100 || raw >= 16283) clipDetected = true;
        sum += raw;
      }
      sampleBuf[i] = (int16_t)(sum / NUM_MICS) - ADC_MID;
    }
    tNext += tickUs;
    while ((int32_t)(micros() - tNext) < 0) {}
  }
}

static void removeDC() {
  for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
    dcState      = DC_ALPHA * dcState + (1.0f - DC_ALPHA) * sampleBuf[i];
    sampleBuf[i] = (int16_t)(sampleBuf[i] - dcState);
  }
}

static float applyBiquad(Biquad &bq, float x) {
  float y = bq.b0 * x + bq.s1;
  bq.s1   = bq.b1 * x - bq.a1 * y + bq.s2;
  bq.s2   = bq.b2 * x - bq.a2 * y;
  return y;
}

static void applyWeighting(WeightMode mode) {
  for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
    float x = (float)sampleBuf[i];
    if (mode == MODE_C) {
      x = applyBiquad(cFilt[0], x);
      x = applyBiquad(cFilt[1], x);
    } else {
      x = applyBiquad(aFilt[0], x);
      x = applyBiquad(aFilt[1], x);
      x = applyBiquad(aFilt[2], x);
    }
    weightBuf[i] = x;
  }
}

// Returns the mean-square energy of the weighted block (no log/sqrt here).
static float computeEnergy() {
  float sum = 0.0f;
  for (uint16_t i = 0; i < BLOCK_SIZE; i++)
    sum += weightBuf[i] * weightBuf[i];
  return sum / BLOCK_SIZE;
}

// Convert mean-square ADC energy to dB SPL.
static float energyToDB(float ms) {
  float vRms = sqrtf(ms) / 16383.0f * 3.3f;
  if (vRms < 1e-10f) vRms = 1e-10f;
  return 20.0f * (logf(vRms) * 0.4342944819f) - MIC_SENSITIVITY_DBV + 94.0f + CAL_OFFSET + runtimeCalOffset;
}

// Accumulate mean-square energy directly — no powf() needed.
static void accumulateLeq(float ms) {
  leqEnergySum += ms;
  if (++leqBlockCount >= LEQ_PERIOD) {
    leqDB         = energyToDB(leqEnergySum / LEQ_PERIOD);
    leqEnergySum  = 0.0f;
    leqBlockCount = 0;
  }
}

// ════════════════════════════════════════════════════════════
// ── Display pipeline ─────────────────────────────────────────
// ════════════════════════════════════════════════════════════

static void stampGlyph(const uint8_t glyph[5], uint8_t col, uint8_t row) {
  for (uint8_t r = 0; r < 5 && (row + r) < ROWS; r++)
    for (uint8_t c = 0; c < 3 && (col + c) < COLS; c++)
      frameBuf[row + r][col + c] = ((glyph[r] >> (2u - c)) & 0x01u) ? DIGIT_BRIGHTNESS : 0;
}

static void stampSmallGlyph(const uint8_t glyph[4], uint8_t col, uint8_t row) {
  for (uint8_t r = 0; r < 4 && (row + r) < ROWS; r++)
    for (uint8_t c = 0; c < 3 && (col + c) < COLS; c++)
      frameBuf[row + r][col + c] = ((glyph[r] >> (2u - c)) & 0x01u) ? LETTER_BRIGHTNESS : 0;
}

static void stampNarrowOne(uint8_t col, uint8_t row) {
  for (uint8_t r = 0; r < 5 && (row + r) < ROWS; r++)
    frameBuf[row + r][col] = DIGIT_BRIGHTNESS;
}

static void rotateFrame180() {
  const uint8_t total = ROWS * COLS;
  for (uint8_t i = 0; i < total / 2; i++) {
    uint8_t j   = total - 1u - i;
    uint8_t tmp = frameBuf[i / COLS][i % COLS];
    frameBuf[i / COLS][i % COLS] = frameBuf[j / COLS][j % COLS];
    frameBuf[j / COLS][j % COLS] = tmp;
  }
}

static void displaySPL(int db, WeightMode mode) {
  if (db < 0) db = 0;
  if (db > 199) db = 199; // display can only render 0-199 cleanly
  // Clear only the digit/letter rows — bottom row (row 7) is owned by updateBottomRow
  for (uint8_t r = 0; r < ROWS - 1; r++) memset(frameBuf[r], 0, COLS);
  if (db >= 100) stampNarrowOne(0, FONT_ROW);
  stampGlyph(DIGIT[(db % 100) / 10], 2, FONT_ROW);
  stampGlyph(DIGIT[db % 10],         6, FONT_ROW);
  stampSmallGlyph(LETTER[mode], 10, LETTER_ROW);
}

// Bottom row (row 7): VU bar cols 0-9, gap col 10, mic dots cols 11-12.
// db range 40-110 → 0-10 lit segments.
static void updateBottomRow(float db) {
  uint8_t bars = (uint8_t)constrain((db - 40.0f) * 10.0f / 70.0f, 0.0f, 10.0f);
  for (uint8_t c = 0; c < 10; c++)
    frameBuf[7][c] = (c < bars) ? DIGIT_BRIGHTNESS : 0;
  frameBuf[7][10] = 0; // gap
  // Mic dots: full bright = selected, dim = available-but-not-selected
  frameBuf[7][11] = (activeMic == 0 || activeMic == NUM_MICS) ? DIGIT_BRIGHTNESS : 1;
  frameBuf[7][12] = (activeMic == 1 || activeMic == NUM_MICS) ? DIGIT_BRIGHTNESS : 1;
}

// Rotate frameBuf 180°, push to matrix, restore. frameBuf stays in logical space.
// When ADC clipping is detected, alternates between normal and all-bright frames at ~1.5 Hz.
static void commitFrame() {
  static bool     clipFlash    = false;
  static uint32_t clipFlipMs   = 0;
  bool doFlash = false;

  if (clipDetected) {
    uint32_t now = millis();
    if (now - clipFlipMs >= CLIP_FLASH_MS) {
      clipFlipMs = now;
      clipFlash  = !clipFlash;
    }
    doFlash = clipFlash;
  } else {
    clipFlash  = false;
    clipFlipMs = 0;
  }

  uint8_t savedBuf[ROWS][COLS];
  if (doFlash) {
    memcpy(savedBuf, frameBuf, sizeof(frameBuf));
    // Only light up off-pixels — content (digits, letter, VU) stays readable
    for (uint8_t r = 0; r < ROWS; r++)
      for (uint8_t c = 0; c < COLS; c++)
        if (frameBuf[r][c] == 0) frameBuf[r][c] = CLIP_FLASH_BG;
  }

  if (DISPLAY_FLIPPED) rotateFrame180();
  noInterrupts();
  matrix.draw((uint8_t*)frameBuf);
  interrupts();
  if (DISPLAY_FLIPPED) rotateFrame180();

  if (doFlash) memcpy(frameBuf, savedBuf, sizeof(frameBuf)); // restore logical state
}

// ════════════════════════════════════════════════════════════
// ── Nav switch control ───────────────────────────────────────
// ════════════════════════════════════════════════════════════

// PCA9554 register addresses
#define PCA9554_REG_IN   0  // input port
#define PCA9554_REG_OUT  1  // output port
#define PCA9554_REG_CFG  3  // configuration (1=input, 0=output)

// PCA9554 GPIO pin mapping (buttons active LOW, LEDs active LOW)
static const uint8_t NAV_UP     = 0;
static const uint8_t NAV_DOWN   = 1;
static const uint8_t NAV_RIGHT  = 2;
static const uint8_t NAV_LEFT   = 3;
static const uint8_t NAV_CENTER = 4;
static const uint8_t NAV_LED_B  = 5;
static const uint8_t NAV_LED_G  = 6;
static const uint8_t NAV_LED_R  = 7;

// Set RGB LED to reflect current mode: A=red, C=green, Leq=blue
static void updateNavLED() {
  if (!navPresent) return;
  uint8_t out = 0xFF;  // all HIGH = all LEDs off (active LOW)
  if (currentMode == MODE_A)   out &= ~(1u << NAV_LED_R);
  if (currentMode == MODE_C)   out &= ~(1u << NAV_LED_G);
  if (currentMode == MODE_LEQ) out &= ~(1u << NAV_LED_B);
  Wire1.beginTransmission(NAV_ADDR);
  Wire1.write(PCA9554_REG_OUT);
  Wire1.write(out);
  Wire1.endTransmission();
}

static void setupNavSwitch() {
  Wire1.begin();
  // Probe: PCA9554 ACKs if present
  Wire1.beginTransmission(NAV_ADDR);
  navPresent = (Wire1.endTransmission() == 0);
  if (!navPresent) {
    Serial.println(F("Nav switch not found on Qwiic (non-fatal)"));
    return;
  }
  // GPIO0-4 = inputs (buttons), GPIO5-7 = outputs (LEDs)
  Wire1.beginTransmission(NAV_ADDR);
  Wire1.write(PCA9554_REG_CFG);
  Wire1.write(0x1F);  // 0b00011111: lower 5 input, upper 3 output
  Wire1.endTransmission();
  // All LEDs off initially
  Wire1.beginTransmission(NAV_ADDR);
  Wire1.write(PCA9554_REG_OUT);
  Wire1.write(0xFF);
  Wire1.endTransmission();
  updateNavLED();
  Serial.println(F("Nav switch online"));
}

// Called every 10 ms (once per audio block).
// Detects falling edges (released→pressed) and acts on them.
static void pollNavSwitch() {
  if (!navPresent) return;

  // Read input port register
  Wire1.beginTransmission(NAV_ADDR);
  Wire1.write(PCA9554_REG_IN);
  Wire1.endTransmission();
  Wire1.requestFrom((uint8_t)NAV_ADDR, (uint8_t)1);
  if (!Wire1.available()) return;
  uint8_t data = Wire1.read();

  // Bit = 1 → released, 0 → pressed.  fell = bits newly pressed this cycle.
  uint8_t fell = navLastState & ~data & 0x1Fu;
  navLastState  = data;
  if (!fell) return;

  if (fell & (1u << NAV_RIGHT)) {                                // next mode
    currentMode   = static_cast<WeightMode>((currentMode + 1) % 3);
    lastDisplayed = -1;
    updateNavLED();
  }
  if (fell & (1u << NAV_LEFT)) {                                 // prev mode
    currentMode   = static_cast<WeightMode>((currentMode + 2) % 3);
    lastDisplayed = -1;
    updateNavLED();
  }
  if (fell & (1u << NAV_UP)) {                                   // next mic
    activeMic     = (activeMic + 1) % (NUM_MICS + 1);
    lastDisplayed = -1;
  }
  if (fell & (1u << NAV_DOWN)) {                                 // prev mic
    activeMic     = (activeMic + NUM_MICS) % (NUM_MICS + 1);
    lastDisplayed = -1;
  }
  if (fell & (1u << NAV_CENTER)) {                               // reset Leq
    leqEnergySum  = 0.0f;
    leqBlockCount = 0;
    leqDB         = 0.0f;
    lastDisplayed = -1;
  }
}

// ════════════════════════════════════════════════════════════
// ── Arduino entry points ─────────────────────────────────────
// ════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  matrix.begin();
  matrix.setGrayscaleBits(3);  // 8 brightness levels (0-7)
  analogReadResolution(14);
  setupNavSwitch();
#if TEST_PWM_D9
  pinMode(D9, OUTPUT);
  analogWriteResolution(TEST_PWM_RES);
  analogWrite(D9, TEST_PWM_DUTY);
  // Theoretical SPL: square wave duty d → V_rms = Vdd * sqrt(d*(1-d))
  {
    float d = TEST_PWM_DUTY / ((float)((1u << TEST_PWM_RES) - 1u));
    float vRms = 3.3f * sqrtf(d * (1.0f - d));
    testExpectedDB = 20.0f * (logf(vRms) * 0.4342944819f)
                     - MIC_SENSITIVITY_DBV + 94.0f + CAL_OFFSET;
  }
  Serial.print(F("TEST_PWM_D9: duty=")); Serial.print(TEST_PWM_DUTY);
  Serial.print(F("/")); Serial.print((1u << TEST_PWM_RES) - 1u);
  Serial.print(F("  expected=")); Serial.print(testExpectedDB, 1);
  Serial.println(F(" dBSPL (unweighted, ideal rails)"));
  // Auto-calibrate: warm up filters then average 50 blocks against the known signal
  Serial.println(F("  calibrating..."));
  for (uint8_t i = 0; i < 20; i++) { collectBlock(); removeDC(); applyWeighting(MODE_A); } // warm-up
  float calEnergySum = 0.0f;
  for (uint8_t i = 0; i < 50; i++) { collectBlock(); removeDC(); applyWeighting(MODE_A); calEnergySum += computeEnergy(); }
  float measuredDB = energyToDB(calEnergySum / 50.0f); // runtimeCalOffset still 0 here
  runtimeCalOffset = testExpectedDB - measuredDB;
  Serial.print(F("  measured=")); Serial.print(measuredDB, 1);
  Serial.print(F("  cal_offset=")); Serial.println(runtimeCalOffset, 1);
#endif
  Serial.println(DEMO_MODE ? F("mobile_spl DEMO sweep") : F("mobile_spl live"));
}

void loop() {
#if DEMO_MODE
  WeightMode mode = static_cast<WeightMode>((simDB / 10) % 3);
  displaySPL(simDB, mode);
  Serial.print(F("sim dB = ")); Serial.println(simDB);
  sweepUp ? simDB++ : simDB--;
  if (simDB >= 130) sweepUp = false;
  if (simDB <= 35)  sweepUp = true;
  delay(150);
#else
  static uint8_t blockCount = 0;
  static uint8_t vuCount    = 0;

  pollNavSwitch();
  collectBlock();
  removeDC();
  applyWeighting(currentMode);
  float energy = computeEnergy();
  float db     = energyToDB(energy);
  accumulateLeq(energy);

  // VU meter + mic dots: refresh every 5 blocks (50 ms)
  if (++vuCount >= 5) {
    vuCount = 0;
    float val    = (currentMode == MODE_LEQ) ? leqDB : db;
    int   intVal = (int)(val + 0.5f);
    if (intVal != lastDisplayed) {
      lastDisplayed = intVal;
      displaySPL(intVal, currentMode);
    }
    updateBottomRow(db);
    commitFrame();
  }

  if (++blockCount >= 25) {
    blockCount = 0;
    float val = (currentMode == MODE_LEQ) ? leqDB : db;
#if TEST_PWM_D9
    Serial.print(F("measured=")); Serial.print(val, 1);
    Serial.print(F("  expected=")); Serial.print(testExpectedDB, 1);
    Serial.print(F("  delta=")); Serial.println(val - testExpectedDB, 1);
#else
    Serial.print(F("dB = ")); Serial.println(val, 1);
#endif
  }
#endif
}

