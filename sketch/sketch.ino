#include "Arduino_LED_Matrix.h"
#include <Arduino_RouterBridge.h>
#include <Wire.h>

// ════════════════════════════════════════════════════════════
// ── Build configuration  (edit here) ────────────────────────
// ════════════════════════════════════════════════════════════

#define DEMO_MODE 0           // 1 = synthetic sweep, 0 = live mic input
#define TEST_PWM_D9   0       // 1 = drive D9 with 500 Hz PWM for A0 bench test, 0 = normal
#define TEST_PWM_DUTY 127U    // analogWrite value (0-255)
#define TEST_PWM_RES  8U      // analogWriteResolution bits

// ── Microphone ──────────────────────────────────────────────
#define NUM_MICS             2
// SPH8878LR5H-1 raw: -44 dBV/Pa typical (single-ended)
// SparkFun breakout OPA344 gain: 64x = +36.12 dB
// Effective breakout sensitivity: -44 + 36.12 = -7.88 dBV/Pa
#define MIC_SENSITIVITY_DBV  (-7.9f)
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

// ── Leq partial-period indicator ────────────────────────────
#define LEQ_PARTIAL_BRIGHTNESS 3U // digit brightness before first full period completes

// ── Nav switch (Qwiic PCA9554) ──────────────────────────────
#define NAV_ADDR  0x20        // PCA9554 default I2C address

// ════════════════════════════════════════════════════════════
// ── Types ───────────────────────────────────────────────────
// ════════════════════════════════════════════════════════════

enum WeightMode { MODE_A = 0, MODE_C = 1, MODE_LEQ = 2, MODE_DIAG = 3 };
#define NUM_MODES 4

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
static volatile WeightMode currentMode = MODE_A;

// Audio buffers
static int16_t sampleBuf[BLOCK_SIZE];
static float   weightBuf[BLOCK_SIZE];

// DC-removal EMA state  (α ≈ 0.99869 → corner ~10 Hz at 48 kHz)
static const float DC_ALPHA = 0.99869f;
static float dcState = 0.0f;

// A-weighting cascade — 3 biquads, Fs=48 kHz, IEC 61672
//   stage 0: HP 20.6 Hz double pole + 2 zeros at DC
//   stage 1: LP 107.7 Hz + 737.9 Hz poles (NO zeros at DC)
//   stage 2: HP 12194 Hz double pole + 2 zeros at DC
// Per-stage normalized so b coefficients stay in float32 range.
static Biquad aFilt[3] = {
  { 0.9977310085f,-1.9954620170f, 0.9977310085f, -1.9946144527f, 0.9946217038f, 0,0},
  { 0.0050850145f, 0.0101700290f, 0.0050850145f, -1.8938018760f, 0.8950921285f, 0,0},
  {59.9270479676f,-119.854095935f,59.9270479676f,  0.0254243152f, 0.0001615990f, 0,0},
};

// C-weighting cascade — 2 biquads, Fs=48 kHz, IEC 61672
//   stage 0: HP 20.6 Hz double pole + 2 zeros at DC
//   stage 1: LP 12194 Hz double pole (NO zeros at DC)
static Biquad cFilt[2] = {
  {0.9977310085f,-1.9954620170f, 0.9977310085f, -1.9946144527f, 0.9946217038f, 0,0},
  {0.2574433331f, 0.5148866662f, 0.2574433331f,  0.0254243152f, 0.0001615990f, 0,0},
};

// Leq accumulator
static float    leqEnergySum     = 0.0f;
static uint16_t leqBlockCount    = 0;
static float    leqDB            = 0.0f;
static bool     leqPeriodComplete = false; // true after first full 60-s period

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

// 3×4 mode letters: A, C, L, D
static const uint8_t LETTER[4][4] = {
  {0b010,0b101,0b111,0b101}, // A
  {0b011,0b100,0b100,0b011}, // C
  {0b100,0b100,0b100,0b111}, // L
  {0b110,0b101,0b101,0b110}, // D (diagnostic)
};

// Diagnostic sub-modes: what value to display
enum DiagSub { DIAG_RAW_RMS = 0, DIAG_WT_RMS = 1, DIAG_RAW_MV = 2, DIAG_DB_UNWEIGHTED = 3 };
#define NUM_DIAG_SUBS 4
static DiagSub diagSub = DIAG_RAW_RMS;
static float diagRawMs = 0.0f;   // raw mean-square (latest block)
static float diagWtMs  = 0.0f;   // weighted mean-square (latest block)

// VU refresh counter — file scope so nav handler can force immediate redraw
static uint8_t vuCount = 0;

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
// leqDB is updated every block as a running average, so it shows a live
// partial Leq immediately rather than staying at 0 for the first full period.
static void accumulateLeq(float ms) {
  leqEnergySum += ms;
  ++leqBlockCount;
  leqDB = energyToDB(leqEnergySum / leqBlockCount); // always up-to-date
  if (leqBlockCount >= LEQ_PERIOD) {
    leqPeriodComplete = true;
    leqEnergySum  = 0.0f;
    leqBlockCount = 0;
    // leqDB retains the just-computed full-period value until next update
  }
}

// ════════════════════════════════════════════════════════════
// ── Display pipeline ─────────────────────────────────────────
// ════════════════════════════════════════════════════════════

static void stampGlyph(const uint8_t glyph[5], uint8_t col, uint8_t row, uint8_t bright = DIGIT_BRIGHTNESS) {
  for (uint8_t r = 0; r < 5 && (row + r) < ROWS; r++)
    for (uint8_t c = 0; c < 3 && (col + c) < COLS; c++)
      frameBuf[row + r][col + c] = ((glyph[r] >> (2u - c)) & 0x01u) ? bright : 0;
}

static void stampSmallGlyph(const uint8_t glyph[4], uint8_t col, uint8_t row) {
  for (uint8_t r = 0; r < 4 && (row + r) < ROWS; r++)
    for (uint8_t c = 0; c < 3 && (col + c) < COLS; c++)
      frameBuf[row + r][col + c] = ((glyph[r] >> (2u - c)) & 0x01u) ? LETTER_BRIGHTNESS : 0;
}

static void stampNarrowOne(uint8_t col, uint8_t row, uint8_t bright = DIGIT_BRIGHTNESS) {
  for (uint8_t r = 0; r < 5 && (row + r) < ROWS; r++)
    frameBuf[row + r][col] = bright;
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

static void displaySPL(int db, WeightMode mode, bool partial = false) {
  if (db < 0) db = 0;
  if (db > 199) db = 199; // display can only render 0-199 cleanly
  // Clear only the digit/letter rows — bottom row (row 7) is owned by updateBottomRow
  for (uint8_t r = 0; r < ROWS - 1; r++) memset(frameBuf[r], 0, COLS);
  uint8_t dBright = partial ? (uint8_t)LEQ_PARTIAL_BRIGHTNESS : (uint8_t)DIGIT_BRIGHTNESS;
  if (db >= 100) stampNarrowOne(0, FONT_ROW, dBright);
  stampGlyph(DIGIT[(db % 100) / 10], 2, FONT_ROW, dBright);
  stampGlyph(DIGIT[db % 10],         6, FONT_ROW, dBright);
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
  if (currentMode == MODE_A)    out &= ~(1u << NAV_LED_R);
  if (currentMode == MODE_C)    out &= ~(1u << NAV_LED_G);
  if (currentMode == MODE_LEQ)  out &= ~(1u << NAV_LED_B);
  if (currentMode == MODE_DIAG) { out &= ~(1u << NAV_LED_R); out &= ~(1u << NAV_LED_G); } // yellow
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
    Monitor.println(F("Nav switch not found on Qwiic (non-fatal)"));
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
  Monitor.println(F("Nav switch online"));
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
    currentMode   = static_cast<WeightMode>((currentMode + 1) % NUM_MODES);
    lastDisplayed = -1;
    updateNavLED();
  }
  if (fell & (1u << NAV_LEFT)) {                                 // prev mode
    currentMode   = static_cast<WeightMode>((currentMode + NUM_MODES - 1) % NUM_MODES);
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
  if (fell & (1u << NAV_CENTER)) {
    if ((int)currentMode == 3) {                                 // cycle diag sub-mode
      diagSub = static_cast<DiagSub>((diagSub + 1) % NUM_DIAG_SUBS);
      lastDisplayed = -1;
      vuCount = 4;                                               // force redraw next block
      updateNavLED();                                            // blink LED as confirmation
    } else {                                                     // reset Leq
      leqEnergySum      = 0.0f;
      leqBlockCount     = 0;
      leqDB             = 0.0f;
      leqPeriodComplete = false;
      lastDisplayed     = -1;
    }
  }
}

// ════════════════════════════════════════════════════════════
// ── Arduino entry points ─────────────────────────────────────
// ════════════════════════════════════════════════════════════

void setup() {
  Bridge.begin();
  Monitor.begin();
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
  Monitor.print(F("TEST_PWM_D9: duty=")); Monitor.print(TEST_PWM_DUTY);
  Monitor.print(F("/")); Monitor.print((1u << TEST_PWM_RES) - 1u);
  Monitor.print(F("  expected=")); Monitor.print(testExpectedDB, 1);
  Monitor.println(F(" dBSPL (unweighted, ideal rails)"));
  // Auto-calibrate: warm up filters then average 50 blocks against the known signal
  Monitor.println(F("  calibrating..."));
  for (uint8_t i = 0; i < 20; i++) { collectBlock(); removeDC(); applyWeighting(MODE_A); } // warm-up
  float calEnergySum = 0.0f;
  for (uint8_t i = 0; i < 50; i++) { collectBlock(); removeDC(); applyWeighting(MODE_A); calEnergySum += computeEnergy(); }
  float measuredDB = energyToDB(calEnergySum / 50.0f); // runtimeCalOffset still 0 here
  runtimeCalOffset = testExpectedDB - measuredDB;
  Monitor.print(F("  measured=")); Monitor.print(measuredDB, 1);
  Monitor.print(F("  cal_offset=")); Monitor.println(runtimeCalOffset, 1);
#endif
  Monitor.println(DEMO_MODE ? F("mobile_spl DEMO sweep") : F("mobile_spl live"));
}

void loop() {
#if DEMO_MODE
  WeightMode mode = static_cast<WeightMode>((simDB / 10) % 3);
  displaySPL(simDB, mode);
  Monitor.print(F("sim dB = ")); Monitor.println(simDB);
  sweepUp ? simDB++ : simDB--;
  if (simDB >= 130) sweepUp = false;
  if (simDB <= 35)  sweepUp = true;
  delay(150);
#else
  static uint8_t blockCount = 0;

  pollNavSwitch();
  collectBlock();
  removeDC();

  // Compute raw (unweighted) energy every block for diagnostics
  {
    float rawSum = 0.0f;
    for (uint16_t i = 0; i < BLOCK_SIZE; i++)
      rawSum += (float)sampleBuf[i] * (float)sampleBuf[i];
    diagRawMs = rawSum / BLOCK_SIZE;
  }

  // Apply weighting — DIAG mode still runs A-weight so LEQ stays valid
  applyWeighting((currentMode == MODE_DIAG) ? MODE_A : currentMode);
  float energy = computeEnergy();
  diagWtMs = energy;
  float db     = energyToDB(energy);
  accumulateLeq(energy);

  // VU meter + mic dots: refresh every 5 blocks (50 ms)
  if (++vuCount >= 5) {
    vuCount = 0;
    if ((int)currentMode == 3) {
      // Diagnostic display: show raw values depending on diagSub
      float diagVal = 0.0f;
      switch (diagSub) {
        case DIAG_RAW_RMS:       diagVal = sqrtf(diagRawMs); break;       // ADC counts RMS
        case DIAG_WT_RMS:        diagVal = sqrtf(diagWtMs); break;        // weighted ADC counts RMS
        case DIAG_RAW_MV:        diagVal = sqrtf(diagRawMs) / 16383.0f * 3300.0f; break; // millivolts
        case DIAG_DB_UNWEIGHTED: diagVal = energyToDB(diagRawMs); break;  // dB from raw signal
      }
      int intVal = (int)(diagVal + 0.5f);
      // Force redraw every cycle in diag mode
      lastDisplayed = -1;
      // Clear ALL rows including bottom
      memset(frameBuf, 0, sizeof(frameBuf));
      // Show value as up to 2 digits + sub-mode number
      if (intVal > 99) intVal = 99;
      if (intVal < 0)  intVal = 0;
      stampGlyph(DIGIT[intVal / 10], 2, FONT_ROW);
      stampGlyph(DIGIT[intVal % 10], 6, FONT_ROW);
      // Show sub-mode number (1-4) at full brightness
      stampGlyph(DIGIT[(uint8_t)diagSub + 1], 10, FONT_ROW, DIGIT_BRIGHTNESS);
      // Bottom row: bar length indicates sub-mode
      for (uint8_t c = 0; c < 10; c++)
        frameBuf[7][c] = (c < (10u - (uint8_t)diagSub * 3u)) ? DIGIT_BRIGHTNESS : 0;
    } else {
      float val    = (currentMode == MODE_LEQ) ? leqDB : db;
      int   intVal = (int)(val + 0.5f);
      if (intVal != lastDisplayed) {
        lastDisplayed = intVal;
        bool partial = (currentMode == MODE_LEQ) && !leqPeriodComplete;
        displaySPL(intVal, currentMode, partial);
      }
      updateBottomRow(db);
    }
    commitFrame();
  }

  if (++blockCount >= 25) {
    blockCount = 0;
    // Compute raw (unweighted) energy for diagnostics
    float rawSum = 0.0f;
    for (uint16_t i = 0; i < BLOCK_SIZE; i++)
      rawSum += (float)sampleBuf[i] * (float)sampleBuf[i];
    float rawMs = rawSum / BLOCK_SIZE;
    float rawRms = sqrtf(rawMs);
    float rawVrms = rawRms / 16383.0f * 3.3f;
    float weightedRms = sqrtf(energy);
    float weightedVrms = weightedRms / 16383.0f * 3.3f;

    float val = (currentMode == MODE_LEQ) ? leqDB : db;
#if TEST_PWM_D9
    Monitor.print(F("measured=")); Monitor.print(val, 1);
    Monitor.print(F("  expected=")); Monitor.print(testExpectedDB, 1);
    Monitor.print(F("  delta=")); Monitor.println(val - testExpectedDB, 1);
#else
    Monitor.print(F("raw_adc_rms=")); Monitor.print(rawRms, 1);
    Monitor.print(F("  raw_mV=")); Monitor.print(rawVrms * 1000.0f, 3);
    Monitor.print(F("  wt_adc_rms=")); Monitor.print(weightedRms, 1);
    Monitor.print(F("  wt_mV=")); Monitor.print(weightedVrms * 1000.0f, 3);
    Monitor.print(F("  dB=")); Monitor.println(val, 1);
#endif
  }
#endif
}

