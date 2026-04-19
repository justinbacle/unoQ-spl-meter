#include "Arduino_LED_Matrix.h"
#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include "arduinoFFT.h"

// ════════════════════════════════════════════════════════════
// ── Build configuration  (edit here) ────────────────────────
// ════════════════════════════════════════════════════════════

// ── Microphone ──────────────────────────────────────────────
#define NUM_MICS             2
// SPH8878LR5H-1 raw: -44 dBV/Pa typical (single-ended)
// SparkFun breakout OPA344 gain: 64x = +36.12 dB
// Effective breakout sensitivity: -44 + 36.12 = -7.88 dBV/Pa
#define MIC_SENSITIVITY_DBV  (-7.9f)
#define CAL_OFFSET           (0.0f)    // per-unit trim (dB)

// ── Sampling ────────────────────────────────────────────────
#define SAMPLE_RATE  48000UL  // Hz
#define BLOCK_SIZE   480 * 2      // samples per block → 10 ms
// deoubled size, need to check if calibration was affected
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
#define LEQ_PERIOD_MS  60000UL  // 60-second integration window (wall clock)

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

enum WeightMode { MODE_A = 0, MODE_C = 1, MODE_LAEQ = 2, MODE_LCEQ = 3, MODE_SPECTRUM = 4 };
#define NUM_MODES 5

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
// Generated via scipy bilinear_zpk + zpk2sos, normalized to 0 dB at 1 kHz.
//   stage 0: LP  12194 Hz double pole (gain-scaled for 0 dB @ 1 kHz)
//   stage 1: HP  zeros at DC + 107.7/737.9 Hz poles
//   stage 2: HP  zeros at DC + 20.6 Hz double pole
static Biquad aFilt[3] = {
  { 0.2341830426f, 0.4683660852f, 0.2341830426f, -0.2245584581f, 0.0126066253f, 0,0},
  { 1.0000000000f,-2.0000000000f, 1.0000000000f, -1.8938704947f, 0.8951597691f, 0,0},
  { 1.0000000000f,-2.0000000000f, 1.0000000000f, -1.9946144560f, 0.9946217070f, 0,0},
};

// C-weighting cascade — 2 biquads, Fs=48 kHz, IEC 61672
// Generated via scipy bilinear_zpk + zpk2sos, normalized to 0 dB at 1 kHz.
//   stage 0: LP  12194 Hz double pole (gain-scaled for 0 dB @ 1 kHz)
//   stage 1: HP  zeros at DC + 20.6 Hz double pole
static Biquad cFilt[2] = {
  { 0.1978907070f, 0.3957814140f, 0.1978907070f, -0.2245584581f, 0.0126066253f, 0,0},
  { 1.0000000000f,-2.0000000000f, 1.0000000000f, -1.9946144560f, 0.9946217070f, 0,0},
};

// ── Spectrum mode (MODE_SPECTRUM) ───────────────────────────
// FFT_SIZE is automatically the smallest power-of-2 ≥ BLOCK_SIZE.
// At 48 kHz: bin width = SAMPLE_RATE / FFT_SIZE Hz.
constexpr uint16_t nextPow2(uint16_t v, uint16_t p = 1) {
  return p >= v ? p : nextPow2(v, p * 2u);
}
static constexpr uint16_t FFT_SIZE = nextPow2(BLOCK_SIZE);
static_assert(FFT_SIZE >= 512, "FFT_SIZE < 512: band table invalid, keep BLOCK_SIZE >= 257");
#define NUM_BANDS 13
// Band edges are defined for a 512-pt FFT reference; SCALE_BIN adjusts to the actual FFT_SIZE.
// Doubling FFT_SIZE halves the bin width, so all bin numbers double — the Hz ranges stay the same.
#define SCALE_BIN(b) ((uint16_t)((uint32_t)(b) * FFT_SIZE / 512u))
static const uint16_t SPEC_START[NUM_BANDS] = { SCALE_BIN(  1), SCALE_BIN(  2), SCALE_BIN(  3), SCALE_BIN(  4), SCALE_BIN(  6), SCALE_BIN(  9), SCALE_BIN( 13), SCALE_BIN( 20), SCALE_BIN( 31), SCALE_BIN( 47), SCALE_BIN( 72), SCALE_BIN(109), SCALE_BIN(167) };
static const uint16_t SPEC_END[NUM_BANDS]   = { SCALE_BIN(  1), SCALE_BIN(  2), SCALE_BIN(  3), SCALE_BIN(  5), SCALE_BIN(  8), SCALE_BIN( 12), SCALE_BIN( 19), SCALE_BIN( 30), SCALE_BIN( 46), SCALE_BIN( 71), SCALE_BIN(108), SCALE_BIN(166), SCALE_BIN(255) };
// dB display range: SPEC_DB_FLOOR (bottom pixel) to SPEC_DB_FLOOR+SPEC_DB_RANGE (top pixel)
#define SPEC_DB_FLOOR 30.0f
#define SPEC_DB_RANGE 80.0f   // 8 rows × 10 dB/row

static float fftReal[FFT_SIZE];         // windowed input / real output after FFT
static float fftImag[FFT_SIZE];         // imaginary output after FFT
static ArduinoFFT<float> fft(fftReal, fftImag, FFT_SIZE, (float)SAMPLE_RATE);
static float specDB[NUM_BANDS];         // EMA-smoothed per-band dB SPL

// Leq accumulators: [0]=A-weighted (LAeq), [1]=C-weighted (LCeq)
static float    leqEnergySum[2]      = {0.0f, 0.0f};
static uint32_t leqBlockCount[2]     = {0, 0};  // blocks in current period (divisor only)
static uint32_t leqStartMs[2]        = {0, 0};  // millis() at start of current period
static float    leqDB[2]             = {0.0f, 0.0f};
static bool     leqPeriodComplete[2]  = {false, false};

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

// 3×4 mode letters: A, C, L, L
static const uint8_t LETTER[4][4] = {
  {0b010,0b101,0b111,0b101}, // A
  {0b011,0b100,0b100,0b011}, // C
  {0b100,0b100,0b100,0b111}, // L (LAeq)
  {0b100,0b100,0b100,0b111}, // L (LCeq)
};



// VU refresh counter — file scope so nav handler can force immediate redraw
static uint8_t vuCount = 0;

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

// Compute per-band dB SPL from the most recent sampleBuf via 512-pt RFFT.
// Applies a Hann window, zero-pads to 512, then bins FFT output into NUM_BANDS
// log-spaced bands. Results are EMA-smoothed into specDB[].
static void computeSpectrum() {
  // Fill fftReal: Hann-windowed samples + zero-pad; clear fftImag
  const float invN1 = 1.0f / (float)(BLOCK_SIZE - 1);
  for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
    float w = 0.5f * (1.0f - cosf(6.2831853f * i * invN1));
    fftReal[i] = (float)sampleBuf[i] * w;
  }
  for (uint16_t i = BLOCK_SIZE; i < FFT_SIZE; i++) fftReal[i] = 0.0f;
  memset(fftImag, 0, sizeof(fftImag));

  // Forward FFT — output: fftReal[k] + j*fftImag[k] for bin k
  fft.compute(FFTDirection::Forward);

  // Map bins to bands, convert to dB SPL with EMA smoothing (α ≈ 0.35)
  for (uint8_t b = 0; b < NUM_BANDS; b++) {
    float sum = 0.0f;
    for (uint16_t k = SPEC_START[b]; k <= SPEC_END[b]; k++) {
      sum += fftReal[k] * fftReal[k] + fftImag[k] * fftImag[k];
    }
    // Parseval normalization: factor 2 (one-sided), divide by N² to recover
    // mean-square ADC counts equivalent, then apply energyToDB calibration.
    float ms = sum * 2.0f / ((float)FFT_SIZE * (float)FFT_SIZE);
    float newDB = energyToDB(ms);
    specDB[b] = 0.65f * specDB[b] + 0.35f * newDB;
  }
}

// Render spectrum bars — each of the 13 columns is a frequency band,
// bars grow from row 7 (bottom) upward, 10 dB per row.
static void displaySpectrum() {
  memset(frameBuf, 0, sizeof(frameBuf));
  for (uint8_t col = 0; col < NUM_BANDS; col++) {
    uint8_t barH = (uint8_t)constrain(
      (specDB[col] - SPEC_DB_FLOOR) / SPEC_DB_RANGE * (float)ROWS,
      0.0f, (float)ROWS);
    for (uint8_t h = 0; h < barH; h++)
      frameBuf[ROWS - 1 - h][col] = DIGIT_BRIGHTNESS;
  }
}

// Convert mean-square ADC energy to dB SPL.
static float energyToDB(float ms) {
  float vRms = sqrtf(ms) / 16383.0f * 3.3f;
  if (vRms < 1e-10f) vRms = 1e-10f;
  return 20.0f * (logf(vRms) * 0.4342944819f) - MIC_SENSITIVITY_DBV + 94.0f + CAL_OFFSET;
}

// Accumulate mean-square energy into one of the two Leq accumulators.
static void accumulateLeq(uint8_t idx, float ms) {
  leqEnergySum[idx] += ms;
  ++leqBlockCount[idx];
  leqDB[idx] = energyToDB(leqEnergySum[idx] / leqBlockCount[idx]);
  if (millis() - leqStartMs[idx] >= LEQ_PERIOD_MS) {
    leqPeriodComplete[idx] = true;
    leqEnergySum[idx]  = 0.0f;
    leqBlockCount[idx] = 0;
    leqStartMs[idx]    = millis();
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
  // Leq modes: show weighting letter (A or C) with underline on row 6
  if (mode == MODE_LAEQ) {
    stampSmallGlyph(LETTER[MODE_A], 10, LETTER_ROW);
    for (uint8_t c = 10; c < 13 && c < COLS; c++) frameBuf[6][c] = LETTER_BRIGHTNESS;
  } else if (mode == MODE_LCEQ) {
    stampSmallGlyph(LETTER[MODE_C], 10, LETTER_ROW);
    for (uint8_t c = 10; c < 13 && c < COLS; c++) frameBuf[6][c] = LETTER_BRIGHTNESS;
  } else {
    stampSmallGlyph(LETTER[mode], 10, LETTER_ROW);
  }
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
  if (currentMode == MODE_A)     out &= ~(1u << NAV_LED_R);
  if (currentMode == MODE_C)     out &= ~(1u << NAV_LED_G);
  if (currentMode == MODE_LAEQ)  out &= ~(1u << NAV_LED_B);
  if (currentMode == MODE_LCEQ)     { out &= ~(1u << NAV_LED_B); out &= ~(1u << NAV_LED_G); } // cyan
  if (currentMode == MODE_SPECTRUM)  { out &= ~(1u << NAV_LED_R); out &= ~(1u << NAV_LED_G); out &= ~(1u << NAV_LED_B); } // white
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
  if (fell & (1u << NAV_CENTER)) {                               // reset both Leq accumulators
    uint32_t now = millis();
    for (uint8_t i = 0; i < 2; i++) {
      leqEnergySum[i]      = 0.0f;
      leqBlockCount[i]     = 0;
      leqStartMs[i]        = now;
      leqDB[i]             = 0.0f;
      leqPeriodComplete[i] = false;
    }
    lastDisplayed = -1;
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
  memset(specDB, 0, sizeof(specDB));
  setupNavSwitch();
  Monitor.println(F("mobile_spl live"));
}

void loop() {
  static uint8_t  blockCount = 0;
  static uint32_t procUs     = 0;   // last measured processing time (µs)
  static uint32_t procUsMax  = 0;   // worst-case since last report

  if (!Monitor) Monitor.begin();

  pollNavSwitch();
  collectBlock();

  uint32_t tProcStart = micros();   // ← start timing here (after ADC, before DSP)
  removeDC();

  if (currentMode == MODE_SPECTRUM) {
    // Spectrum mode: FFT-based frequency display, no SPL weighting needed
    computeSpectrum();
    if (++vuCount >= 5) {
      vuCount = 0;
      displaySpectrum();
      commitFrame();
    }
    if (++blockCount >= 25) {
      blockCount = 0;
      uint32_t blockUs = (uint32_t)BLOCK_SIZE * 1000000UL / SAMPLE_RATE;
      uint32_t loadPct = procUsMax * 100UL / blockUs;
      Monitor.print(F("spec"));
      for (uint8_t b = 0; b < NUM_BANDS; b++) {
        Monitor.print(F("  b")); Monitor.print(b); Monitor.print(F("="));
        Monitor.print(specDB[b], 1);
      }
      Monitor.print(F("  proc=")); Monitor.print(procUs);
      Monitor.print(F("us  max=")); Monitor.print(procUsMax);
      Monitor.print(F("us  load=")); Monitor.print(loadPct);
      Monitor.println(F("%"));
      procUsMax = 0;
    }
  } else {
    // SPL / Leq modes: biquad-weighted energy
    // Always compute both A-weighted and C-weighted energy
    applyWeighting(MODE_A);
    float energyA = computeEnergy();
    applyWeighting(MODE_C);
    float energyC = computeEnergy();

    // Feed both LEQ accumulators every block
    accumulateLeq(0, energyA);
    accumulateLeq(1, energyC);

    // Live dB for VU bar uses the weighting matching the current mode
    float energy = (currentMode == MODE_C || currentMode == MODE_LCEQ) ? energyC : energyA;
    float db     = energyToDB(energy);

    // VU meter + mic dots: refresh every 5 blocks (50 ms)
    if (++vuCount >= 5) {
      vuCount = 0;
      float val;
      bool  partial = false;
      if (currentMode == MODE_LAEQ) {
        val     = leqDB[0];
        partial = !leqPeriodComplete[0];
      } else if (currentMode == MODE_LCEQ) {
        val     = leqDB[1];
        partial = !leqPeriodComplete[1];
      } else {
        val = db;
      }
      int intVal = (int)(val + 0.5f);
      // Always redraw in Leq modes so partial→full brightness transition isn't missed
      bool forceRedraw = (currentMode == MODE_LAEQ || currentMode == MODE_LCEQ);
      if (intVal != lastDisplayed || forceRedraw) {
        lastDisplayed = intVal;
        displaySPL(intVal, currentMode, partial);
      }
      updateBottomRow(db);
      commitFrame();
    }

    if (++blockCount >= 25) {
      blockCount = 0;
      // Block period = BLOCK_SIZE / SAMPLE_RATE seconds
      uint32_t blockUs = (uint32_t)BLOCK_SIZE * 1000000UL / SAMPLE_RATE;
      uint32_t loadPct = procUsMax * 100UL / blockUs;
      Monitor.print(F("dBA=")); Monitor.print(energyToDB(energyA), 1);
      Monitor.print(F("  dBC=")); Monitor.print(energyToDB(energyC), 1);
      Monitor.print(F("  LAeq=")); Monitor.print(leqDB[0], 1);
      Monitor.print(F("  LCeq=")); Monitor.print(leqDB[1], 1);
      Monitor.print(F("  proc=")); Monitor.print(procUs);
      Monitor.print(F("us  max=")); Monitor.print(procUsMax);
      Monitor.print(F("us  load=")); Monitor.print(loadPct);
      Monitor.println(F("%"));
      procUsMax = 0;  // reset peak tracker
    }
  }

  // Capture processing time (excludes ADC collection)
  procUs = micros() - tProcStart;
  if (procUs > procUsMax) procUsMax = procUs;
}

