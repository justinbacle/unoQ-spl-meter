#include "Arduino_LED_Matrix.h"

// ── Microphone configuration ────────────────────────────────
static const uint8_t NUM_MICS            = 2;
static const uint8_t MIC_PINS[NUM_MICS]  = { A0, A1 };
static const float   MIC_SENSITIVITY_DBV = -42.0f;  // SPH8878: -42 dBV/Pa
static const float   CAL_OFFSET          =   0.0f;  // per-unit calibration (dB)

// Which input to measure: 0..NUM_MICS-1 = single mic, NUM_MICS = average all
static uint8_t activeMic = 0;

// ── Sampling configuration ──────────────────────────────────
static const uint32_t SAMPLE_RATE = 48000;            // Hz
static const uint16_t BLOCK_SIZE  = 480;              // samples per block (10 ms)
static const int16_t  ADC_MID     = 8192;             // 14-bit midpoint (2^13)

// One block of raw ADC samples, filled by collectBlock()
static int16_t sampleBuf[BLOCK_SIZE];

// Collect one block of BLOCK_SIZE samples at SAMPLE_RATE using spin-wait timing.
// If activeMic < NUM_MICS: reads that single mic.
// If activeMic == NUM_MICS: reads all mics and averages each sample.
static void collectBlock() {
  const uint32_t tickUs = 1000000UL / SAMPLE_RATE;  // ~20 µs at 48 kHz
  uint32_t tNext = micros();
  for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
    if (activeMic < NUM_MICS) {
      sampleBuf[i] = (int16_t)analogRead(MIC_PINS[activeMic]) - ADC_MID;
    } else {
      int32_t sum = 0;
      for (uint8_t m = 0; m < NUM_MICS; m++)
        sum += analogRead(MIC_PINS[m]);
      sampleBuf[i] = (int16_t)(sum / NUM_MICS) - ADC_MID;
    }
    tNext += tickUs;
    while ((int32_t)(micros() - tNext) < 0) { /* spin */ }
  }
}

// ── DC offset removal ───────────────────────────────────────
// EMA high-pass: corner ~10 Hz at 48 kHz → alpha = 1 - 2π·10/48000 ≈ 0.99869
static const float DC_ALPHA = 0.99869f;
static float dcState = 0.0f;  // reset when activeMic changes

// Remove DC from sampleBuf in-place; updates dcState across blocks.
static void removeDC() {
  for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
    dcState = DC_ALPHA * dcState + (1.0f - DC_ALPHA) * sampleBuf[i];
    sampleBuf[i] = (int16_t)(sampleBuf[i] - dcState);
  }
}

// ── Display configuration ───────────────────────────────────
static const bool    DISPLAY_FLIPPED = true;
static const uint8_t ROWS     = 8;
static const uint8_t COLS     = 13;
static const uint8_t FONT_ROW   = 1;  // digits start at row 1 (5px tall, baseline row 5)
static const uint8_t LETTER_ROW = 2;  // letters start at row 2 (4px tall, baseline row 5)

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

// Mode letters A, C, L — 3x4 font, cols 10-12, rows 2-5 (same baseline as digits).
static const uint8_t LETTER[3][4] = {
  {0b010,0b101,0b111,0b101}, // A
  {0b011,0b100,0b100,0b011}, // C
  {0b100,0b100,0b100,0b111}, // L
};

enum WeightMode { MODE_A=0, MODE_C=1, MODE_LEQ=2 };
static ArduinoLEDMatrix matrix;
static byte frameBuf[ROWS][COLS];

static void stampGlyph(const uint8_t glyph[5], uint8_t col, uint8_t row) {
  for (uint8_t r=0; r<5 && (row+r)<ROWS; r++)
    for (uint8_t c=0; c<3 && (col+c)<COLS; c++)
      frameBuf[row+r][col+c] = (glyph[r]>>(2u-c))&0x01u;
}

// 4-row variant for the mode letters
static void stampSmallGlyph(const uint8_t glyph[4], uint8_t col, uint8_t row) {
  for (uint8_t r=0; r<4 && (row+r)<ROWS; r++)
    for (uint8_t c=0; c<3 && (col+c)<COLS; c++)
      frameBuf[row+r][col+c] = (glyph[r]>>(2u-c))&0x01u;
}

// Narrow single-column 1 for hundreds (saves 2 cols vs full 3-wide glyph)
static void stampNarrowOne(uint8_t col, uint8_t row) {
  for (uint8_t r=0; r<5 && (row+r)<ROWS; r++)
    frameBuf[row+r][col] = 1;
}

// Rotate 180 in-place by swapping pixel i with pixel (total-1-i)
static void rotateFrame180() {
  const uint8_t total = ROWS * COLS;
  for (uint8_t i=0; i<total/2; i++) {
    uint8_t j   = total-1u-i;
    byte    tmp = frameBuf[i/COLS][i%COLS];
    frameBuf[i/COLS][i%COLS] = frameBuf[j/COLS][j%COLS];
    frameBuf[j/COLS][j%COLS] = tmp;
  }
}

void displaySPL(int db, WeightMode mode, bool flipped = DISPLAY_FLIPPED) {
  db = constrain(db, 0, 130);
  memset(frameBuf, 0, sizeof(frameBuf));
  if (db >= 100) stampNarrowOne(0, FONT_ROW);
  stampGlyph(DIGIT[(db%100)/10], 2, FONT_ROW);
  stampGlyph(DIGIT[db%10],       6, FONT_ROW);
  // Col 9 = gap; cols 10-12 = 3-row mode letter at bottom of display.
  stampSmallGlyph(LETTER[mode], 10, LETTER_ROW);
  if (flipped) rotateFrame180();
  uint32_t packed[4] = {0,0,0,0};
  for (uint8_t r=0; r<ROWS; r++)
    for (uint8_t c=0; c<COLS; c++)
      if (frameBuf[r][c]) {
        uint8_t bit = r*COLS+c;
        packed[bit>>5] |= 1u<<(31u-(bit&0x1Fu));
      }
  matrix.loadFrame(packed);
}

static int  simDB   = 35;
static bool sweepUp = true;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis()<3000) {}
  matrix.begin();
  analogReadResolution(14);  // STM32U585 supports 14-bit ADC (0-16383)
  Serial.println(F("mobile_spl Phase 1 - 35-130dB sweep"));
}

void loop() {
  WeightMode mode = static_cast<WeightMode>((simDB/10)%3);
  displaySPL(simDB, mode);
  Serial.print(F("sim dB = ")); Serial.print(simDB);
  Serial.print(F("  mode = ")); Serial.println(mode);
  sweepUp ? simDB++ : simDB--;
  if (simDB>=130) sweepUp=false;
  if (simDB<=35)  sweepUp=true;
  delay(150);
}
