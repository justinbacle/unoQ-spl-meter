#include "Arduino_LED_Matrix.h"

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
