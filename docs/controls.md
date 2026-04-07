# Mobile SPL Meter — Controls

## SparkFun Qwiic Navigation Switch (PCA9554, I2C 0x20)

Connected to the UNO Q Qwiic connector (I²C bus `Wire1`). Detected at startup — if absent, the app runs normally with no input controls.

| Direction | Action |
|-----------|--------|
| **Right** | Next weighting mode: A → C → Leq → A |
| **Left**  | Previous weighting mode: A → Leq → C → A |
| **Up**    | Next mic input: Mic 0 → Mic 1 → Average → Mic 0 |
| **Down**  | Previous mic input: Mic 0 → Average → Mic 1 → Mic 0 |
| **Center** | Reset Leq accumulator (restarts 60 s integration window) |

### RGB Status LED

| Colour | Mode |
|--------|------|
| Red    | A-weighting |
| Green  | C-weighting |
| Blue   | Leq |

---

## LED Matrix — Bottom Row Indicator

The bottom row (row 7) updates every 50 ms independently of the digit display.

```
cols:  0  1  2  3  4  5  6  7  8  9  10  11  12
       [────────── VU bar ──────────]  [ ]  [M0] [M1]
```

| Segment | Description |
|---------|-------------|
| **Cols 0–9** | VU bar — each segment represents 7 dB. Full scale = 10 segments at 110 dBSPL, empty = 40 dBSPL |
| **Col 10** | Gap (always off) |
| **Col 11** | Mic 0 — full brightness when selected, dim when available |
| **Col 12** | Mic 1 — full brightness when selected, dim when available |

Both dots are fully lit when **Average** mode is active (`activeMic = NUM_MICS`).

---

## Weighting Modes

| Letter | Mode | Standard |
|--------|------|---------|
| **A** | A-weighting | IEC 61672 — human hearing loudness |
| **C** | C-weighting | IEC 61672 — peak / low-freq events |
| **L** | Leq | Time-averaged equivalent level (60 s window) |

---

## Build Configuration (`sketch.ino`)

| `#define` | Default | Description |
|-----------|---------|-------------|
| `DEMO_MODE` | `0` | `1` = synthetic dB sweep, `0` = live mic |
| `NUM_MICS` | `2` | Number of microphone ADC inputs |
| `MIC_SENSITIVITY_DBV` | `-42.0` | Mic sensitivity in dBV/Pa |
| `CAL_OFFSET` | `0.0` | Per-unit calibration trim (dB) |
| `LEQ_PERIOD` | `6000` | Leq window in 10 ms blocks (= 60 s) |
| `DIGIT_BRIGHTNESS` | `7` | LED brightness for digits (0–7) |
| `LETTER_BRIGHTNESS` | `4` | LED brightness for mode letter (0–7) |
| `NAV_ADDR` | `0x20` | PCA9554 I2C address |
