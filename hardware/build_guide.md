# Mobile SPL Shield — Manual Build Guide

Build this on perfboard or a breadboard. No PCB software needed.

---

## Bill of Materials

| Qty | Part                  | Value            | Notes                            |
|-----|-----------------------|------------------|----------------------------------|
| 3   | MEMS Microphone       | SPH8878LR5H-1    | SparkFun BOB-19389 breakout ×3   |
| 2   | Dual Op-Amp IC        | OPA2376AIDR      | SOIC-8; or use MCP6022 (cheaper) |
| 1   | Resistor              | 49.9 kΩ 1%       | R_F1 — HIGH channel feedback     |
| 1   | Resistor              | 9.1 kΩ 1%        | R_F2 — MID channel feedback      |
| 1   | Resistor              | 1.0 kΩ 1%        | R_F3 — LOW channel feedback      |
| 3   | Resistor              | 1.0 kΩ 1%        | R_G1–R_G3 — gain resistors       |
| 6   | Resistor              | 100 kΩ           | R_BIAS — VCC/2 dividers (2 per mic)|
| 3   | Capacitor             | 100 nF ceramic   | C_IN1–C_IN3 — AC coupling        |
| 5   | Capacitor             | 100 nF ceramic   | Decoupling (1 per op-amp VCC, 1 per mic VCC, 1 spare)|
| 1   | Capacitor             | 10 µF            | Bulk decoupling at power entry   |
| 1   | 5-way nav switch      | ALPS SKQUCAA010  | Or any 5-direction tact switch   |

**Using SparkFun mic breakouts** (BOB-19389) simplifies the build — each breakout
already has the mic soldered. You only need to connect VCC, GND, and AUD (signal out).

---

## Gain Stages — What They Do

Three microphones feed three separate amplifier channels, each with a different gain.
This gives a 100 dB combined dynamic range:

| Channel | Pin | Gain  | R_F      | R_G    | SPL Range    |
|---------|-----|-------|----------|--------|--------------|
| HIGH    | A0  | 50×   | 49.9 kΩ  | 1.0 kΩ | 33 – 105 dB  |
| MID     | A1  | 10×   | 9.1 kΩ   | 1.0 kΩ | 44 – 119 dB  |
| LOW     | A2  | 2×    | 1.0 kΩ   | 1.0 kΩ | 58 – 133 dB  |

Gain formula: G = 1 + R_F / R_G

---

## Schematic — One Amplifier Channel (repeat × 3)

```
3.3V ──────┬─────────────────────────────────┐
           │                                 │
         [100k]  R_BIAS_A                  [100nF]  C_BYP  (close to U+ pin)
           │                                 │
           ├── BIAS ──────────────────── U_VCC (pin 8)
           │
         [100k]  R_BIAS_B
           │
          GND


  SPH8878 breakout                 OPA2376 (one channel)
  ┌──────────────┐
  │  VCC ─── 3.3V│            ┌─────────────────────┐
  │  GND ─── GND │            │                     │
  │  AUD ────────┼──[C_IN]────┤ IN+ (pin 3)         │
  │  (signal out)│  100nF     │                     │
  └──────────────┘       ┌────┤ IN- (pin 2)    OUT (pin 1) ────── to Arduino A0/A1/A2
                         │    │                     │
                        [R_G] └─────────────────────┘
                         │              │
                        GND           [R_F]
                                        │
                                       (connect back to IN-)

```

### Non-Inverting Amplifier with DC Bias — Redrawn Clearly

```
                        3.3V
                         │
                       [100k]  ← R_BIAS_A
                         │
                         ├──────────── to IN+ (op-amp non-inverting input)
                         │
                       [100k]  ← R_BIAS_B
                         │
                        GND

Mic AUD ──[100nF]──────────────── IN+ (sets DC = 1.65V = VCC/2)

                           ┌──────────[R_F]──────┐
                           │                     │
                IN- ───────┘              OUT ───┴──── to Arduino analog pin
                           │
                          [R_G]
                           │
                          GND
```

### Op-Amp Pin Reference (OPA2376 SOIC-8)

OPA2376 is a **dual** op-amp — two independent amplifiers in one 8-pin package.
Use one package for channels HIGH + MID, second package for channel LOW (+ one spare).

```
        OPA2376 SOIC-8
        ┌────────────┐
  OUT_A ┤ 1        8 ├ VCC  ─── 3.3V
  IN-_A ┤ 2        7 ├ OUT_B
  IN+_A ┤ 3        6 ├ IN-_B
   GND  ┤ 4        5 ├ IN+_B
        └────────────┘
```

**Package A (U1)**: Channel HIGH on amp A (pins 1–3), Channel MID on amp B (pins 5–7)
**Package B (U2)**: Channel LOW on amp A (pins 1–3), spare amp B unused

---

## Full Schematic — All Three Channels

```
3.3V ────┬──────────────────────────────────────────────────────────┐
         │                                                          │
       [10µF]  (bulk, at power entry)                            (to all VCC pins)
         │
        GND


──── CHANNEL HIGH (A0, gain 50×) ────────────────────────────────────

3.3V ─[100k]─┬─[100k]─ GND          (voltage divider sets BIAS = 1.65V)
              │
MIC1_AUD ─[100nF]─┤ U1 IN+_A (pin 3)
                   │
                   ├─[1.0k R_G]─ GND
                   │
        U1 IN-_A (pin 2) ──┬──────────────────── U1 OUT_A (pin 1) ─── A0
                            └──[49.9k R_F]────────────────────────┘


──── CHANNEL MID (A1, gain 10×) ─────────────────────────────────────

3.3V ─[100k]─┬─[100k]─ GND
              │
MIC2_AUD ─[100nF]─┤ U1 IN+_B (pin 5)
                   │
                   ├─[1.0k R_G]─ GND
                   │
        U1 IN-_B (pin 6) ──┬──────────────────── U1 OUT_B (pin 7) ─── A1
                            └──[9.1k R_F]─────────────────────────┘


──── CHANNEL LOW (A2, gain 2×) ──────────────────────────────────────

3.3V ─[100k]─┬─[100k]─ GND
              │
MIC3_AUD ─[100nF]─┤ U2 IN+_A (pin 3)
                   │
                   ├─[1.0k R_G]─ GND
                   │
        U2 IN-_A (pin 2) ──┬──────────────────── U2 OUT_A (pin 1) ─── A2
                            └──[1.0k R_F]─────────────────────────┘


──── DECOUPLING ──────────────────────────────────────────────────────

U1 VCC (pin 8) ─── 3.3V, with [100nF] cap as close to pin as possible → GND
U2 VCC (pin 8) ─── 3.3V, with [100nF] cap as close to pin as possible → GND
U1 GND (pin 4) ─── GND
U2 GND (pin 4) ─── GND

Each SparkFun mic breakout: place [100nF] on VCC pin → GND on the breakout itself
```

---

## Navigation Switch Schematic

Use any 5-way tact switch (ALPS SKQUCAA010 or similar). Each direction is
an independent normally-open contact.

```
          ALPS SKQUCAA010 (top view)
               ┌─────┐
    UP    ────-┤  ↑  ├─── GND
    DOWN  ────-┤  ↓  ├─── GND
    LEFT  ────-┤  ←  ├─── GND
    RIGHT ────-┤  →  ├─── GND
    CENTER────-┤  ·  ├─── GND
               └─────┘

Arduino pins:
  UP     → D2 (INPUT_PULLUP)
  DOWN   → D3 (INPUT_PULLUP)
  LEFT   → D4 (INPUT_PULLUP)
  RIGHT  → D5 (INPUT_PULLUP)
  CENTER → D6 (INPUT_PULLUP)

No external resistors needed — Arduino internal pull-ups are used.
Button press pulls the pin to GND → reads LOW → active.
```

---

## Step-by-Step Build (Perfboard)

### 1 — Power rail
- Run a 3.3V rail and a GND rail along the top and bottom of the board.
- Solder the 10 µF bulk cap between 3.3V and GND at the power entry point.

### 2 — Bias dividers (×3, one per channel)
For each channel, solder two 100 kΩ resistors in series between 3.3V and GND.
The midpoint of each pair is the BIAS point for that channel's IN+ input.

### 3 — Op-amp ICs
- Solder U1 (OPA2376) and U2 (OPA2376) on SOIC-to-DIP adapters or directly
  to the perfboard with short leads.
- Connect pin 8 (VCC) to 3.3V and pin 4 (GND) to GND on each package.
- Solder a 100 nF cap from each VCC pin to GND immediately adjacent.

### 4 — Gain networks (per channel)
For each of the three channels:
1. Connect IN- to GND through R_G (1.0 kΩ).
2. Connect a wire from IN- back to OUT.
3. In that feedback wire, insert R_F (49.9k / 9.1k / 1.0k depending on channel).

### 5 — AC coupling and mic input (per channel)
1. Solder the 100 nF cap in series between the SparkFun AUD pad and IN+.
2. Connect the BIAS midpoint to IN+ (same node as the cap output).
3. Connect the mic breakout VCC to 3.3V and GND to GND.
   Add a 100 nF cap on the breakout's VCC pin close to the mic.

### 6 — Output wires
Run wires from each OUT pin to the Arduino Uno Q headers:
- U1 OUT_A → A0
- U1 OUT_B → A1
- U2 OUT_A → A2

### 7 — Nav switch
Solder the 5-way switch near a board edge. Connect each direction output to
the corresponding Arduino digital pin (D2–D6). Connect all switch commons to GND.

### 8 — Arduino connections summary

| Shield point     | Arduino pin | Wire colour (suggestion) |
|------------------|-------------|--------------------------|
| HIGH channel out | A0          | Red                      |
| MID channel out  | A1          | Orange                   |
| LOW channel out  | A2          | Yellow                   |
| Nav UP           | D2          | White                    |
| Nav DOWN         | D3          | White                    |
| Nav LEFT         | D4          | White                    |
| Nav RIGHT        | D5          | White                    |
| Nav CENTER       | D6          | White                    |
| 3.3V             | 3.3V pin    | Red                      |
| GND              | GND pins    | Black (use ≥2 GND pins)  |

---

## Firmware Changes Needed for This Shield

The current sketch (`sketch.ino`) is written for the prototype with:
- 2 mics (A0, A1) at the same gain
- Nav switch via I2C PCA9554

For the shield you'll need to update:

1. **`NUM_MICS` and channel gains** — extend to 3 channels with per-channel sensitivity
2. **`MIC_SENSITIVITY_DBV` per channel**:
   - HIGH (A0): `-10.0f`  (-44 dBV/Pa + 34 dB gain)
   - MID  (A1): `-24.0f`  (-44 dBV/Pa + 20 dB gain)
   - LOW  (A2): `-38.0f`  (-44 dBV/Pa + 6 dB gain)
3. **Channel selection logic** — pick the highest-gain non-clipping channel each block
4. **Nav switch driver** — replace PCA9554 I2C code with `digitalRead(D2..D6)`

These changes are **not yet in the sketch** — they are the next firmware task.

---

## Test Points

Before connecting to the Arduino:
1. Power the board from 3.3V and measure the BIAS point on each channel: should be ~1.65V.
2. Without audio input, measure each OUT: should be ~1.65V (DC-coupled op-amp at mid-rail).
3. Apply audio (speak into each mic): verify the output swings around 1.65V.

The Arduino ADC reads 0–3.3V, midpoint 1.65V → ADC_MID ≈ 8192 (14-bit). DC bias at
1.65V means the resting ADC value is near 8192, and audio modulates around that — which
matches the existing `ADC_MID 8192` definition in the firmware.
