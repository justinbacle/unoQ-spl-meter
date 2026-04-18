# Mobile SPL Meter — Shield PCB Design Document

## 1. Overview

A shield PCB that sits on top of the Arduino Uno Q via standard pin headers,
containing all analog front-end and user-interface hardware:

- 2× MEMS microphones (SPH8878LR5H-1) with separate gain stages
- Dual op-amp circuit (one per mic, different gains)
- 5-way navigation switch (direct GPIO, no I2C)
- Power filtering and decoupling
- Status LED (optional)

Board dimensions: standard Arduino Uno shield (68.6 × 53.3 mm).

---

## 2. Design Goals

| Parameter            | Current (prototype)      | Target (shield)              |
|----------------------|--------------------------|------------------------------|
| Measurement range    | 33 – 103 dB SPL         | **33 – 133 dB SPL**         |
| Dynamic range        | 70 dB                   | **100 dB**                   |
| Weighting            | A and C                 | A and C (unchanged)          |
| ADC utilization      | Half-range (64× gain)   | **Full range (dual gain)**   |
| Nav switch           | I2C breakout (PCA9554)  | **Direct GPIO, on-board**    |
| Mic count            | 2 (same gain)           | 2 (different gains)          |

---

## 3. Analog Front-End Architecture

### 3.1 Triple-Gain Strategy

Three analog inputs, each with a different gain stage on a shared mic type:

| Channel | Arduino Pin | Gain | Gain (dB) | Range (dB SPL)  | Limited by       |
|---------|-------------|------|-----------|-----------------|------------------|
| HIGH    | A0          | 50×  | +34.0     | 33 – 105        | Mic noise / ADC  |
| MID     | A1          | 10×  | +20.0     | 44 – 119        | ADC / ADC        |
| LOW     | A2          |  2×  |  +6.0     | 58 – 133        | ADC / ADC        |

- **Combined range**: 33 – 133 dB SPL (**100 dB dynamic range**)
- **HIGH–MID overlap**: 44 – 105 dB (61 dB)
- **MID–LOW overlap**: 58 – 119 dB (61 dB)
- Software selects the highest-gain non-clipping channel for best SNR,
  with hysteresis to avoid rapid switching.

**Why 2× for LOW?** The mic AOP is 134 dB SPL. The ADC clips first when gain
exceeds 1.85× (crossover point). At 2× gain, the ADC ceiling is 133.3 dB —
just 0.7 dB below the mic AOP — giving near-maximum range while providing
6 dB better noise floor than unity gain, with a 58 dB floor that overlaps
well with the MID channel.

### 3.2 Range Calculations

```
Mic: SPH8878LR5H-1
  Sensitivity:      -44 dBV/Pa  (6.31 mV/Pa)
  Self-noise:       33 dB(A) SPL
  Max acoustic overload: 134 dB SPL  (AOP, confirmed from SparkFun hookup guide)
  Output at 134 dB: 892 mV peak  (→ needs gain ≤ 1.85× to avoid ADC clip)

ADC: STM32U585, 14-bit (16384 counts), 0–3.3 V
  LSB:          201.4 µV
  Max AC swing: ±1.65 V from VCC/2 bias

HIGH channel (gain = 50×):
  Max SPL = 94 + 20·log₁₀(1.65 / (50·√2·0.00631)) = 105.4 dB  (ADC clips)
  ADC floor = 94 + 20·log₁₀(201.4e-6 / (50·0.00631)) = 30.1 dB
  Effective floor = max(30.1, 33.0) = 33.0 dB  (mic-noise limited ✓)

MID channel (gain = 10×):
  Max SPL = 94 + 20·log₁₀(1.65 / (10·√2·0.00631)) = 119.3 dB  (ADC clips)
  ADC floor = 94 + 20·log₁₀(201.4e-6 / (10·0.00631)) = 44.1 dB

LOW channel (gain = 2×):
  Max SPL = 94 + 20·log₁₀(1.65 / (2·√2·0.00631)) = 133.3 dB  (ADC clips)
  Mic AOP = 134 dB SPL → ADC clips first at 133.3 dB ✓  (only 0.7 dB below AOP)
  ADC floor = 94 + 20·log₁₀(201.4e-6 / (2·0.00631)) = 58.1 dB
```

### 3.3 Op-Amp Selection

**Requirement**: GBW > gain × audio bandwidth = 50 × 20 kHz = 1 MHz minimum.
For low distortion and flat response, target 5–10× margin → 5–10 MHz GBW.

| Parameter       | OPA2344 (current) | **OPA2376** (recommended) | MCP6022 (alt)    |
|-----------------|--------------------|-----------------------|-------------------|
| Package         | SOIC-8             | SOIC-8 / MSOP-8       | SOIC-8            |
| Channels        | 2                  | 2                     | 2                 |
| GBW             | 1 MHz              | **5.5 MHz**           | 10 MHz            |
| Input noise     | 15 nV/√Hz         | **7.5 nV/√Hz**        | 8.7 nV/√Hz       |
| Rail-to-rail    | In + Out           | In + Out              | In + Out          |
| Supply          | 2.7–5.5 V         | 2.2–5.5 V            | 2.5–5.5 V        |
| Quiescent       | 750 µA             | 950 µA                | 1.1 mA            |
| Cost (approx)   | $1.50              | $1.80                 | $0.80             |

**Recommendation**: **OPA2376** — dual package handles two channels, 5.5 MHz GBW
gives 110× margin at 50× gain / 20 kHz, very low noise (7.5 nV/√Hz), and
rail-to-rail I/O on 3.3 V supply. **Use two OPA2376 packages** (4 op-amp channels
total, 3 used for the gain stages + 1 spare).

MCP6022 is a good budget alternative with even higher GBW.

### 3.4 Circuit Topology (per channel)

```
                     VCC (3.3V)
                      │
                     [R_bias]  100kΩ
                      │
 SPH8878LR5H-1       ├──────────── VCC/2 bias point
   OUT ──[C_in]──┤+  │
         100nF   │   [R_bias]  100kΩ
                 │ OPA2376    │
          ┌──┤-  │           GND
          │      │
         [Rg]   OUT ──────── to Arduino A0/A1/A2
          │
          ├──[Rf]──┘
          │
         GND

  HIGH channel (A0): Rf = 49.9kΩ, Rg = 1.02kΩ  → G = 1 + 49.9/1.02 = 49.9×
  MID  channel (A1): Rf = 9.1kΩ,  Rg = 1.02kΩ  → G = 1 + 9.1/1.02  =  9.9×
  LOW  channel (A2): Rf = 1.0kΩ,  Rg = 1.02kΩ  → G = 1 + 1.0/1.02  =  1.98×
```

**Note on mic sharing**: Each mic has its own dedicated op-amp channel.
The three mics are physically separate — no signal splitting required.
Place all three at the board edge, separated by ≥ 10 mm.

**Component notes**:
- C_in (100 nF): AC-coupling, with R_bias sets HPF at ~16 Hz (well below 20 Hz)
- R_bias pair (100kΩ + 100kΩ): Sets VCC/2 operating point for non-inverting input
- Rf, Rg: 1% tolerance metal-film for gain accuracy
- Add 100 nF + 10 µF decoupling at op-amp VCC pin
- Add 100 nF bypass at each mic VCC pin

### 3.5 Mic Mounting

SPH8878LR5H-1 is a **bottom-port** MEMS microphone:
- Mount on the **top side** of the shield PCB
- PCB needs an **acoustic port hole** (Ø 0.5–1.0 mm) directly under the sound inlet
- Recommended hole diameter: **0.7 mm** (per Knowles/InvenSense app notes)
- Place all three mics at the board edge for best acoustic access
- Keep analog traces short — route mic output directly to nearby op-amp input
- Separate mics by at least 10 mm to avoid acoustic coupling
- Group each mic with its op-amp channel for shortest trace routing

---

## 4. Navigation Switch

### 4.1 Switch Selection

Replace the external Qwiic PCA9554 breakout with an on-board 5-way tactile switch.

**Recommended part**: ALPS SKQUCAA010 or equivalent 5-way tact switch
- Through-hole mounting (sturdy for finger operation)
- 5 directions: Up, Down, Left, Right, Center-push
- Compact footprint (~10×10 mm)

Alternative (SMD): Würth 434133025816 or C&K KSJ series

### 4.2 GPIO Wiring

Wire directly to Arduino digital pins (no I2C needed):

| Direction | Arduino Pin | Shield Header |
|-----------|-------------|---------------|
| UP        | D2          | Digital 2     |
| DOWN      | D3          | Digital 3     |
| LEFT      | D4          | Digital 4     |
| RIGHT     | D5          | Digital 5     |
| CENTER    | D6          | Digital 6     |

Each switch leg connects between the GPIO pin and GND.
Use internal pull-ups (INPUT_PULLUP) in software — no external resistors needed.

### 4.3 Software Changes Required

Replace PCA9554 I2C reads with direct `digitalRead()`:
```cpp
#define NAV_UP     2
#define NAV_DOWN   3
#define NAV_LEFT   4
#define NAV_RIGHT  5
#define NAV_CENTER 6

// In setup():
for (int p = NAV_UP; p <= NAV_CENTER; p++)
  pinMode(p, INPUT_PULLUP);

// In pollNav():
bool up     = !digitalRead(NAV_UP);
bool down   = !digitalRead(NAV_DOWN);
// ... etc
```

---

## 5. Power

- Shield powered entirely from Arduino Uno Q 3.3V pin
- Total current budget:
  - 3× SPH8878LR5H-1: ~250 µA each = 0.75 mA
  - 2× OPA2376 dual: ~950 µA each = 1.9 mA
  - **Total: ~2.7 mA** (negligible load on Uno Q 3.3V regulator)
- Decoupling:
  - 10 µF electrolytic + 100 nF ceramic at board power entry
  - 100 nF ceramic at each op-amp VCC pin (close to pin)
  - 100 nF ceramic at each mic VCC pin

---

## 6. Pin Header Mapping

Standard Arduino Uno header layout. Pins used by the shield:

| Pin     | Function                    |
|---------|-----------------------------|
| A0      | Mic 1 (HIGH gain, 50×)      |
| A1      | Mic 2 (MID gain, 10×)       |
| A2      | Mic 3 (LOW gain, 2×)        |
| D2      | Nav UP                      |
| D3      | Nav DOWN                    |
| D4      | Nav LEFT                    |
| D5      | Nav RIGHT                   |
| D6      | Nav CENTER                  |
| 3.3V    | Power supply                |
| GND     | Ground (use multiple pins)  |

**Unused/available**: D7–D13, A3–A5, 5V, VIN, SCL/SDA, IOREF, RESET

---

## 7. Bill of Materials (BOM)

| Qty | Part                    | Value / Package       | Reference          |
|-----|-------------------------|-----------------------|--------------------|
| 3   | SPH8878LR5H-1          | MEMS mic, bottom-port | MIC1, MIC2, MIC3   |
| 2   | OPA2376 (or MCP6022)   | Dual op-amp, SOIC-8   | U1, U2             |
| 1   | Resistor                | 49.9kΩ 1%, 0402/0603  | R_F1 (HIGH ch)     |
| 1   | Resistor                | 9.1kΩ 1%, 0402/0603   | R_F2 (MID ch)      |
| 1   | Resistor                | 1.0kΩ 1%, 0402/0603   | R_F3 (LOW ch)      |
| 3   | Resistor                | 1.02kΩ 1%, 0402/0603  | R_G1, R_G2, R_G3   |
| 6   | Resistor                | 100kΩ 5%, 0402/0603   | R_BIAS×6           |
| 3   | Capacitor               | 100nF ceramic, 0402   | C_IN1, C_IN2, C_IN3|
| 5   | Capacitor               | 100nF ceramic, 0402   | C_BYP×5            |
| 1   | Capacitor               | 10µF ceramic, 0805    | C_BULK             |
| 1   | 5-way nav switch        | ALPS SKQUCAA010       | SW1                |
| 1   | Pin header (1×10)       | 2.54mm, female        | J1 (digital)       |
| 1   | Pin header (1×8)        | 2.54mm, female        | J2 (digital)       |
| 1   | Pin header (1×6)        | 2.54mm, female        | J3 (analog)        |
| 1   | Pin header (1×8)        | 2.54mm, female        | J4 (power)         |

**Total unique parts**: ~10 unique components + headers
**Total component count**: ~30 placements

---

## 8. Software Changes Summary

Firmware modifications needed for triple-gain hardware:

1. **Triple-channel ADC reading**: Read A0 (50×), A1 (10×), and A2 (2×) each block
2. **Channel selection logic**: Use highest-gain non-clipping channel for best SNR
   - Use HIGH when < ~100 dB, MID when < ~117 dB, LOW for > ~117 dB
   - Add 2–3 dB hysteresis to prevent rapid channel switching
3. **Gain-aware dB calculation**: Different sensitivity per channel
   - HIGH (50×): -44 + 34.0 = -10.0 dBV/Pa
   - MID  (10×): -44 + 20.0 = -24.0 dBV/Pa
   - LOW   (2×): -44 +  6.0 = -38.0 dBV/Pa
4. **Nav switch driver**: Replace PCA9554 I2C with direct GPIO digitalRead
5. **Weighting filters**: Run on selected channel (or all three independently)
6. **Crossover blending** (optional): In overlap regions, weighted average for smooth transitions

---

## 9. PCB Layout Guidelines

- **2-layer board** is sufficient (top: components, bottom: ground plane)
- Keep mic-to-opamp traces **< 10 mm**, avoid routing near digital signals
- Ground plane under analog section (mics + op-amp) should be unbroken
- Acoustic holes for mics: Ø 0.7 mm, keep area around hole clear of copper
- Place nav switch near board edge for ergonomic access
- Place decoupling caps within 2 mm of their IC power pins
- Shield outline matches Arduino Uno (68.6 × 53.3 mm) with mounting holes
  at standard Arduino positions

---

## 10. Revision History

| Rev | Date       | Notes                                 |
|-----|------------|---------------------------------------|
| 0.1 | 2026-04-18 | Initial design, dual-gain concept     |
| 0.2 | 2026-04-18 | Corrected AOP to 134 dB SPL; LOW channel ceiling 133 dB (ADC-limited); 100 dB dynamic range |
