# Mobile SPL Meter

Dual-channel precision sound level meter running on the **Arduino Uno Q**
(STM32U585 MCU + Qualcomm QRB2210 MPU).

Displays live dBA / dBC / LAeq / LCeq readings and an FFT spectrum on the
integrated 8×13 LED matrix. Modes are cycled with the Qwiic nav switch.

---

## Requirements

- **Hardware**: Arduino Uno Q, 2× SparkFun SPH8878 MEMS mic breakouts,
  SparkFun Qwiic 5-way nav switch
- **Host tools**: `arduino-app-cli`, `arduino-cli`, `ssh`, `scp`
- **Board package**: `arduino:zephyr` (includes the `unoq` target)
- The Uno Q must be **on the same network** as your development machine and
  accessible via SSH (default: `arduino@192.168.1.146`)

> Update the IP address in `.vscode/settings.json` if yours differs.

---

## Project structure

```text
mobile_spl/
├── app.yaml              # Uno Q app manifest (MPU side)
├── python/
│   └── main.py           # Python entry point (MPU — minimal stub)
├── sketch/
│   ├── sketch.ino        # MCU firmware (audio DSP, display, nav switch)
│   └── sketch.yaml       # Board target + library pins
├── hardware/
│   ├── shield_design.md  # Triple-gain analog front-end PCB design
│   └── build_guide.md    # Perfboard hand-build guide (BOM, schematics, steps)
└── README.md
```

---

## Deploy and run

### 1. Copy files to the board

```bash
scp sketch/sketch.ino arduino@192.168.1.146:~/ArduinoApps/mobile_spl/sketch/
scp sketch/sketch.yaml arduino@192.168.1.146:~/ArduinoApps/mobile_spl/sketch/
scp python/main.py     arduino@192.168.1.146:~/ArduinoApps/mobile_spl/python/
```

### 2. Restart the app

```bash
ssh arduino@192.168.1.146 "arduino-app-cli app stop ~/ArduinoApps/mobile_spl; \
                            arduino-app-cli app start ~/ArduinoApps/mobile_spl"
```

This compiles the sketch, flashes it over SWD, and starts the Python container.
The full cycle takes ~30–60 seconds.

### 3. Watch live MCU output

```bash
ssh arduino@192.168.1.146 "arduino-app-cli monitor"
```

Sample output:

```text
dBA=49.8  dBC=54.1  LAeq=51.0  LCeq=57.5
dBA=50.3  dBC=55.9  LAeq=51.0  LCeq=57.4
```

### 4. Watch Python / MPU logs

```bash
ssh arduino@192.168.1.146 "arduino-app-cli app logs ~/ArduinoApps/mobile_spl --tail 20"
```

---

## One-liner deploy + verify

```bash
scp sketch/sketch.ino arduino@192.168.1.146:~/ArduinoApps/mobile_spl/sketch/ && \
ssh arduino@192.168.1.146 \
  "arduino-app-cli app stop ~/ArduinoApps/mobile_spl; \
   arduino-app-cli app start ~/ArduinoApps/mobile_spl && \
   timeout 6 arduino-app-cli monitor"
```

---

## Display modes

Cycle through modes by pressing the nav switch **left / right**.

| Mode | Display | Description |
| :--- | :--- | :--- |
| `A` | `49.8 A` | Live dBA (fast, 125 ms RMS) |
| `C` | `54.1 C` | Live dBC (fast, 125 ms RMS) |
| `LA` | `51.0 LA` | LAeq — 60 s equivalent level |
| `LC` | `57.5 LC` | LCeq — 60 s equivalent level |
| `SP` | bar graph | FFT spectrum (512-point, 48 kHz) |

A brief flash of all pixels indicates ADC clipping.

---

## Key configuration (`sketch/sketch.ino`)

| `#define` | Default | Description |
| :--- | :--- | :--- |
| `SAMPLE_RATE` | `48000` | ADC sample rate (Hz) |
| `BLOCK_SIZE` | `480 * 2` | Samples per processing block (20 ms) |
| `MIC_SENSITIVITY_DBV` | `-7.9` | Mic + preamp sensitivity (dBV/Pa) |
| `CAL_OFFSET` | `0.0` | Per-unit calibration trim (dB) |
| `NUM_MICS` | `2` | Number of ADC channels sampled |

---

## Hardware

See [hardware/shield_design.md](hardware/shield_design.md) for the full
triple-gain analog front-end design, and
[hardware/build_guide.md](hardware/build_guide.md) for the perfboard
hand-build guide.
