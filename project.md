---

# 🔊 Project Master Spec: Dual-Channel Precision Sound Meter
**Platform:** Arduino Uno Q (STM32U585 + Qualcomm QRB2210)  
**Input:** 2x SparkFun SPH8878 MEMS Microphones  
**Control:** SparkFun Qwiic Navigation Switch (5-way Joystick)  
**Output:** Integrated 8x13 LED Matrix  

---

## 🛠 1. Hardware Configuration

### A. Wiring Table
| Component | Pin / Port | Purpose | Logic Level |
| :--- | :--- | :--- | :--- |
| **Mic 1 (Left)** | **A0** | Analog Audio Input | 3.3V |
| **Mic 2 (Right)** | **A1** | Analog Audio Input | 3.3V |
| **Nav Switch** | **Qwiic Port** | Mode/Input Navigation | I2C (Address 0x48) |
| **Common VCC** | **3.3V** | Power for both MEMS mics | **Do Not Use 5V** |
| **Common GND** | **GND** | Shared Ground | — |

### B. Compact Build Strategy
* **The "Ears":** Mount the two SPH8878 mics on the top corners of the Uno Q. Use a tiny piece of acoustic foam under the sensors to dampen board vibrations.
* **The Controller:** Plug the Nav Switch into the Qwiic port. Secure it to the right side of the board for thumb-access.
* **Power:** Use a high-current USB-C power bank (2A+ output) to handle the dual-processor load of the Uno Q.

---

## 📐 2. Measurement Capabilities
* **Weighting Filters:** * **dBA:** For environmental/human hearing noise.
    * **dBB:** Intermediate weighting.
    * **dBC:** For high-volume/bass-heavy machinery or music.
* **Metrics:**
    * **SPL (Fast):** 125ms RMS window.
    * **$L_{Aeq}$:** Integrated energy average (Leq) with 1-minute rolling buffer.
* **Range:** ~35dB to 130dB SPL.

---

## 🕹️ 3. User Interface Logic (Nav Switch + Matrix)

### Navigation Map
* **Joystick UP / DOWN:** Cycle Weighting Mode (A → B → C).
* **Joystick LEFT / RIGHT:** Toggle Mic Input (Mic 1 → Mic 2 → Stereo Average).
* **Center Click:** Toggle between **Live SPL** and **$L_{Aeq}$**.
* **Long Press (Center):** Reset the $L_{Aeq}$ accumulation buffer.

### LED Matrix Display (8x13)
* **Value (Left 8 Rows):** Large 3x5 font showing the dB value.
* **Mode (Right 3 Rows):** Small character indicator:
    * `A, B, or C` (Weighting)
    * `L` (When displaying $L_{Aeq}$)

---

## 💻 4. Firmware Logic Flow
1.  **Initialize:** Start I2C for Nav Switch and set ADC resolution to **14-bit** (STM32 default).
2.  **Sampling:** Use a timer interrupt to sample **A0 and A1 at 48kHz**.
3.  **DSP Pipeline:**
    * Remove DC Offset (subtract 1.65V equivalent).
    * Apply Digital IIR Filter (A/B/C coefficients).
    * Calculate RMS: $V_{rms} = \sqrt{\frac{1}{n} \sum V^2}$.
4.  **Conversion:** Convert $V_{rms}$ to $dB$ using the `-42 dBV/Pa` sensitivity of the SPH8878.
5.  **Display:** Refresh the LED Matrix every 250ms to ensure readability.

---

## ⚖️ 5. Calibration Procedure
* **Setup:** Use your professional measurement microphone as the "Gold Standard."
* **Step 1:** Play Pink Noise through a flat-response speaker.
* **Step 2:** Match the Pro Mic and the Uno Q distances (approx 0.5m).
* **Step 3:** Record the difference between the Pro Mic reading and the Uno Q raw output.
* **Step 4:** Define `float CAL_OFFSET = [Difference];` in your code.
* **Step 5:** Verify linearity at 70dB, 80dB, and 95dB.

---

## 📦 6. Developer Checklist
- [ ] **Library:** `Arduino_LED_Matrix.h`
- [ ] **Library:** `SparkFun_Qwiic_Joystick_Arduino_Library.h` (compatible with Nav Switch).
- [ ] **Library:** `arm_math.h` (CMSIS-DSP for fast RMS/Filtering).
- [ ] **Task:** Implement a rolling buffer for $L_{Aeq}$ using a `float` array of 480 samples (1 per 125ms over 60s).
- [ ] **Task:** Create bitmap arrays for letters A, B, C, L for the 8x13 grid.
- [ ] **Check:** Ensure the Qualcomm chip is not running heavy tasks to minimize EMI noise on the analog pins.

---
**Document Status:** *Ready for implementation.*
