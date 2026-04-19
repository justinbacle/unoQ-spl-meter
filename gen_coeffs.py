"""Generate correct A/C weight biquad coefficients using scipy bilinear transform."""
import numpy as np
from scipy import signal

fs = 48000

# IEC 61672 pole/zero frequencies
f1, f2, f3, f4 = 20.598997, 107.65265, 737.86223, 12194.217
w1, w2, w3, w4 = [2*np.pi*f for f in (f1, f2, f3, f4)]

# === A-weighting ===
z_a = [0, 0, 0, 0]
p_a = [-w1, -w1, -w2, -w3, -w4, -w4]
# Normalize at 1 kHz
s1k = 2j * np.pi * 1000
num_a = s1k**4
den_a = ((s1k + w1)**2) * (s1k + w2) * (s1k + w3) * ((s1k + w4)**2)
k_a = abs(den_a / num_a)

z_ad, p_ad, k_ad = signal.bilinear_zpk(z_a, p_a, k_a, fs)
sos_a = signal.zpk2sos(z_ad, p_ad, k_ad)

# Fine-tune normalization to exactly 0 dB at 1 kHz
_, h1k = signal.sosfreqz(sos_a, worN=[1000], fs=fs)
gain_1k = abs(h1k[0])
sos_a[0, :3] /= gain_1k  # adjust b coefficients of first stage

# === C-weighting ===
z_c = [0, 0]
p_c = [-w1, -w1, -w4, -w4]
num_c = s1k**2
den_c = ((s1k + w1)**2) * ((s1k + w4)**2)
k_c = abs(den_c / num_c)

z_cd, p_cd, k_cd = signal.bilinear_zpk(z_c, p_c, k_c, fs)
sos_c = signal.zpk2sos(z_cd, p_cd, k_cd)

_, h1k_c = signal.sosfreqz(sos_c, worN=[1000], fs=fs)
gain_1k_c = abs(h1k_c[0])
sos_c[0, :3] /= gain_1k_c

# === Verify ===
freqs = [31.5, 63, 125, 250, 500, 1000, 2000, 4000, 8000, 16000]
iec_a = {
    31.5: -39.4, 63: -26.2, 125: -16.1, 250: -8.6, 500: -3.2,
    1000: 0.0, 2000: 1.2, 4000: 1.0, 8000: -1.1, 16000: -6.6,
}
iec_c = {
    31.5: -3.0, 63: -0.8, 125: -0.2, 250: 0.0, 500: 0.0,
    1000: 0.0, 2000: -0.2, 4000: -0.8, 8000: -3.0, 16000: -8.5,
}

_, ha = signal.sosfreqz(sos_a, worN=freqs, fs=fs)
_, hc = signal.sosfreqz(sos_c, worN=freqs, fs=fs)

print("A-weight IEC conformance:")
for f, h in zip(freqs, ha):
    db = 20*np.log10(abs(h))
    print(f"  {f:>6} Hz: {db:>+8.4f} dB  IEC: {iec_a[f]:>+6.1f}  err: {db-iec_a[f]:>+7.4f}")

print("\nC-weight IEC conformance:")
for f, h in zip(freqs, hc):
    db = 20*np.log10(abs(h))
    print(f"  {f:>6} Hz: {db:>+8.4f} dB  IEC: {iec_c[f]:>+6.1f}  err: {db-iec_c[f]:>+7.4f}")

# === Output for sketch ===
print("\n// === A-weight biquad coefficients (3 stages) ===")
print("static Biquad aFilt[3] = {")
for i, s in enumerate(sos_a):
    comma = "," if i < len(sos_a)-1 else ""
    print(f"  {{ {s[0]:.10f}f, {s[1]:.10f}f, {s[2]:.10f}f, {s[4]:.10f}f, {s[5]:.10f}f, 0,0}}{comma}")
print("};")

print("\n// === C-weight biquad coefficients (2 stages) ===")
print("static Biquad cFilt[2] = {")
for i, s in enumerate(sos_c):
    comma = "," if i < len(sos_c)-1 else ""
    print(f"  {{ {s[0]:.10f}f, {s[1]:.10f}f, {s[2]:.10f}f, {s[4]:.10f}f, {s[5]:.10f}f, 0,0}}{comma}")
print("};")

# === Broadband comparison ===
print("\n// Broadband pink noise test:")
f_range = np.arange(20, 20001, dtype=float)
_, ha_all = signal.sosfreqz(sos_a, worN=f_range, fs=fs)
_, hc_all = signal.sosfreqz(sos_c, worN=f_range, fs=fs)
pink = 1.0 / f_range
a_power = 10*np.log10(np.sum(np.abs(ha_all)**2 * pink))
c_power = 10*np.log10(np.sum(np.abs(hc_all)**2 * pink))
print(f"// A-weight pink noise total: {a_power:.4f} dB")
print(f"// C-weight pink noise total: {c_power:.4f} dB")
