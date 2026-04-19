#!/usr/bin/env python3
"""Verify A/C weighting biquad coefficients and compute correct ones for 48 kHz."""
import cmath
import math

fs = 48000
k = 2 * fs  # bilinear transform constant


def biquad_gain(b, a, f):
    w = 2 * math.pi * f / fs
    z = cmath.exp(1j * w)
    zi = 1/z
    N = b[0] + b[1]*zi + b[2]*zi**2
    D = 1 + a[0]*zi + a[1]*zi**2
    return N / D


# ── Current (buggy) coefficients from sketch.ino ──
stages_a = [
    ([1, -2, 1], [-1.99462, 0.99462]),
    ([1, -2, 1], [-1.89392, 0.89521]),
    ([1, -2, 1], [-0.22456, 0.01261]),
]
stages_c = [
    ([1, -2, 1], [-1.99462, 0.99462]),
    ([1, -2, 1], [-0.22456, 0.01261]),
]

gain_a = 1.0
for b, a in stages_a:
    gain_a *= abs(biquad_gain(b, a, 1000))
print(f"CURRENT A-weight gain at 1 kHz: {gain_a:.6e}  = {20*math.log10(gain_a):.1f} dB  (should be 0 dB)")

gain_c = 1.0
for b, a in stages_c:
    gain_c *= abs(biquad_gain(b, a, 1000))
print(f"CURRENT C-weight gain at 1 kHz: {gain_c:.6e}  = {20*math.log10(gain_c):.1f} dB  (should be 0 dB)")

print("\nA-weight: 3 stages x [1,-2,1] = 6 zeros at DC (should be 4)")
print("C-weight: 2 stages x [1,-2,1] = 4 zeros at DC (should be 2)")

# ── Compute CORRECT biquad coefficients ──
print("\n=== Correct A-weighting biquads at 48 kHz ===")

w1 = 2*math.pi*20.598997
w2 = 2*math.pi*107.65265
w3 = 2*math.pi*737.86223
w4 = 2*math.pi*12194.217


def prewarp(w):
    return 2*fs*math.tan(w/(2*fs))


W1 = prewarp(w1)
W2 = prewarp(w2)
W3 = prewarp(w3)
W4 = prewarp(w4)


def hp2_biquad(W):
    """s^2/(s+W)^2 via bilinear"""
    a_c = (W - k)/(W + k)
    g = (k / (k + W))**2
    return [g, -2*g, g], [2*a_c, a_c**2]


def lp2_pair_biquad(Wa, Wb):
    """1/((s+Wa)(s+Wb)) via bilinear"""
    a2 = (Wa - k)/(Wa + k)
    a3 = (Wb - k)/(Wb + k)
    g = 1.0 / ((k+Wa)*(k+Wb))
    return [g, 2*g, g], [a2+a3, a2*a3]


def lp2_double_biquad(W):
    """1/(s+W)^2 via bilinear — note: W/(k+W) gain, (z+1)^2 numerator"""
    a_c = (W - k)/(W + k)
    g = (W / (k + W))**2
    return [g, 2*g, g], [2*a_c, a_c**2]


# A-weight: s^4 / [(s+w1)^2 (s+w2)(s+w3) (s+w4)^2]
# Partition: stage0 = s^2/(s+W1)^2, stage1 = 1/((s+W2)(s+W3)), stage2 = s^2/(s+W4)^2
b0, a0 = hp2_biquad(W1)
b1, a1 = lp2_pair_biquad(W2, W3)
b2, a2 = hp2_biquad(W4)

# Normalize to 0 dB at 1 kHz
g_total = 1.0
for b, a in [(b0, a0), (b1, a1), (b2, a2)]:
    g_total *= abs(biquad_gain(b, a, 1000))
norm = 1.0 / g_total
b0 = [x*norm for x in b0]

g_check = 1.0
for b, a in [(b0, a0), (b1, a1), (b2, a2)]:
    g_check *= abs(biquad_gain(b, a, 1000))
print(f"Post-normalization gain at 1 kHz: {20*math.log10(g_check):.4f} dB")

print("\n// A-weight stage 0: HP 20.6 Hz double pole + 2 zeros at DC")
print(f"  {{{b0[0]:.10f}f,{b0[1]:.10f}f,{b0[2]:.10f}f,  {a0[0]:.10f}f,{a0[1]:.10f}f, 0,0}},")
print("// A-weight stage 1: LP 107.7+737.9 Hz poles, NO zeros at DC")
print(f"  {{{b1[0]:.15e}f,{b1[1]:.15e}f,{b1[2]:.15e}f,  {a1[0]:.10f}f,{a1[1]:.10f}f, 0,0}},")
print("// A-weight stage 2: HP 12194 Hz double pole + 2 zeros at DC")
print(f"  {{{b2[0]:.10f}f,{b2[1]:.10f}f,{b2[2]:.10f}f,  {a2[0]:.10f}f,{a2[1]:.10f}f, 0,0}},")

# C-weight: s^2 / [(s+w1)^2 (s+w4)^2]
# Partition: stage0 = s^2/(s+W1)^2, stage1 = 1/(s+W4)^2
print("\n=== Correct C-weighting biquads at 48 kHz ===")
cb0, ca0 = hp2_biquad(W1)
cb1, ca1 = lp2_double_biquad(W4)

gc_total = 1.0
for b, a in [(cb0, ca0), (cb1, ca1)]:
    gc_total *= abs(biquad_gain(b, a, 1000))
cnorm = 1.0 / gc_total
cb0 = [x*cnorm for x in cb0]

gc_check = 1.0
for b, a in [(cb0, ca0), (cb1, ca1)]:
    gc_check *= abs(biquad_gain(b, a, 1000))
print(f"Post-normalization gain at 1 kHz: {20*math.log10(gc_check):.4f} dB")

print("\n// C-weight stage 0: HP 20.6 Hz double pole + 2 zeros at DC")
print(f"  {{{cb0[0]:.10f}f,{cb0[1]:.10f}f,{cb0[2]:.10f}f,  {ca0[0]:.10f}f,{ca0[1]:.10f}f, 0,0}},")
print("// C-weight stage 1: LP 12194 Hz double pole, NO zeros at DC")
print(f"  {{{cb1[0]:.15e}f,{cb1[1]:.15e}f,{cb1[2]:.15e}f,  {ca1[0]:.10f}f,{ca1[1]:.10f}f, 0,0}},")

# ── Spot-check against IEC 61672 Table 3 ──
print("\n=== A-weighting verification vs IEC 61672 ===")
print(f"{'freq':>8} {'computed':>10} {'IEC':>10} {'err':>8}")
iec_a = {
    31.5: -39.4, 63: -26.2, 125: -16.1, 250: -8.6, 500: -3.2,
    1000: 0.0, 2000: 1.2, 4000: 1.0, 8000: -1.1, 16000: -6.6,
}
for f, target in sorted(iec_a.items()):
    g = 1.0
    for b, a in [(b0, a0), (b1, a1), (b2, a2)]:
        g *= abs(biquad_gain(b, a, f))
    db = 20*math.log10(g)
    print(f"{f:>8} {db:>+10.2f} {target:>+10.1f} {db-target:>+8.2f}")

print("\n=== C-weighting verification vs IEC 61672 ===")
print(f"{'freq':>8} {'computed':>10} {'IEC':>10} {'err':>8}")
iec_c = {
    31.5: -3.0, 63: -0.8, 125: -0.2, 250: 0.0, 500: 0.0,
    1000: 0.0, 2000: -0.2, 4000: -0.8, 8000: -3.0, 16000: -8.5,
}
for f, target in sorted(iec_c.items()):
    g = 1.0
    for b, a in [(cb0, ca0), (cb1, ca1)]:
        g *= abs(biquad_gain(b, a, f))
    db = 20*math.log10(g)
    print(f"{f:>8} {db:>+10.2f} {target:>+10.1f} {db-target:>+8.2f}")
