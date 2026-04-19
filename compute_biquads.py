#!/usr/bin/env python3
"""Compute properly-scaled A/C weighting biquads for 48 kHz float32."""
import cmath
import math

fs = 48000
k = 2 * fs


def biquad_gain(b, a, f):
    w = 2 * math.pi * f / fs
    z = cmath.exp(1j * w)
    zi = 1/z
    N = b[0] + b[1]*zi + b[2]*zi**2
    D = 1 + a[0]*zi + a[1]*zi**2
    return N / D


def prewarp(w):
    return 2*fs*math.tan(w/(2*fs))


w1 = 2*math.pi*20.598997
w2 = 2*math.pi*107.65265
w3 = 2*math.pi*737.86223
w4 = 2*math.pi*12194.217
W1 = prewarp(w1)
W2 = prewarp(w2)
W3 = prewarp(w3)
W4 = prewarp(w4)


def hp2_biquad(W):
    a_c = (W - k)/(W + k)
    g = (k / (k + W))**2
    return [g, -2*g, g], [2*a_c, a_c**2]


def lp2_pair_biquad(Wa, Wb):
    a2 = (Wa - k)/(Wa + k)
    a3 = (Wb - k)/(Wb + k)
    g = 1.0 / ((k+Wa)*(k+Wb))
    return [g, 2*g, g], [a2+a3, a2*a3]


def lp2_double_biquad(W):
    a_c = (W - k)/(W + k)
    g = (W / (k + W))**2
    return [g, 2*g, g], [2*a_c, a_c**2]


# ── A-weighting ──
b0, a0 = hp2_biquad(W1)
b1, a1 = lp2_pair_biquad(W2, W3)
b2, a2 = hp2_biquad(W4)
stages_a = [(b0, a0), (b1, a1), (b2, a2)]

# Normalize each stage individually to |H(1kHz)|=1, then compute overall K
for i, (b, a) in enumerate(stages_a):
    g = abs(biquad_gain(b, a, 1000))
    stages_a[i] = ([x/g for x in b], a)

# Now each stage is ~0 dB at 1 kHz; overall product should also be ~0 dB
g_check = 1.0
for b, a in stages_a:
    g_check *= abs(biquad_gain(b, a, 1000))
print(f"A-weight gain at 1 kHz after per-stage norm: {20*math.log10(g_check):.4f} dB")

print("\n// Corrected A-weighting: 3 biquads, Fs=48 kHz, IEC 61672")
print("// Each stage individually normalized so b coefficients stay in float32 range")
for i, (b, a) in enumerate(stages_a):
    print(f"  // Stage {i}")
    # Check max b value fits float32
    print(f"  {{{b[0]:.10f}f, {b[1]:.10f}f, {b[2]:.10f}f,  {a[0]:.10f}f, {a[1]:.10f}f, 0,0}},")

# ── C-weighting ──
cb0, ca0 = hp2_biquad(W1)
cb1, ca1 = lp2_double_biquad(W4)
stages_c = [(cb0, ca0), (cb1, ca1)]

for i, (b, a) in enumerate(stages_c):
    g = abs(biquad_gain(b, a, 1000))
    stages_c[i] = ([x/g for x in b], a)

gc_check = 1.0
for b, a in stages_c:
    gc_check *= abs(biquad_gain(b, a, 1000))
print(f"\nC-weight gain at 1 kHz after per-stage norm: {20*math.log10(gc_check):.4f} dB")

print("\n// Corrected C-weighting: 2 biquads, Fs=48 kHz, IEC 61672")
for i, (b, a) in enumerate(stages_c):
    print(f"  // Stage {i}")
    print(f"  {{{b[0]:.10f}f, {b[1]:.10f}f, {b[2]:.10f}f,  {a[0]:.10f}f, {a[1]:.10f}f, 0,0}},")

# ── Verify IEC 61672 ──
print("\n=== A-weighting vs IEC 61672 ===")
print(f"{'freq':>8} {'dB':>8} {'IEC':>8} {'err':>8}")
iec_a = {
    31.5: -39.4, 63: -26.2, 125: -16.1, 250: -8.6, 500: -3.2,
    1000: 0.0, 2000: 1.2, 4000: 1.0, 8000: -1.1, 16000: -6.6,
}
for f, target in sorted(iec_a.items()):
    g = 1.0
    for b, a in stages_a:
        g *= abs(biquad_gain(b, a, f))
    db = 20*math.log10(g)
    print(f"{f:>8} {db:>+8.2f} {target:>+8.1f} {db-target:>+8.2f}")

print("\n=== C-weighting vs IEC 61672 ===")
print(f"{'freq':>8} {'dB':>8} {'IEC':>8} {'err':>8}")
iec_c = {
    31.5: -3.0, 63: -0.8, 125: -0.2, 250: 0.0, 500: 0.0,
    1000: 0.0, 2000: -0.2, 4000: -0.8, 8000: -3.0, 16000: -8.5,
}
for f, target in sorted(iec_c.items()):
    g = 1.0
    for b, a in stages_c:
        g *= abs(biquad_gain(b, a, f))
    db = 20*math.log10(g)
    print(f"{f:>8} {db:>+8.2f} {target:>+8.1f} {db-target:>+8.2f}")
