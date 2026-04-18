#!/usr/bin/env python3
"""Analyze expected noise floor with corrected A/C weighting filters on 14-bit ADC."""
import math, cmath
import numpy as np

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
W1 = prewarp(w1); W2 = prewarp(w2); W3 = prewarp(w3); W4 = prewarp(w4)

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

# ── Build A-weighting stages (per-stage normalized) ──
b0, a0 = hp2_biquad(W1)
b1, a1 = lp2_pair_biquad(W2, W3)
b2, a2 = hp2_biquad(W4)
stages_a = [(b0,a0),(b1,a1),(b2,a2)]
for i, (b, a) in enumerate(stages_a):
    g = abs(biquad_gain(b, a, 1000))
    stages_a[i] = ([x/g for x in b], a)

# ── Build C-weighting stages ──
cb0, ca0 = hp2_biquad(W1)
cb1, ca1 = lp2_double_biquad(W4)
stages_c = [(cb0,ca0),(cb1,ca1)]
for i, (b, a) in enumerate(stages_c):
    g = abs(biquad_gain(b, a, 1000))
    stages_c[i] = ([x/g for x in b], a)

# ── Compute overall gain at many frequencies ──
freqs = np.linspace(1, fs/2 - 1, 10000)

def overall_gain_db(stages, f):
    g = 1.0
    for b, a in stages:
        g *= abs(biquad_gain(b, a, f))
    return 20 * math.log10(max(g, 1e-30))

# Print A-weight response at key frequencies including near-Nyquist
print("=== A-weighting response (corrected, per-stage normalized) ===")
test_freqs = [20, 100, 250, 500, 1000, 2000, 4000, 8000, 10000, 12000, 
              16000, 20000, 22000, 23000, 23500, 23900, 23990]
for f in test_freqs:
    print(f"  {f:>6} Hz: {overall_gain_db(stages_a, f):>+8.2f} dB")

print("\n=== Per-stage gain at Nyquist (23999 Hz) ===")
for i, (b, a) in enumerate(stages_a):
    g = abs(biquad_gain(b, a, 23999))
    print(f"  Stage {i}: {20*math.log10(max(g,1e-30)):>+8.2f} dB  (linear: {g:.4f})")

# ── Simulate noise floor ──
# ADC: 14-bit, 3.3V range, midpoint at 8192
# Quantization noise RMS = LSB / sqrt(12)
adc_bits = 14
vref = 3.3
lsb = vref / (2**adc_bits)
adc_noise_rms_lsb = 1.0 / math.sqrt(12)  # ~0.289 LSB
# Real ADC noise is typically ~2-4 LSB RMS
adc_noise_rms_lsb_real = 3.0  # conservative estimate

print(f"\n=== ADC noise analysis ===")
print(f"  LSB = {lsb*1000:.4f} mV")
print(f"  Quantization noise RMS = {adc_noise_rms_lsb:.3f} LSB = {adc_noise_rms_lsb * lsb * 1000:.4f} mV")
print(f"  Realistic ADC noise = {adc_noise_rms_lsb_real:.1f} LSB RMS = {adc_noise_rms_lsb_real * lsb * 1000:.4f} mV")

# White noise PSD (flat from 0 to fs/2)
adc_noise_power = (adc_noise_rms_lsb_real * lsb)**2  # in V^2
noise_psd = adc_noise_power / (fs/2)  # V^2/Hz

# Compute A-weighted noise power by integrating |H(f)|^2 * PSD
print(f"\n=== Noise power integration ===")
df = (fs/2 - 1) / 10000
noise_power_unweighted = 0
noise_power_a = 0
noise_power_c = 0
for f in freqs:
    ga = 1.0
    gc = 1.0
    for b, a in stages_a:
        ga *= abs(biquad_gain(b, a, f))
    for b, a in stages_c:
        gc *= abs(biquad_gain(b, a, f))
    noise_power_unweighted += noise_psd * df
    noise_power_a += (ga**2) * noise_psd * df
    noise_power_c += (gc**2) * noise_psd * df

rms_unweighted = math.sqrt(noise_power_unweighted)
rms_a = math.sqrt(noise_power_a)
rms_c = math.sqrt(noise_power_c)

# Convert to SPL using the energyToDB formula:
# dB = 20*log10(Vrms) + 136  (with -42 dBV/Pa mic sensitivity + 94 dB ref)
MIC_SENS = -42.0
def vrms_to_spl(vrms):
    if vrms < 1e-15:
        return -999
    return 20*math.log10(vrms) - MIC_SENS + 94.0

print(f"\n  Unweighted noise: {rms_unweighted*1000:.4f} mV RMS -> {vrms_to_spl(rms_unweighted):.1f} dB SPL")
print(f"  A-weighted noise: {rms_a*1000:.4f} mV RMS -> {vrms_to_spl(rms_a):.1f} dB SPL")
print(f"  C-weighted noise: {rms_c*1000:.4f} mV RMS -> {vrms_to_spl(rms_c):.1f} dB SPL")

# What about with the OLD broken filter (35 dB less)?
print(f"\n  With OLD broken filter (-35 dB): A-weighted would read ~{vrms_to_spl(rms_a) - 35:.1f} dB SPL")

# What SPL level corresponds to various mV at the mic?
print(f"\n=== Mic output levels ===")
print(f"  Mic sensitivity: {MIC_SENS} dBV/Pa = {10**(MIC_SENS/20)*1000:.3f} mV/Pa")
for spl in [30, 40, 50, 60, 70, 80, 90, 94, 100, 110, 120]:
    pa = 10**((spl - 94)/20) * 1.0  # pressure in Pa (94 dB = 1 Pa)
    mv = pa * 10**(MIC_SENS/20) * 1000  # mV
    adc_counts = mv / (lsb * 1000)
    print(f"  {spl:>3} dB SPL -> {pa:.4f} Pa -> {mv:.4f} mV -> {adc_counts:.1f} ADC counts")

# Time-domain simulation: generate white noise, filter through biquads, compute RMS
print(f"\n=== Time-domain simulation (5 seconds of ADC noise) ===")
np.random.seed(42)
duration = 5.0
n_samples = int(duration * fs)
# Generate noise in ADC counts
noise_adc = np.random.normal(0, adc_noise_rms_lsb_real, n_samples)

# Apply biquad cascade
def apply_biquad(x, b, a):
    """Direct form II transposed biquad."""
    y = np.zeros_like(x)
    w1 = 0.0; w2 = 0.0
    for n in range(len(x)):
        y[n] = b[0]*x[n] + w1
        w1 = b[1]*x[n] - a[0]*y[n] + w2
        w2 = b[2]*x[n] - a[1]*y[n]
    return y

# Raw (unweighted) RMS
raw_rms_adc = np.sqrt(np.mean(noise_adc**2))
raw_vrms = raw_rms_adc / 16383.0 * 3.3

# A-weighted
sig = noise_adc.copy()
for b, a in stages_a:
    sig = apply_biquad(sig, b, a)
# Skip first 0.5s for filter settling
skip = int(0.5 * fs)
a_rms_adc = np.sqrt(np.mean(sig[skip:]**2))
a_vrms = a_rms_adc / 16383.0 * 3.3

# C-weighted
sig_c = noise_adc.copy()
for b, a in stages_c:
    sig_c = apply_biquad(sig_c, b, a)
c_rms_adc = np.sqrt(np.mean(sig_c[skip:]**2))
c_vrms = c_rms_adc / 16383.0 * 3.3

print(f"  Raw noise:  {raw_rms_adc:.2f} ADC RMS, {raw_vrms*1000:.4f} mV -> {vrms_to_spl(raw_vrms):.1f} dB SPL")
print(f"  A-weighted: {a_rms_adc:.2f} ADC RMS, {a_vrms*1000:.4f} mV -> {vrms_to_spl(a_vrms):.1f} dB SPL")
print(f"  C-weighted: {c_rms_adc:.2f} ADC RMS, {c_vrms*1000:.4f} mV -> {vrms_to_spl(c_vrms):.1f} dB SPL")
print(f"\n  A-weight amplification factor: {a_rms_adc/raw_rms_adc:.2f}x ({20*math.log10(a_rms_adc/raw_rms_adc):+.1f} dB)")
print(f"  C-weight amplification factor: {c_rms_adc/raw_rms_adc:.2f}x ({20*math.log10(c_rms_adc/raw_rms_adc):+.1f} dB)")

# Block-based analysis (like the sketch does it: 480 samples per block)
print(f"\n=== Block-based analysis (480 samples/block, like sketch) ===")
block_size = 480
n_blocks = (n_samples - skip) // block_size
a_block_energies = []
raw_block_energies = []
for i in range(n_blocks):
    start = skip + i * block_size
    end = start + block_size
    a_block_energies.append(np.mean(sig[start:end]**2))
    raw_block_energies.append(np.mean(noise_adc[start:end]**2))

# Average energy over all blocks (like LEQ)
avg_a_energy = np.mean(a_block_energies)
avg_raw_energy = np.mean(raw_block_energies)
a_leq_vrms = math.sqrt(avg_a_energy) / 16383.0 * 3.3
raw_leq_vrms = math.sqrt(avg_raw_energy) / 16383.0 * 3.3
print(f"  Raw LEQ:  {raw_leq_vrms*1000:.4f} mV -> {vrms_to_spl(raw_leq_vrms):.1f} dB SPL")
print(f"  A-wt LEQ: {a_leq_vrms*1000:.4f} mV -> {vrms_to_spl(a_leq_vrms):.1f} dB SPL")
