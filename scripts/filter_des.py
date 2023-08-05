import math

import numpy as np
import scipy.signal as sig

import matplotlib.pyplot as plt

FS = 12500  # Hz


def fir_deemph_spec(fs=FS):
    def rolloff(f):
        return ((math.log10(f) - 3.0) * -20) - 0

    points = [(10.0, -5.0), (30.0, 4.0), (100.0, 7.0),
              (200.0, 12.0), (250.0, 11.5)]

    for f in np.linspace(300, 7000, 200):
        points.append((f, rolloff(f)))

    freqs = []
    gains = []

    for f, g in points:
        freqs.append(f)
        gains.append(g)

    return freqs, gains


def standard_deemph(tau=50e-6, fs=FS):
    w_c = 1.0 / tau

    w_ca = 2.0 * fs * math.tan(w_c / (2.0 * fs))

    k = -w_ca / (2.0 * fs)
    z1 = -1.0
    p1 = (1.0 + k) / (1.0 - k)
    b0 = -k / (1.0 - k)

    btaps = [b0 * 1.0, b0 * -z1]
    ataps = [1.0, -p1]

    return btaps, ataps


def reson_lp(reson_freq, fs=FS, Q=2.0):
    gain = math.pow(10, (4.0 / 20))
    wc = 2.0 * math.pi * reson_freq
    a1 = 1.0 / Q
    a0 = 1.0
    b2 = 0.0
    b1 = 0.0
    b0 = gain

    return sig.bilinear([b2, b1, b0], [1.0, a1, a0], fs=fs / wc)


b1s, a1s = reson_lp(250)
print("reson coefs")
print(list(b1s), list(a1s))

w1, h1 = sig.freqz(b1s, a1s)
w1 *= (FS / 2) / math.pi

b2s, a2s = sig.butter(3, 5000, 'low', analog=True)
b2s, a2s = sig.bilinear(b2s, a2s, fs=FS)
print("deemph coefs")
print(list(b2s), list(a2s))

b3s, a3s = standard_deemph()
_, h3 = sig.freqz(b3s, a3s)

_, h2 = sig.freqz(b2s, a2s)
plt.plot(w1, 20 * np.log10(abs(h1)), label="reson", linestyle="dashed")
plt.plot(w1, 20 * np.log10(abs(h2)), label="deemph", linestyle="dashed")
plt.plot(w1, 20 * np.log10(abs(h1) + abs(h2)), label="combined")
plt.plot(w1, 20 * np.log10(abs(h3)),
         label="standard (50us)", linestyle="dashed")

plt.grid(True, which="both")
plt.xscale("log")
plt.xlim([100, FS / 2])
plt.ylim([-20, 15])
plt.legend()
plt.xlabel("Hz")
plt.ylabel("dB")
plt.show()

w, h = fir_deemph_spec()

plt.plot(w, h, linestyle="none", marker="x")
plt.xscale("log")
plt.grid(True, which="both")
plt.show()
