import math
import cmath
import random

import numpy as np

import scipy.signal as sig
from scipy.fftpack import fft, fftshift
import scipy.io.wavfile as wv

import matplotlib.pyplot as plt


def array_prnt(data):
    s = [data[n:n+8] for n in range(0, len(data), 8)]

    for l in s:
        print("    " + ", ".join(
            list(map(lambda n: "{}{:.8f}f".format(" " if n >= 0 else "", round(n, 8)), l))) + ",")
    print("---")


class SimpleBiquad:
    def __init__(self, b, a):
        self.b0, self.b1, self.b2 = b
        self.a0, self.a1, self.a2 = a
        self.x1, self.x2, self.y1, self.y2 = [0.0] * 4

    def __call__(self, x):
        y = self.b0 * x + self.b1 * self.x1 + self.b2 * \
            self.x2 - self.a1 * self.y1 - self.a2 * self.y2
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y

    def get_taps(self):
        return [self.b0, self.b1, self.b2], [self.a0, self.a1, self.a2]


class PLL:
    def __init__(self, cf, sr, loop_gain=8.0, loop_filt=False):
        self.ref = 0.0
        self.old_ref = 0.0
        self.phase = 0.0
        self.integral = 0.0
        self.cf = cf
        self.sr = sr
        self.loop_gain = loop_gain

        if not loop_filt:
            self.loop_filt = None
        else:
            self.loop_filt = SimpleBiquad(*sig.butter(2, 100, fs=self.sr))

        self.out_filt = SimpleBiquad(*sig.butter(2, 5, fs=self.sr))
        array_prnt(np.concatenate(self.out_filt.get_taps()))
        self.lock_filt = SimpleBiquad(*sig.butter(2, 5, fs=self.sr))

        b_taps, a_taps = self.out_filt.get_taps()
        w, h = sig.freqz(b_taps, a_taps)

        w *= (self.sr / 2) / math.pi
        plt.plot(w, 20 * np.log10(abs(h)), label="output filter")

        b_taps, a_taps = self.lock_filt.get_taps()
        _, h = sig.freqz(b_taps, a_taps)
        plt.plot(w, 20 * np.log10(abs(h)), label="lock filter")

        if loop_filt:
            b_taps, a_taps = self.loop_filt.get_taps()
            _, h = sig.freqz(b_taps, a_taps)
            plt.plot(w, 20 * np.log10(abs(h)), label="loop filter")

        plt.grid(True)
        plt.xscale("log")
        plt.xlim([0, self.sr / 1.8])
        #plt.ylim([-20, 15])
        plt.legend()
        plt.show()

    def update(self, x):
        loop_control = x * self.ref * self.loop_gain
        if not self.loop_filt is None:
            loop_control = self.loop_filt(loop_control)
        out = self.out_filt(loop_control)
        self.integral += loop_control / self.sr
        self.ref = math.sin(
            2 * math.pi * self.cf * (self.phase + self.integral))
        quad_ref = math.cos(
            2 * math.pi * self.cf * (self.phase + self.integral))
        lock = self.lock_filt(-quad_ref * x)

        self.phase += 1.0 / self.sr
        self.old_ref = self.ref

        return (out if self.loop_filt is None else
                (2 / self.loop_gain) * out), 2 * lock

    def __call__(self, xs):
        out = np.array(list(map(lambda x: self.update(x), xs)))
        return out.transpose()


def plot(pll, t, s, f1, f2):

    pll_out, lock_out = pll(s)
    fig, axs = plt.subplots(2, figsize=(40, 50))
    for ax in axs:
        ax.grid(True)

    axs[0].plot(t, s, alpha=0.8)
    axs[0].set_xlabel("time [s]")

    unlocked = np.ma.masked_where(lock_out > 0.7, pll_out)
    locked = np.ma.masked_where(lock_out <= 0.7, pll_out)

    cf = (f2 - f1) / 2
    ax2 = axs[0].twinx()
    ax2.plot(t, (locked * cf) + cf + f1, color="green")
    ax2.plot(t, (unlocked * cf) + cf + f1, color="red")
    ax2.set_xlabel("time [s]")

    axs[1].plot(t, lock_out)
    axs[1].set_xlabel("time [s]")
    plt.show()


pll1 = PLL(150, 12500)

t1 = np.arange(0, 4 + (1 / 12500), 1 / 12500)
s1 = sig.chirp(t1, f0=0, f1=300, t1=4, method='linear')
noise = (np.random.rand(len(s1)) * 2) - 1
s1 += noise

plot(pll1, t1, s1, 0, 300)

sr2, s2 = wv.read('samples/ctcss_sample.wav')
s2 = s2.astype(np.float32)
s2 /= np.iinfo(np.int32).max
t2 = np.arange(0, (len(s2) / sr2), 1 / sr2)

plot(pll1, t2, s2, 0, 300)
