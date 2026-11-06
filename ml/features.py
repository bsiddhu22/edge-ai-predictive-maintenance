"""Offline (host) feature extraction. MUST match firmware/src/dsp.c exactly:
the golden-vector unit test compares the two on the same raw windows."""
import numpy as np

FS = 3200
N = 512
LSB_PER_G = 256.0
EPS = 1e-6
BANDS = {"1x": (18.0, 32.0), "2x": (43.0, 57.0), "harm": (70.0, 160.0), "hf": (500.0, 1500.0)}
FEATURE_NAMES = ["logrms", "crest", "kurt", "e1x", "e2x", "eharm", "ehf", "fpk"]

_i = np.arange(N)
HANN = 0.5 - 0.5 * np.cos(2.0 * np.pi * _i / N)          # periodic Hann
WIN_POWER = float(np.sum(HANN ** 2))
FREQS = np.arange(N // 2) * FS / N


def features_axis(x_lsb):
    """x_lsb: (W, N) raw sensor counts for one axis -> (W, 8) features."""
    x = x_lsb.astype(np.float64) / LSB_PER_G
    xc = x - x.mean(axis=1, keepdims=True)
    m2 = np.mean(xc ** 2, axis=1)
    m4 = np.mean(xc ** 4, axis=1)
    peak = np.max(np.abs(xc), axis=1)
    rms = np.sqrt(m2)

    spec = np.fft.fft(xc * HANN, axis=1)[:, : N // 2]
    psd = (np.abs(spec) ** 2) * 2.0 / (N * WIN_POWER)
    psd[:, 0] = 0.0
    kmax = 1 + np.argmax(psd[:, 1:], axis=1)

    def band(lo, hi):
        mask = (FREQS >= lo) & (FREQS < hi)
        mask[0] = False
        return psd[:, mask].sum(axis=1)

    return np.stack([
        np.log10(rms + EPS),
        peak / (rms + EPS),
        m4 / (m2 * m2 + EPS),
        np.log10(band(*BANDS["1x"]) + EPS),
        np.log10(band(*BANDS["2x"]) + EPS),
        np.log10(band(*BANDS["harm"]) + EPS),
        np.log10(band(*BANDS["hf"]) + EPS),
        kmax * (FS / N) / 100.0,
    ], axis=1)


def features(frames):
    """frames: (W, N, 3) int16 -> (W, 24) float64, axis-major (x feats, y feats, z feats)."""
    return np.concatenate([features_axis(frames[:, :, a]) for a in range(3)], axis=1)


def feature_names():
    return [f"{ax}_{n}" for ax in "xyz" for n in FEATURE_NAMES]
