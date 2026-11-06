"""int8 post-training quantization and a bit-exact integer reference of the
firmware kernels in firmware/src/nn.c (TFLite int8 scheme, gemmlowp math)."""
import math

import numpy as np

INT32_MIN = -(2 ** 31)


def quantize_multiplier(real):
    """real -> (Q31 multiplier, shift) with real = m * 2^(shift - 31)."""
    if real == 0.0:
        return 0, 0
    q, shift = math.frexp(real)              # real = q * 2^shift, q in [0.5, 1)
    q_fixed = int(round(q * (1 << 31)))
    if q_fixed == (1 << 31):
        q_fixed //= 2
        shift += 1
    return q_fixed, shift


def _srdhm(a, b):
    """SaturatingRoundingDoublingHighMul, vectorised, int64 math."""
    a = np.asarray(a, dtype=np.int64)
    ab = a * np.int64(b)
    nudge = np.where(ab >= 0, np.int64(1 << 30), np.int64(1 - (1 << 30)))
    x = ab + nudge
    d = np.int64(1 << 31)
    res = np.where(x >= 0, x // d, -((-x) // d))   # truncate toward zero like C
    if b == INT32_MIN:
        res = np.where(a == INT32_MIN, np.int64(2 ** 31 - 1), res)
    return res


def _rdbpot(x, exponent):
    x = np.asarray(x, dtype=np.int64)
    mask = np.int64((1 << exponent) - 1)
    remainder = x & mask
    threshold = (mask >> 1) + (x < 0).astype(np.int64)
    return (x >> exponent) + (remainder > threshold).astype(np.int64)


def requantize(acc, multiplier, shift):
    left = shift if shift > 0 else 0
    right = 0 if shift > 0 else -shift
    return _rdbpot(_srdhm(np.asarray(acc, dtype=np.int64) * (1 << left), multiplier), right)


def round_half_away(x):
    return np.sign(x) * np.floor(np.abs(x) + 0.5)


def asym_params(lo, hi):
    lo, hi = min(lo, 0.0), max(hi, 0.0)
    scale = (hi - lo) / 255.0
    zp = int(np.clip(round(-128 - lo / scale), -128, 127))
    return scale, zp


def quantize_model(W1, b1, W2, b2, z_calib):
    """W1: (IN,H), W2: (H,OUT) as in sklearn. z_calib: standardized features."""
    in_scale, in_zp = asym_params(float(z_calib.min()), float(z_calib.max()))
    h_float = np.maximum(z_calib @ W1 + b1, 0.0)
    h_scale = float(h_float.max()) / 255.0
    h_zp = -128
    out_float = h_float @ W2 + b2
    out_scale, out_zp = asym_params(float(out_float.min()), float(out_float.max()))

    sw1 = float(np.abs(W1).max()) / 127.0
    sw2 = float(np.abs(W2).max()) / 127.0
    qw1 = np.clip(round_half_away(W1.T / sw1), -127, 127).astype(np.int8)    # (H, IN)
    qw2 = np.clip(round_half_away(W2.T / sw2), -127, 127).astype(np.int8)    # (OUT, H)
    qb1 = round_half_away(b1 / (in_scale * sw1)).astype(np.int64)
    qb2 = round_half_away(b2 / (h_scale * sw2)).astype(np.int64)
    m1, s1 = quantize_multiplier(in_scale * sw1 / h_scale)
    m2, s2 = quantize_multiplier(h_scale * sw2 / out_scale)
    return dict(in_scale=in_scale, in_zp=in_zp, h_scale=h_scale, h_zp=h_zp,
                out_scale=out_scale, out_zp=out_zp, w1=qw1, b1=qb1, m1=m1, s1=s1,
                w2=qw2, b2=qb2, m2=m2, s2=s2)


def quantize_input(z, q):
    v = round_half_away(z / q["in_scale"]) + q["in_zp"]
    return np.clip(v, -128, 127).astype(np.int8)


def infer_int8(q_in, q):
    """Bit-exact reference of nn_infer(). q_in: (W, IN) int8 -> (W, OUT) int8 logits."""
    x = q_in.astype(np.int64) - q["in_zp"]
    acc1 = x @ q["w1"].astype(np.int64).T + q["b1"]
    h = np.clip(requantize(acc1, q["m1"], q["s1"]) + q["h_zp"], q["h_zp"], 127)
    acc2 = (h - q["h_zp"]) @ q["w2"].astype(np.int64).T + q["b2"]
    return np.clip(requantize(acc2, q["m2"], q["s2"]) + q["out_zp"], -128, 127).astype(np.int8)
