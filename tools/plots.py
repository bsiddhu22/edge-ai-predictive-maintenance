#!/usr/bin/env python3
"""Generate the figures used in the README.

    python tools/plots.py     (needs build/pdm_sim, build/pdm_datagen, ml/metrics.json)
"""
import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ml"))
import features as F  # noqa: E402
from train import load  # noqa: E402

IMG = ROOT / "docs" / "img"
OUT = ROOT / "out"
CLASSES = ["NORMAL", "IMBALANCE", "MISALIGNMENT", "BEARING"]
HEALTH_Y = {"OK": 0, "SUSPECT": 1, "ALARM": 2, "SENSOR_FAULT": 3}


def exe(name):
    suffix = ".exe" if sys.platform.startswith("win") else ""
    for c in (ROOT / "build" / (name + suffix), ROOT / "build" / "Release" / (name + suffix)):
        if c.exists():
            return str(c)
    sys.exit(f"{name} not built")


def spectra():
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "s.bin"
        subprocess.run([exe("pdm_datagen"), str(p), "1", "4", "777"], check=True, stdout=subprocess.DEVNULL)
        frames, y, _, sev = load(p)
    fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True, sharey=True)
    for c in range(4):
        a = ax.flat[c]
        x = frames[y == c][:, :, 0].astype(float) / 256.0
        xc = x - x.mean(axis=1, keepdims=True)
        spec = np.abs(np.fft.fft(xc * F.HANN, axis=1)[:, : F.N // 2]) ** 2 * 2 / (F.N * F.WIN_POWER)
        a.semilogy(F.FREQS[1:], spec.mean(axis=0)[1:], lw=1)
        for lo, hi in F.BANDS.values():
            a.axvspan(lo, hi, alpha=0.12, color="tab:orange")
        a.set_title(f"{CLASSES[c]}" + (f"  (severity {sev[y == c][0]:.2f})" if c else ""), fontsize=10)
        a.grid(alpha=0.3)
        a.set_ylim(1e-8, 1)
    for a in ax[1]:
        a.set_xlabel("Frequency [Hz]")
    for a in ax[:, 0]:
        a.set_ylabel("PSD, x axis [g²/bin]")
    fig.suptitle("Vibration spectrum by condition (shaded: 1x, 2x, 3-6x, 0.5-1.5 kHz feature bands)")
    fig.tight_layout()
    fig.savefig(IMG / "spectra.png", dpi=110)
    plt.close(fig)


def timeline(name, title):
    with open(OUT / f"{name}_windows.csv", newline="") as f:
        rows = list(csv.DictReader(f))
    t = [float(r["t_s"]) for r in rows]
    fig, ax = plt.subplots(2, 1, figsize=(11, 5.5), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})
    for k, c in enumerate(CLASSES):
        ax[0].plot(t, [float(r[f"p_{c.lower()}"]) for r in rows], lw=1.2, label=c)
    truth_change = [float(r["t_s"]) for i, r in enumerate(rows) if i and r["truth"] != rows[i - 1]["truth"]]
    for tc in truth_change:
        for a in ax:
            a.axvline(tc, color="k", ls=":", lw=1)
    ax[0].set_ylabel("class probability")
    ax[0].legend(loc="center left", fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[0].set_title(title, fontsize=11)
    ax[1].step(t, [HEALTH_Y[r["health"]] for r in rows], where="post", lw=1.6, color="tab:red")
    ax[1].set_yticks([0, 1, 2])
    ax[1].set_yticklabels(["OK", "SUSPECT", "ALARM"])
    ax[1].set_xlabel("time [s]")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(IMG / f"timeline_{name}.png", dpi=110)
    plt.close(fig)


def confusion():
    m = json.loads((ROOT / "ml" / "metrics.json").read_text())
    cm = np.array(m["confusion_int8"])
    fig, a = plt.subplots(figsize=(5.2, 4.4))
    a.imshow(cm, cmap="Blues")
    for i in range(4):
        for j in range(4):
            a.text(j, i, cm[i, j], ha="center", va="center", color="white" if cm[i, j] > cm.max() / 2 else "black")
    a.set_xticks(range(4)); a.set_yticks(range(4))
    a.set_xticklabels([c[:5] for c in CLASSES]); a.set_yticklabels(CLASSES)
    a.set_xlabel("predicted (int8 model)"); a.set_ylabel("true")
    a.set_title(f"Held-out sessions: int8 accuracy {m['accuracy_int8']:.1%}", fontsize=10)
    fig.tight_layout()
    fig.savefig(IMG / "confusion_int8.png", dpi=110)
    plt.close(fig)


def main():
    IMG.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(exist_ok=True)
    for s in ("bearing", "normal_marginal", "recovery"):
        subprocess.run([exe("pdm_sim"), s, str(OUT)], check=True, stdout=subprocess.DEVNULL)
    spectra()
    timeline("bearing", "Bearing defect appears at 5 s: SUSPECT at +0.44 s, ALARM at +0.92 s")
    timeline("normal_marginal", "Rough but healthy installation: single-window errors, debounce keeps health OK")
    timeline("recovery", "Imbalance repaired at 8 s: alarm clears only after 8 clean windows")
    confusion()
    print("figures written to", IMG)


if __name__ == "__main__":
    main()
