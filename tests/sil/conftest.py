"""Fixtures for software-in-the-loop tests: run pdm_sim scenarios and load
their per-window log, telemetry and summary."""
import csv
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build"


def _find(name):
    exe = name + (".exe" if sys.platform.startswith("win") else "")
    for c in (BUILD / exe, BUILD / "Release" / exe, BUILD / "Debug" / exe):
        if c.exists():
            return c
    return None


@pytest.fixture(scope="session")
def sim_exe():
    exe = _find("pdm_sim")
    if exe is None:
        subprocess.run(["cmake", "-S", str(ROOT), "-B", str(BUILD)], check=True)
        subprocess.run(["cmake", "--build", str(BUILD), "--config", "Release"], check=True)
        exe = _find("pdm_sim")
    assert exe is not None
    return exe


class Run:
    def __init__(self, name, out):
        with open(out / f"{name}_windows.csv", newline="") as f:
            self.windows = list(csv.DictReader(f))
        self.telemetry = [l for l in (out / f"{name}_telemetry.log").read_text().splitlines() if l.strip()]
        self.summary = json.loads((out / f"{name}_summary.json").read_text())

    def first(self, pred):
        return next((w for w in self.windows if pred(w)), None)

    def health_seen(self):
        return {w["health"] for w in self.windows}


@pytest.fixture(scope="session")
def run(sim_exe, tmp_path_factory):
    out = tmp_path_factory.mktemp("sil")
    cache = {}

    def _run(name):
        if name not in cache:
            subprocess.run([str(sim_exe), name, str(out)], check=True, stdout=subprocess.DEVNULL)
            cache[name] = Run(name, out)
        return cache[name]

    return _run
