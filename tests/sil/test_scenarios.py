"""SIL acceptance tests: each test is a requirement checked against the
firmware running on the simulated motor and sensor."""
import pytest

FAULTS = {"imbalance": "IMBALANCE", "misalignment": "MISALIGNMENT", "bearing": "BEARING"}
ONSET_S = 5.0


# ------------------------------------------------------------ healthy motor
def test_healthy_motor_never_alarms(run):
    r = run("normal")
    assert r.health_seen() == {"OK"}
    assert r.summary["dropped_windows"] == 0 and r.summary["overruns"] == 0


def test_every_window_processed(run):
    r = run("normal")
    # 20 s at 160 ms per window = 125 windows (the last may straddle the end)
    assert r.summary["valid_windows"] >= 124


def test_rough_installation_debounced(run):
    """Individual windows are misclassified, but the k-of-n debounce keeps the alarm quiet."""
    r = run("normal_marginal")
    assert sum(w["cls"] != "0" for w in r.windows) > 0
    assert "ALARM" not in r.health_seen()


# ------------------------------------------------------------------- faults
@pytest.mark.parametrize("scenario", list(FAULTS))
def test_fault_detected_and_diagnosed(run, scenario):
    r = run(scenario)
    assert r.summary["final_health"] == "ALARM"
    assert r.summary["fault_class"] == FAULTS[scenario]
    alarm = r.first(lambda w: w["health"] == "ALARM")
    assert ONSET_S < float(alarm["t_s"]) < ONSET_S + 1.5          # alarm within 1.5 s


@pytest.mark.parametrize("scenario", list(FAULTS))
def test_no_alarm_before_fault(run, scenario):
    r = run(scenario)
    assert all(w["health"] == "OK" for w in r.windows if float(w["t_s"]) < ONSET_S)


def test_intermittent_bursts_do_not_alarm(run):
    r = run("intermittent")
    assert "ALARM" not in r.health_seen()


def test_alarm_clears_after_repair_with_hysteresis(run):
    r = run("recovery")
    assert r.first(lambda w: w["health"] == "ALARM") is not None
    ok = r.first(lambda w: float(w["t_s"]) > 8.0 and w["health"] == "OK")
    assert ok is not None
    # needs 8 clean windows (1.28 s), not one
    assert 8.0 + 1.2 <= float(ok["t_s"]) <= 8.0 + 2.0


# --------------------------------------------------------- real-time budget
def test_processing_within_deadline(run):
    s = run("normal").summary
    assert s["timing_us"]["window_total"]["max"] < 0.1 * s["deadline_us"]   # >10x margin
    assert s["cpu_load_pct"] < 5.0


def test_blocking_debug_dump_blows_deadline_and_is_caught(run):
    r = run("debug_dump")
    s = r.summary
    assert s["timing_us"]["window_total"]["max"] > s["deadline_us"]
    assert s["valid_windows"] == 0 and s["dropped_windows"] > 0           # nothing torn gets classified
    assert s["overruns"] > 0


def test_unprotected_torn_windows_are_silent(run):
    """Without the deadline check, torn windows are classified as if nothing
    happened: the corruption is silent, which is why the check must be explicit."""
    r = run("debug_dump_unprotected")
    torn = [w for w in r.windows if w["valid"] == "0"]
    assert len(torn) == len(r.windows) > 0
    assert all(w["cls"] != "-1" for w in torn)


def test_fast_uart_dump_fits_deadline(run):
    s = run("debug_dump_fast").summary
    assert s["dropped_windows"] == 0 and s["overruns"] == 0
    assert s["timing_us"]["window_total"]["max"] < s["deadline_us"]


# ------------------------------------------------------------ sensor fault
def test_sensor_fault_detected_at_boot(run):
    r = run("sensor_fault")
    s = r.summary
    assert s["accel_status"] != 0 and s["devid"] == "0x00"
    assert s["final_health"] == "SENSOR_FAULT"
    assert s["valid_windows"] == 0
    assert any(",SENSOR_FAULT," in line for line in r.telemetry)


# --------------------------------------------------------------- telemetry
def _checksum_ok(line):
    body = line.split(" ", 1)[1].strip()
    payload, cs = body[1:].split("*")
    calc = 0
    for ch in payload:
        calc ^= ord(ch)
    return calc == int(cs, 16)


@pytest.mark.parametrize("scenario", ["normal", "bearing", "debug_dump"])
def test_telemetry_well_formed(run, scenario):
    r = run(scenario)
    assert r.telemetry, "no telemetry"
    assert all(_checksum_ok(l) for l in r.telemetry)


def test_telemetry_reports_alarm(run):
    r = run("bearing")
    assert ",ALARM,BEARING," in r.telemetry[-1]
