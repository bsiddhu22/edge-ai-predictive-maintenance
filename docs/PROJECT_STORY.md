# Project Story: Edge-AI Predictive Maintenance Node (for the Zentra interview)

This replaces Part 1B / Part 2 of your story doc for this project. Every claim below matches the code and the measured results in this repo. Before Tuesday, read §1 first: it lists what changed from your doc and why.

---

## 1. What changed from your story doc

| Your doc said | What is true now | Why it matters |
|---|---|---|
| STM32 Cortex-M4 board, bench motor, recorded data | Firmware written for a Cortex-M4F + ADXL345-compatible sensor, **run in simulation** against an emulated sensor and a motor-vibration model; data is **synthetic** | Say "built and validated in simulation". Never "on my bench". |
| Quantized **TensorFlow Lite for Microcontrollers** model | int8 model using **TFLite's quantization scheme**, with **my own C inference kernels** (gemmlowp fixed-point requantization), bit-exact with a Python reference | Don't say you used the TFLM interpreter. Explain why own kernels: 448 weights, no interpreter overhead, full control. |
| Binary normal/abnormal | **4 classes:** normal, imbalance, misalignment, bearing | Diagnosis, not just detection. |
| No numbers | Real numbers from runs (§4). Timing is from a **cycle-cost model**, not silicon | Always add "estimated" to timing numbers. |
| Torn windows caused **false alarms** | Reproduced: torn windows were classified **NORMAL at 98% confidence**, so the corruption was **silent** | Better story (§3, story 1). |
| Feature mismatch laptop vs board | **Golden-vector parity test** between Python and C: features within 9e-6, int8 outputs bit-exact | The check exists and runs in CI. |
| No state machine, no debounce, no HIL | **Health state machine** with 3-of-8 / 6-of-8 debounce and hysteresis; **automated SIL** tests in CI | Now legitimately covers JD state machines, diagnostics, SIL. |

---

## 2. Sixty-second version

"I built an edge-AI predictive-maintenance node: bare-metal C firmware for a Cortex-M4 that detects and diagnoses motor faults on the device. I built and validated it in simulation, against an emulated accelerometer and a motor-vibration model.
- **Acquisition:** DMA samples three axes at 3.2 kHz into a ping-pong buffer. The interrupts only flag which half is ready.
- **Processing:** the main loop computes FFT-based features and runs an int8 neural network I quantized myself, using TFLite's scheme with my own C kernels. It's bit-exact with my Python reference, and that's checked in CI.
- **Health logic:** a debounced state machine turns per-window predictions into OK, SUSPECT or ALARM. It alarms 0.9 seconds after a fault appears, but stays quiet when single windows are wrong.

The most interesting part was a timing problem. I added a blocking UART dump of raw windows for data collection. It took 268 milliseconds against a 160-millisecond deadline, so DMA was overwriting data mid-processing. I expected false alarms, but every torn window was still classified normal with 98% confidence. The corruption was completely silent. So I made deadline misses explicit: a late window is dropped and counted, never classified. That's the mindset real-time power firmware needs."

---

## 3. Detailed story

**Problem.** Motor faults change the vibration spectrum before failure:
- imbalance shows at 1x shaft speed
- misalignment shows at 2x and axially
- bearing defects show as impacts that ring a high-frequency resonance

I wanted detection to run on the microcontroller next to the motor, with no dependency on a network. That makes it a hard real-time firmware problem: samples arrive on the hardware's schedule.

**Role and scope.** A solo personal project covering:
- requirements and architecture
- driver, DMA acquisition and DSP
- data generation, training and int8 quantization
- the inference kernels and the health logic
- the simulator and the test suites

No hardware was used.

**Requirements I set.**
1. Sample 3 axes at 3.2 kHz with no gaps.
2. Process each 512-sample window before DMA returns to it (160 ms).
3. Classify on-device into 4 conditions.
4. Never classify a corrupted window.
5. Static memory only.
6. Don't alarm on single-window errors.
7. Report status over UART with integrity checking.
8. Detect a missing or misconfigured sensor at boot.

**Architecture.**
- **Sensor and DMA:** ADXL345-compatible accelerometer → timer-triggered SPI DMA in circular mode into 2 × 512 frames.
- **ISRs:** the half and full-transfer interrupts only set a flag.
- **Super-loop:** sleeps in WFI. On a flag it takes the newest half and computes features in place, axis by axis.
- **Deadline check:** then it checks the window. Only a valid window goes on to int8 inference, the health state machine and telemetry.
- **Hardware boundary:** everything hardware-specific sits behind `hal.h`. The simulator implements it on a virtual clock where DMA interrupts preempt the main loop exactly as they would on target.

**Key decisions and rejected alternatives.**
- **DMA vs polling/per-sample interrupts:** regular sampling independent of CPU load; two interrupts per window instead of 512.
- **Ping-pong in place (zero-copy) vs copying the window:** saves 3 KB of RAM and the copy time, but makes the deadline strict, so I had to detect violations.
- **Features vs raw samples into a CNN:** the fault physics lives in specific bands. 24 features means a 448-weight model and a 15 µs inference.
- **Own int8 kernels vs the TFLM interpreter:** for a 2-layer dense model, an interpreter adds flash, RAM arena and overhead. I kept TFLite's quantization spec so the model could move to TFLM later.
- **Bare-metal vs RTOS:** one pipeline, one deadline. I'd add an RTOS when there are concurrent tasks with different priorities (comms stack, logging).
- **Split train/test by installation, not by window:** windows from one session are near-duplicates, so a window-level split would leak and inflate accuracy.

**Data.** `pdm_datagen` creates 240 "installations", each with its own:
- speed (23.5–26.5 Hz)
- mounting gain per axis (±30%)
- noise level
- residual unbalance
- fault severity (0.1–1.5)

It records 15 windows from each, through the same sensor conversion the firmware sees, for 3,600 windows in total. My first version gave 100% test accuracy. That was a sign the data was unrealistically clean, so I widened installation variation and added incipient faults until the classes overlapped the way real machines do.

**Debugging stories (true, reproducible from this repo)**

1. *The silent torn windows.*
   - **Setup:** I added raw-window streaming over UART to collect data. It blocks for 268 ms per window against a 160 ms deadline, so DMA wraps around and overwrites the half while the features are still being computed.
   - **Surprise:** I expected the stitched windows to cause false alarms. They didn't. All 45 were classified NORMAL at 98% confidence, and features moved by under 1%.
   - **Lesson:** you can't detect timing failures from the output; the system has to check its own deadline.
   - **Fix:** after processing, compare the DMA completion counter to its value when the window was taken. If DMA has moved on, the window is torn: drop it and count it. Also count a half that completes while still unconsumed (an overrun).
   - **Second fix:** move the dump to 921,600 baud (34 ms), and in production make UART non-blocking.
   - *Reproduce:* `pdm_sim debug_dump_unprotected` vs `debug_dump` vs `debug_dump_fast`.

2. *The node went silent exactly when it was in trouble.*
   - **Found by:** a SIL test checking telemetry in the dump scenario. It failed with no telemetry at all.
   - **Cause:** status was only sent after *valid* windows, so when every window was being dropped, the node stopped reporting.
   - **Fix:** telemetry is now driven by windows seen, valid or not, so dropped and overrun counters always reach the supervisor.
   - *Reproduce:* revert `maybe_send_telemetry()` to count only valid windows and run `pytest tests/sil`.

3. *Bit-exact int8 needs exact rounding semantics.*
   - **Found by:** my own unit test expected −50.5 to round to −51.
   - **Cause:** gemmlowp's fixed-point multiply rounds ties toward +∞ for negatives (−50.5 → −50).
   - **Why it matters:** the Python reference had to implement truncate-toward-zero division explicitly, because Python's `//` floors. A one-LSB disagreement would break the golden test.

**Testing.**
- **Unit tests (46 checks):**
  - driver against an emulated register map, including a wrong-identity case
  - ping-pong and overrun logic
  - FFT against a naive DFT
  - features on pure tones
  - requantization reference cases
  - health debounce and hysteresis
  - telemetry checksum
  - golden-vector parity with Python
- **SIL acceptance tests (20):** one per requirement, run on every push in CI on Linux and Windows.
- **Retrain job:** regenerates the data, retrains and checks that accuracy holds.

---

## 4. Numbers you can quote

| Metric | Value |
|---|---|
| Held-out accuracy (72 unseen installations, 1,080 windows) | int8 **95.1%**, float 94.4%, agreement 99.2% |
| Fault recall by severity | 0.1–0.3: **67%**; ≥ 0.3: **100%** |
| Healthy windows flagged individually | 10.4% → health stays OK because of the 3-of-8 / 6-of-8 debounce |
| Alarm latency | SUSPECT +0.44 s, ALARM +0.92 s (6 windows) |
| Recovery | OK 1.28 s after repair (8 clean windows) |
| Processing per window *(estimated, 168 MHz M4F)* | features 0.73 ms, inference 15 µs, average total 1.37 ms vs 160 ms deadline |
| CPU load *(estimated)* | 0.8% |
| Model flash | 752 B (448 int8 weights vs 1,792 B as float32) |
| RAM | 6 KB DMA buffer + 12 KB DSP working memory, static |
| Golden parity | max relative feature error 8.9e-6; int8 logits bit-exact |

---

## 5. Zentra JD: what this project now supports

- **Supported:**
  - real-time embedded C
  - microcontroller peripherals (SPI, DMA, interrupts)
  - drivers with an identity check
  - deterministic deadline handling
  - fault detection and diagnostics
  - state machine with debounce and hysteresis
  - telemetry with integrity checking
  - SIL and automated firmware validation
  - motor domain
- **Say clearly you have not done:**
  - real hardware bring-up or lab equipment
  - control loops or protection that acts on hardware
  - deterministic industrial buses
  - bootloaders or updates
  - production or manufacturing
- **Bridge:** "the next step is the same scenarios as HIL on a Nucleo board, with DWT-measured timing."

---

## 6. Deep-dive questions (updated answers)

1. **How do you know you meet the deadline?** Per-stage cycle accounting. Here it comes from a cycle model; on target it would come from DWT->CYCCNT. Also an explicit runtime check (the completion counter) and overrun counters reported in telemetry. Be honest: this is not formal worst-case execution-time (WCET) analysis.
2. **Why drop torn windows instead of copying the buffer first?** Copying only moves the deadline: the copy itself must finish before DMA returns. Detection is needed either way. Zero-copy saves 3 KB of RAM and the copy time.
3. **Why your own int8 kernels?** Tiny dense model. TFLM's interpreter costs flash and an arena for little benefit. I kept the TFLite quantization spec so the model could switch to TFLM later.
4. **How did quantization affect accuracy?** Float 94.4%, int8 95.1%, 99.2% agreement on held-out installations. The +0.7% is noise (a net 7 of 1,080 windows; 9 changed class), not a real improvement.
5. **Why split by installation?** Windows within a session are near-duplicates, so a window-level split leaks.
6. **Why debounce, and what does it cost?** Single windows misfire 10% of the time on healthy machines. 6-of-8 costs about 0.9 s of latency, which is irrelevant for a fault developing over days and essential to avoid nuisance alarms.
7. **What is synthetic data hiding?** Mounting resonances, temperature drift, speed variation beyond ±6%, combined faults, EMI. That's why I'd collect real data first on hardware.
8. **How would this become a protection function?** It wouldn't. Protection must be deterministic and fast, and should latch and drive a safe state. A classifier is an advisory layer above it.
9. **Variable-speed motors?** The bands are fixed around 25 Hz. A variable-speed motor needs order tracking: resample against shaft angle using a tachometer input or speed estimate.

---

## 7. Resume bullets that no longer match (flagging, not changing)

- **"STM32 Cortex-M4":** *phrasing fix.* The code targets it, but it ran in simulation. Something like "firmware for an STM32 Cortex-M4, validated in software-in-the-loop simulation".
- **"Deployed a quantized TensorFlow Lite for Microcontrollers (TinyML) model":** *needs a change.* TFLM was not used. Truthful: an int8 model with TFLite-style quantization and custom C kernels.
- **"Using DMA to process 3-axis accelerometer data":** *phrasing fix.* DMA moves the data and the CPU processes it.
- **Last bullet ("Applied data structures, algorithms…"):** *phrasing fix.* It reads like coursework. The silent torn-window story belongs there.

Tell me when you want rewrite drafts.

---

## 8. Before Tuesday

- **Fri:** build, run every scenario, read `acq.c`, `app.c` and `health.c` line by line.
- **Sat:** reproduce stories 1 and 2 yourself (commands in §3). Change a debounce constant and watch `normal_marginal` and the latency change.
- **Sun:** practice the 60-second version and a 5-minute screen-share (README diagram → `acq.c` → timeline plots → `pytest`). Push to GitHub with CI green.
- **Mon:** rehearse §6 out loud.
