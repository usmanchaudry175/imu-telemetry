# IMU Project — Measured Results & CV Claims Log

Running record of every verified, measured result from this project. Only add a claim
here once it's been genuinely tested — this file is the single source of truth for
CV wording, README content, and interview prep. Never state a number here that
hasn't been observed and confirmed.

---

## Gyroscope calibration (Step 9)

**Method:** 1000-sample stationary average per axis, converted to °/s using
GYRO_SENSITIVITY_DEFAULT (131 LSB/(°/s), default ±250°/s range).

**Result — bias values (confirmed repeatable across 3 independent runs):**
| Run | Bias X | Bias Y | Bias Z |
|---|---|---|---|
| 1 | -4.586 | -2.572 | -1.027 |
| 2 | -4.592 | -2.568 | -1.030 |
| 3 | -4.621 | -2.499 | -0.949 |

**Before/after correction (single verification reading):**
| Axis | Raw (°/s) | Corrected (°/s) |
|---|---|---|
| X | -4.702 | -0.081 |
| Y | -2.672 | -0.172 |
| Z | -1.176 | -0.227 |

**Headline claim:** ~95% reduction in gyro static bias across all three axes,
confirmed repeatable across 3 independent 1000-sample calibration runs.

---

## Accelerometer calibration (Step 9, Part B)

**Method:** 6-position calibration (flat/upside-down, and both directions on
each of the other two axes), 50-sample average per position. Full raw data
in `docs/calibration.md`.

**Computed offset/scale:**
| Axis | Offset | Scale |
|---|---|---|
| X | 671.5 | 16281.5 |
| Y | 16 | 16263 |
| Z | -1252 | 16667 |

**Before calibration — magnitude scatter observed:** 0.936g – 1.109g (~±8-9%)

**After calibration — verification (4 independent readings, varied orientations):**
1.006, 0.999, 0.994, 1.013 — tight cluster within ±1.3% of true 1.0g

**Headline claim:** Accelerometer magnitude error reduced from ~±8% (uncalibrated)
to ~±1.3% (calibrated), a ~6-7x improvement, verified across 4 independent readings.

---

## Complementary filter (Step 10)

**Method:** atan2-based tilt angle from accelerometer, blended with integrated
gyro angle using α = 0.98.

**Steady-state test (board flat, stationary):** Pitch settled from an initial
0.47° to a stable 0.49–0.50°, holding flat — no runaway drift observed.

**Drift-resistance test (board held at fixed tilt, ~55 samples):** Held at
approximately -30° tilt by hand. Pitch estimate remained within -30.40° to
-30.27° (0.13° total spread) — no measurable drift over the sampling window,
confirming the accelerometer correction term successfully counteracts gyro
integration drift.

**Headline claim:** Implemented a complementary filter fusing accelerometer
and gyroscope data; verified it holds a fixed orientation within ~0.1° over
a sustained hold, versus unbounded drift expected from gyro integration alone.

**Not yet done:** roll axis (X/Z), ALPHA tuning comparison, Kalman filter
comparison (optional).

---

## CV-relevant confirmed facts (non-project)

- First Year result: 72%
- A-Levels: Chemistry A*, Mathematics A, Physics A
- Siemens Technology Virtual Work Experience (Springpod), completed August 2024,
  confirmed covered Digital Twins and Industry 4.0 by name
- Analogue Signal Conditioning Lab: physically built 1st/2nd-order active
  High-Pass/Low-Pass RC filters, measured via virtual instrumentation bench,
  plotted in Excel (no LTspice used)
- Integrated Design Project 1A/1B: solo redesign of solar garden light casing/
  mounting in Fusion 360, core brief only (no optional advanced elements),
  written report produced
- Current course status: BEng Mechanical Engineering, transfer to Electrical
  and Electronic Engineering submitted and pending (not yet approved)
- Confirmed real project skills to date: C, C++ (Arduino/AVR), MATLAB,
  Fusion 360, Git/GitHub
- NOT yet true — do not add to CV until actually done: Python (planned Steps
  12-13), KiCad circuit design (planned Step 19, documentation-only use),
  CI/GitHub Actions (Step 14), CAN bus (Step 15), static analysis (Step 17)

---

## Placement application tracking

| Company | Status | Deadline |
|---|---|---|
| Mercedes-AMG Petronas F1 (Electronics) | Open | 15 Sep 2026 |
| Aston Martin Aramco F1 (Engineering – Electronics) | Open | 24 Sep 2026 |
| McLaren (Industrial Placement) | Opens 28 Sep | Closes 18 Oct 2026 |
| Rolls-Royce Motor Cars (Bespoke Engineering) | Opens 1 Oct | Closes 30 Nov 2026 (or sooner) |
| Airbus (Electrical Design, Filton) | Open | Rolling |
| JLR (Electronics & Complex Systems) | Not yet open | Opens Jan 2027 |
| Bentley Motors | Closed for this cycle | Reopens ~Nov 2026 |
| Crux Product Design | Uncertain, requires First-class | Verify directly |