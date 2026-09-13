# IMU Sensor Fusion & Telemetry Logger

A bare-metal embedded system (ATmega328P / Arduino UNO, no OS, no dynamic
allocation) that reads a MEMS accelerometer and gyroscope over I2C, fuses the
readings into a stable orientation estimate, and logs the result — with a
custom I2C driver, interrupt-driven sampling, a lock-free tick-queue between
the ISR and main loop, and self-implemented complementary and Kalman fusion
filters.

Full reasoning, raw data, and honestly-logged limitations for every result
below live in [`docs/project-log.md`](docs/project-log.md) and
[`docs/calibration.md`](docs/calibration.md) — this README summarises;
those files are the source of truth.

## System overview

MPU-6050 (I2C) → Timer1 ISR + tick-queue → main loop (calibration-corrected
fusion filter) → SD card (binary log) and Bluetooth (HC-05, downsampled
telemetry).

## Design decisions

Running log in [`docs/project-log.md`](docs/project-log.md), added as each
decision is made. Highlights:

- **Burst read over per-register reads** for accel+gyro (14 bytes in one I2C
  transaction) to cut per-sample transaction overhead.
- **Tick-queue over a single sample-ready flag** between the ISR and main
  loop — a single flag has no way to detect or report a missed sample; the
  queue does, via an explicit `missedSamples` counter.
- **Q/R measured from real sensor data, not guessed**, for the Kalman filter
  — see Results below. `Q_angle` came out ~750x smaller than the initial
  placeholder guess once measured properly.
- **Known limitations documented, not hidden or silently patched** — see
  below. Two real bugs (an ~11–16ms timing anomaly, and a 93%-RAM overflow)
  were root-caused through systematic elimination rather than guessed at;
  both write-ups are in the project log in full.

## Results

All figures below are measured and independently verified — see
`docs/project-log.md` for method, raw data, and repeat-run confirmation.

| Metric | Before | After |
|---|---|---|
| Accelerometer magnitude error | ~±8% (0.936g–1.109g scatter) | ~±1.3% (4-reading verification) |
| Gyroscope static bias | up to −4.6°/s | ~95% reduction, confirmed across 3 runs |
| Complementary filter drift (held tilt) | unbounded (raw gyro integration) | within ~0.1–1° over a sustained hold |
| Kalman filter drift (held tilt) | unbounded (raw gyro integration) | within ~1° over a sustained hold, agrees with complementary filter to 0.01–0.03° at steady state |
| Sample jitter — clean reads (depth-1, 73.2% of samples) | — | p50 6,512µs / p99 10,000µs / p99.9 17,112µs |
| Sample jitter — catch-up reads (depth 2–4, 26.8% of samples) | — | p50 21,964µs / p99 33,168µs |
| Ring buffer overflow (27,000-sample, 4.5-minute run) | undetectable (single-flag design) | 0 true overflows; max depth 4 of 8 slots observed |
| Binary vs JSON log size | | *not yet benchmarked — deferred, see Blackbox project for the equivalent wire-format comparison* |

## Known limitations

Documented honestly as found, per this project's convention — see the
project log for full detail on each:

- **Yaw is not fusion-corrected.** No magnetometer on this 6-DOF sensor;
  yaw drifts unboundedly under pure gyro integration. Deliberately deferred
  (hardware for a 9-DOF upgrade has arrived but is not yet integrated).
- **Roll has a wraparound bug at the ±180° boundary** (`atan2` range limit)
  — causes a visible jump in blended output right at that boundary. Correct
  and usable within the realistic ±90° demo range; the fix (detect >180°
  discrepancy and add/subtract 360° before blending) is known but not yet
  implemented.
- **Jitter percentiles are a lower bound, not exact.** Logged timestamps
  reflect processing time, not the ISR's original tick time — see the
  project log for why this only ever *understates* jitter, never overstates
  it.
- **SD log has no session markers.** A power-cycle mid-test silently
  concatenates two sessions into one file (a real instance of this was
  caught at record 168 via the independent Python verification script).
  Fix is designed (session ID + first-of-session flag) and will land with
  the Bluetooth link's move to the full framed wire protocol.
- **Bluetooth link is implemented but not yet on the framed wire protocol**
  — currently sends the raw `ImuSample` struct rather than the
  COBS/CRC/sequence-numbered format already designed and benchmarked in the
  [Blackbox](https://github.com/usmanchaudry175/blackbox) companion
  project.

## Build and run

```bash
# PlatformIO (VS Code extension or CLI)
pio run                    # build
pio run --target upload    # flash to the board
pio device monitor         # serial monitor, 115200 baud
```

Requires an Arduino UNO, an MPU-6050 breakout, and (for full functionality)
an SD breakout and an HC-05 Bluetooth module.

## Status

Core sensor pipeline complete: I2C driver, calibration, complementary and
Kalman fusion, interrupt-driven sampling with a lock-free tick-queue, and
SD/Bluetooth logging — all measured and documented above.

Planned next: a host-side 3D visualiser, CAN bus output, moving the
Bluetooth link onto the full framed wire protocol (see Known limitations),
and a wiring diagram.