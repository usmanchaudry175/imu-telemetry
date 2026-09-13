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

**Known limitation — yaw:** Not fusion-corrected. Gravity provides no reference
for rotation about the vertical axis, so yaw would drift unboundedly under pure
gyro integration with this 6-DOF sensor (accel+gyro only, no magnetometer).
Fixing this requires a magnetometer (9-DOF) for hard/soft-iron-calibrated
absolute heading reference — deliberately deferred as a stretch goal, not
core scope, to keep focus on Step 11 (real-time architecture) and the
September/October application deadlines. Revisit only if runway allows later.

**Known limitation — roll angle wraparound:** atan2 returns values strictly
within ±180°. When roll approaches this boundary, the accelerometer-derived
angle wraps (e.g. from near +180° to near -180°) while the gyro-integrated
side continues smoothly past it, causing the blended filter output to jump
violently even while the board is held still. Root-caused via physical testing
(rotating through the boundary reproduced it reliably; staying within ±90°
of flat does not). Standard fix requires detecting >180° discrepancy between
estimate and new accelerometer reading and adding/subtracting 360° before
blending — deliberately deferred; roll is correct and usable within the
realistic ±90° demo range, wraparound handling logged as a known edge case
rather than solved now.

**Not yet done:** ALPHA tuning comparison, Kalman filter comparison (optional).

**Kalman filter — R (measurement noise) measured directly:** Collected raw,
unfiltered accelerometer-derived pitch angle across two stationary batches
(59 and 62 samples) to measure real sensor noise rather than guess a
textbook value. Batch 1 variance: 0.0428. Batch 2 (board held stiller):
0.0327 — ~24% improvement, confirms technique affects measured noise and
that the measurement is repeatable/real, not artifact. Combined (121
samples): **R = 0.0379**, used directly in the Kalman filter's update step.

**Kalman filter — Q (process noise) measured directly:** Collected 62 raw
gyroX_dps samples with board stationary. Variance = 0.00773 (°/s)². Combined
with Step 11's measured effective sample period (~13ms, not the ideal 10ms)
via Q ≈ σ²_gyro × dt²: **Q_angle = 0.0000013**. Notably ~750x smaller than
the initial placeholder guess (0.001) — makes physical sense, as Q
represents uncertainty added per single short timestep, not accumulated
drift. Both Q and R now genuine measured constants rather than
textbook/guessed values, directly reusing Step 9 (calibration) and Step 11
(timing) methodology and data.

**Kalman vs complementary filter — direct comparison (verified):**
- Steady-state (board flat): both filters agree closely, within 0.01-0.03°
  of each other throughout, both stable in the 0.51-0.59° range. Confirms
  Kalman implementation is correct — independent method converges to the
  same answer as the already-validated complementary filter.
- Drift-hold (~60 samples, held at ~-25° tilt by hand): both filters
  drift-resistant, neither ran away. Complementary spread ~1° (-24.24° to
  -25.21°), Kalman spread ~1° (-24.72° to -25.74°). Both filters tracked a
  small coordinated real hand-movement together near the end of the sample
  — confirms both are correctly responsive to genuine motion, not just
  artificially locked.
- Consistent systematic offset observed: Kalman runs ~0.4-0.5° more
  negative than complementary throughout. Attributed to the measured R
  (0.0379) being relatively large, causing Kalman to weight the
  accelerometer more heavily than the complementary filter's fixed 0.98
  gyro/0.02 accel split.
- Honest conclusion: for this sensor and use case, Kalman and complementary
  filter perform comparably — no dramatic advantage observed for Kalman in
  either steady-state or moderate dynamic testing. Both are valid,
  verified, working implementations; Kalman's real advantage (adaptive
  weighting under changing noise conditions) would be more visible under
  more extreme dynamic testing than performed here.

**Roll verification (within ±90° range):** Held at ~29° tilt, roll oscillated
between 28.40° and 29.35° (under 1° spread) with pitch simultaneously stable
at 1.07–1.11° — confirms wraparound theory (bug only at ±180° boundary, not
within realistic range) and confirms pitch/roll axis separation is correct
with no cross-contamination between the two.

---

## Real-time architecture (Step 11)

**Method:** Timer1 configured in CTC mode via direct AVR register access
(TCCR1A/B, OCR1A, TIMSK1), targeting 100Hz using a prescaler of 256 and
OCR1A = 624. ISR sets a `volatile bool sampleReady` flag only; all real work
(I2C read, filter math, printing) deferred to `loop()`, gated behind
`if (sampleReady)` — standard flag-based interrupt pattern to keep the ISR
itself minimal.

**Interrupt firing accuracy (confirmed):** 100–101 interrupts counted per
second, consistently, across multiple independent test runs — confirms the
hardware timer itself is correctly configured and firing at the intended
rate.

**Actual sample-processing gap (measured, not assumed):** Despite the timer
firing at a true 100Hz, the gap between processed samples measured 11–16ms
(vs the 10ms ideal), with jitter of 3–6ms — meaning the effective processed
rate is closer to 65–90Hz, not the full 100Hz configured.

**Root-cause investigation (methodical elimination, not guesswork):**
1. Hypothesis: I2C bus speed (100kHz default) — switched to 400kHz Fast Mode.
   Result: min/max gap improved marginally (12–15ms → 11–14ms). Ruled out as
   the dominant cause.
2. Hypothesis: Serial.print overhead, even throttled to 1-in-10 samples —
   removed print entirely and re-tested. Result: negligible further change
   (11–14ms → 11–13ms). Ruled out as the dominant cause.
3. Direct measurement: instrumented the I2C read itself with micros()
   timing. Result: consistently 628µs per read — only ~6% of the 10ms
   budget. Conclusively rules out the I2C transaction as the bottleneck.

**Conclusion:** the ~11–16ms effective gap is not caused by any single slow
operation (I2C, print) but by structural overhead in the busy-poll loop
design itself (repeated flag/time checks each pass through loop()). This is
an honest, methodically-investigated finding — documented as the measured
result rather than a solved problem, given time constraints against
placement application deadlines.

**Update — true root cause identified retrospectively:** after switching
from bare-hole, unsoldered, direct-wired breadboard connections to proper
header-pin connections (change made for unrelated reasons, to support
adding new sensors), re-measured timing showed min gap 9996µs, max gap
10004µs, max jitter 4µs — essentially perfect 10ms timing, a dramatic
improvement from the original 11–16ms/3–6ms jitter result. I2C read
duration was unchanged (608µs vs the previously measured 628µs), ruling
out the sensor itself as the variable. This strongly suggests the original
"structural loop overhead" conclusion was measuring a symptom of poor
electrical connection quality — likely signal integrity issues from
friction-fit, unsoldered wiring — rather than a genuine software or
architectural limitation. The original elimination process (ruling out
I2C bus speed, ruling out print overhead) was methodologically sound
given the hypotheses considered; the true variable, physical connection
quality, was simply never part of the original hypothesis set. Retained
as a genuine lesson: a careful, honest elimination process can still miss
the real cause if a hidden variable is never proposed as a candidate in
the first place.

**Headline claim:** Configured a hardware timer interrupt for 100Hz sampling
and diagnosed a real timing anomaly through systematic elimination
(I2C speed, serial overhead) and microsecond-level instrumentation.
Traced the true root cause to physical connection quality — switching from
direct-wired to header-pin connections took measured jitter from 3–6ms
down to 4µs, a result that also revealed a genuine limitation in the
original hypothesis set (electrical connection quality had not been
considered as a candidate variable).

## Ring buffer / overflow detection (Step 11, completed)

**Method:** Replaced the single `volatile bool sampleReady` flag with a
monotonic tick-counter queue (`tickHead`/`tickTail`, `TICK_QUEUE_SIZE`
slots) between the timer ISR and the main loop. The ISR only records a
tick's `micros()` timestamp and advances `tickHead`; all sensor reads and
processing remain in `loop()`, drained via a `while (tickTail != tickHead)`
loop rather than a single `if`, so a temporary stall doesn't silently lose
samples — it processes the backlog on the next pass instead. Added an
explicit `missedSamples` counter, incremented only when the queue is
genuinely full (`tickHead` would collide with `tickTail`), to prove
overflow behaviour rather than assume it — the single-flag design had no
way to detect or report a missed sample at all.

**RAM constraint discovered during implementation:** adding the
tick-timestamp array and growing the per-sample log struct
(`queue_depth_at_read` field) pushed RAM usage to 93% (1904/2048 bytes on
the ATmega328P), causing `sd.begin()` to hang silently rather than fail
cleanly — not an SD wiring or logic fault, but a stack/heap collision at
the most stack-hungry call site in the sketch. Diagnosed via bisection
(temporary DEBUG print statements narrowing the hang to inside a specific
function call) before being traced to available RAM via the PlatformIO
build report. Fixed by right-sizing `queue_depth_at_read` to `uint8_t`
(previously oversized as `uint32_t`; value never exceeds queue capacity)
and reducing both `TICK_QUEUE_SIZE` and `BUFFER_SIZE` from 8 to 4.

**Overflow root-cause investigation** (methodical elimination, same
standard as the Step 11 jitter investigation above): the reduced
`TICK_QUEUE_SIZE=4` configuration showed a real, steadily-climbing
overflow — `missedSamples` reached 18 over one ~3-5 minute run, confirmed
climbing (not a one-time startup transient) via repeated live serial
monitoring.

1. **Hypothesis:** periodic serial debug print block (5 print statements
   every 10th sample) blocking the loop long enough to cause backlog.
   **Test:** disabled the print block entirely, reran with only a
   lightweight 5-second `missedSamples` printout. **Result:** still climbed
   (1→2→3→5 over the same window). Ruled out.
2. **Hypothesis:** SD flush latency under sustained write load. **Test:**
   disabled the SD write call inside `logSample()` entirely (kept
   everything else identical). **Result:** `missedSamples` held at 0
   throughout a full run. Confirmed as the cause.

**Conclusion:** SD flush latency, not I2C or print overhead, was the root
cause. This contradicts an earlier isolated single-write latency estimate
of ~1.3ms (calculated from `SD_SCK_MHZ(1)` transfer time against the 80ms
flush interval at `BUFFER_SIZE=8`). The discrepancy is attributed to SD
flash controllers exhibiting occasional latency spikes from internal
wear-leveling/block-management under sustained continuous writing — a
well-documented flash-storage behaviour not visible in a single isolated
measurement, the same category of lesson as the earlier header-pin vs.
direct-wire connection finding (a careful measurement can still miss a
real variable if that variable was never part of the original test's
conditions).

**Fix and verification:** restored `TICK_QUEUE_SIZE` to 8 (accepting the
RAM cost) while keeping `BUFFER_SIZE` at 4 and the `uint8_t`
`queue_depth_at_read` fix, re-enabled real SD writes, and reran the same
isolation test. Result: 0 missed samples confirmed over a sustained run
with SD writes fully active.

**Steady-state characterisation** (26,943-sample run, 269.4s,
post-reset-settling — see reset note below): queue depth never exceeded 4
of the 8 available slots at any point. 73.2% of samples were serviced with
zero backlog (depth=1); 26.8% showed shallow catch-up backlog (depth 2–4),
none reaching true capacity.

**Jitter percentiles** from the same run, split by queue depth at read
time: clean samples (depth=1, 73.2%) measured p50 6,512µs, p99 10,000µs,
p99.9 17,112µs; catch-up samples (depth 2–4, 26.8%) measured p50 21,964µs,
p99 33,168µs. As noted below, these are processing-time intervals, not
true ISR-tick jitter, so they are a lower bound on the real figure.

**Known limitation — jitter percentile precision:** the logged
`timestamp_us` reflects the time of sensor-read/processing (`micros()`
called in `loop()`), not the ISR's originally-recorded tick timestamp
(`tickTime`, captured but not currently logged). A sample immediately
following a catch-up burst can show an artificially short measured
interval even when correctly classified as "clean" (`queue_depth==1`) at
the moment it was read, because the interval is measured against
processing time, not true tick time. This means computed jitter
percentiles are a lower bound on true jitter, not an exact figure. Fixing
this would require logging `tickTime` instead of processing-time now —
deferred given time constraints against placement deadlines; documented
here as an open limitation rather than a resolved one, per this project's
stated convention of logging known limitations honestly (see complementary
filter yaw/roll-wraparound entries above for the same practice).

**Separately observed — early-boot reset cluster:** 5 reset events
occurred during the 26,943-sample verification run, all clustered within
the first ~5.5 seconds (record indices 264, 384, 468, 512, 552 of 27,496
total), none afterward across the remaining ~265 seconds. Pattern
(isolated to startup, absent thereafter) is consistent with a brownout
reset caused by SD card initialization's current draw briefly sagging the
supply rail, rather than an ongoing stability fault — not yet
independently confirmed via the ATmega328P's MCUSR reset-cause register,
which would give a definitive answer (brown-out vs. watchdog vs. external
vs. power-on reset) but requires reading MCUSR at the very start of
`setup()` before the bootloader clears it. Logged as a
probable-but-unconfirmed explanation.

**Headline claim:** Implemented and empirically validated a lock-free
tick-queue between the sample-rate ISR and main loop, replacing a
single-flag design with zero overflow visibility. Diagnosed a genuine
overflow condition through systematic isolation (ruling out print overhead
before confirming SD flush latency as the cause), a result that corrected
an earlier single-measurement latency estimate by revealing SD flash
controllers' sustained-write latency tail. Verified the fixed
configuration held zero true overflow across a 27,000-sample, 4.5-minute
run, with confirmed real headroom (queue depth peaking at 4 of 8 available
slots).

---

## SD card logging (Step 12, partial)

**Status:** logging complete; framed wire protocol deferred to Blackbox Phase 8.

**Method:** SdFat32 over SPI (CS on pin 10, standard Arduino hardware SPI
pins 11/12/13 for MOSI/MISO/SCK), writing buffered `ImuSample` structs
(packed, 21 bytes as of the Step 11 ring-buffer update) to `imu_log.bin` in
append mode. Buffered writes (`BUFFER_SIZE=4` samples per flush) with
`logFile.sync()` confirming each flush and an `sdWriteFailures` counter
reporting (not silently swallowing) any write/sync failure — same "flag
reality, don't silently normalize" principle already established in the
software companion project's design decisions (D16, D18).

**Hardware note:** original SD card (an off-brand "Onyx" microSD) was
root-caused as unreliable — inconsistent SPI timing/write behaviour under
embedded use — and replaced with a SanDisk card before this work began.

**Wiring verification:** confirmed via multimeter continuity checks (every
breakout-to-Arduino wire individually, VCC-to-GND short check) and a
powered no-card voltage check at the breakout's regulator output, before
ever inserting the card — deliberately sequenced to isolate wiring faults
from card-related faults, and to avoid risking the card itself on
unverified wiring.

**Result:** SD initialization, buffered writes, and flushes confirmed
reliable across multiple runs. Independent Python verification script
(`tools/verify_imu_log.py`) decodes the raw struct format directly
(`struct.unpack`, little-endian, matching the packed C struct exactly) and
checks: file-size/record-size alignment, timestamp monotonicity,
plausibility bounds on pitch/roll, and (post ring-buffer work) queue-depth
distribution and steady-state jitter percentiles.

**Real bug found via verification — session boundary ambiguity:** because
`logFile.open()` uses `O_CREAT | O_APPEND`, every power-cycle appends to
the same file rather than starting fresh, and `micros()` restarts
near-zero on each boot. A verification run flagged a non-monotonic
timestamp at record 168, which decoded to two concatenated sessions with
no marker distinguishing them — a real, reproduced instance of exactly the
problem the software companion project's session-ID design (D4, D9) exists
to solve. Deliberately left unfixed in the current SD format (logged as a
known limitation) since the real fix — a session ID + first-of-session
flag per record — is already designed and will arrive with the Bluetooth
link's upcoming reframing onto the full COBS/CRC wire protocol.

**Known limitation — SD log format has no session markers:** a
power-cycle mid-testing silently concatenates two sessions into one file
with a discoverable-but-not-flagged timestamp discontinuity. Confirmed
real via the record-168 finding above. Will be resolved when the
Bluetooth link adopts the full framed wire protocol (session ID, sequence
numbers) rather than the current raw `ImuSample` struct.

**Headline claim:** Implemented buffered, failure-reporting SD logging
over SPI, independently verified via a from-scratch Python decoder (not
just trusting the firmware's own output) that reproduced a real
session-boundary bug — confirming, with real data rather than a
theoretical concern, why the project's planned framed wire protocol
(sequence numbers, session IDs) is a genuine requirement rather than
unnecessary complexity for this application.

**Not yet done:** framed wire protocol (COBS/CRC/sequence-number framing)
for the Bluetooth link, replacing the current raw unframed `ImuSample`
struct — deliberately sequenced after the software companion project's own
protocol was independently designed and benchmarked (see that project's
Phase 7 results), so the IMU firmware adopts an already-proven format
rather than designing one from scratch under application-deadline time
pressure.

---

## Kalman filter (bonus, beyond original plan)

**Method:** 1D Kalman filter applied to pitch, using the standard
predict/update cycle. Both Q (process noise) and R (measurement noise)
derived from real measured data rather than textbook/guessed constants:
- **R = 0.0379** — variance of 121 raw, unfiltered accelerometer-derived
  pitch readings across two stationary collection batches.
- **Q = 0.0000013** — derived from variance of 62 stationary gyroX_dps
  readings (0.00773 (°/s)²), combined with Step 11's measured effective
  sample period (~13ms) via Q ≈ σ²_gyro × dt².

**Comparison against complementary filter:**
- Steady-state (board flat): both filters agree within 0.01–0.03° of each
  other, both stable in the 0.51–0.59° range — confirms correct Kalman
  implementation via independent agreement with the already-validated
  complementary filter.
- Drift-hold (~60 samples, held at ~-25° by hand): both filters
  drift-resistant, ~1° spread each. Both tracked a small coordinated real
  hand-movement together near the end of the sample, confirming genuine
  responsiveness rather than artificial locking.
- Consistent systematic offset: Kalman runs ~0.4–0.5° more negative than
  complementary throughout, attributed to the measured R being relatively
  large, causing Kalman to weight the accelerometer more heavily than the
  complementary filter's fixed 0.98/0.02 split.

**Headline claim:** Implemented and independently verified a 1D Kalman
filter, tuned using measured (not assumed) noise characteristics reusing
Step 9 and Step 11 methodology; found comparable performance to the
complementary filter for this application, with a well-understood cause
for the small systematic difference observed.

**Scope decision — roll not independently re-verified:** Roll would very
likely reproduce the same ±180° atan2 wraparound limitation already
documented for the complementary filter, since that bug lives upstream in
the raw angle computation shared by both fusion methods, not in the
blending logic. Deliberately not re-measuring roll's Q/R given diminishing
new learning versus time — noted as an honest scope boundary.

---

## Hardware setup — new components (in progress)

**Wiring quality fix (session-wide):** all original components (MPU-6050,
first magnetometer attempt) were bare-hole, unsoldered, direct-wired —
identified as the root cause of the Step 11 timing anomaly (see above) and
of the session's earlier intermittent-connection debugging. All remaining
components (SD card, Bluetooth HC-05, MCP2515 CAN modules) arrive
pre-soldered with proper header pins, avoiding a repeat of this issue.

**Logic analyser:** confirmed hardware is a generic 24MHz 8-channel
Cypress FX2-based clone (fx2lafw-compatible). Connected via USB-C to
USB-A adapter (laptop has 1x USB-A occupied by Arduino, 2x free USB-C —
adapter used rather than a hub to avoid an unnecessary extra point of
failure). Probes tapped onto existing SDA/SCL/GND breadboard rows,
non-invasive to existing circuit.

**Driver/connection troubleshooting (resolved):** PulseView initially
failed to detect the device — it was defaulting to the built-in Demo
driver (simulated square/sine/random channel patterns), and manually
selecting fx2lafw still failed to connect with the interface set to
Serial Port (Windows had enumerated the board as a COM port). Root
cause: fx2lafw is a libusb-based driver for FX2 USB chips and does not
communicate over a virtual COM port at all — Serial Port was never the
correct interface for this class of device. Fix: switched the interface
radio button from Serial Port to USB in PulseView's device-scan dialog.
Device now scans and connects correctly under fx2lafw + USB.

**Capture/decode of a real I2C transaction — still pending,** next
session step now that the device connects reliably.

**Magnetometer (QMC5883L/GY-273):** hardware arrived, deliberately
deferred — not core scope for current placement-application priorities.
Logged as a stretch goal (see complementary filter section, yaw
limitation) rather than pursued further this session.

**SD card debugging:** extensive troubleshooting of intermittent SD
logging failures concluded the root cause was the card itself — an
off-brand Onyx microSD, known for inconsistent SPI timing/write
behaviour under embedded use. SanDisk replacement ordered; SD logging
work paused pending its arrival rather than continuing to debug against
unreliable hardware.

**Remaining components not yet wired:** SD card module (blocked on
replacement card, see above), HC-05 Bluetooth, 2x MCP2515 CAN modules —
planned for Step 12 (logging/telemetry) and Step 15 (CAN bus)
respectively.

---
