# Accelerometer Calibration — 6-Position Method

Raw readings captured with `captureAccelPosition()`, 50-sample average per position.
Board held still for each orientation for ~1-2 seconds during capture.

## Raw readings

Sensitivity used for g conversion: 16384 LSB/g (default ±2g range)

| Position | X (raw) | X (g) | Y (raw) | Y (g) | Z (raw) | Z (g) |
|---|---|---|---|---|---|---|
| Flat (Z up) | 495 | 0.030 | 170 | 0.010 | 15415 | 0.941 |
| Upside down (Z down) | 914 | 0.056 | 119 | 0.007 | -17919 | -1.094 |
| Side, X up | 16953 | 1.035 | -285 | -0.017 | -1844 | -0.113 |
| Side, X down | -15610 | -0.953 | 2520 | 0.154 | -1886 | -0.115 |
| Side, Y up | -1860 | -0.114 | 16279 | 0.994 | -1681 | -0.103 |
| Side, Y down | -408 | -0.025 | -16247 | -0.992 | -1979 | -0.121 |

## Computed offset and scale

Formula: `offset = (max + min) / 2`, `scale = (max - min) / 2`

| Axis | Max (raw) | Min (raw) | Offset | Scale |
|---|---|---|---|---|
| X | 16953 | -15610 | 671.5 | 16281.5 |
| Y | 16279 | -16247 | 16 | 16263 |
| Z | 15415 | -17919 | -1252 | 16667 |

## Notes

- Gyro bias calibration (separate, already done): X: -4.6°/s → -0.08°/s, Y: -2.7°/s → -0.17°/s, Z: -1.2°/s → -0.23°/s (~95% reduction, confirmed repeatable across 3 runs)
- Accel magnitude scatter observed pre-calibration: 0.93g–1.11g across various readings — expected to tighten once offset/scale correction applied
- **Post-calibration result: magnitude = 1.006g** (vs 0.936–1.109g uncalibrated scatter) — offset/scale correction confirmed working
- **Verification (4 independent readings, varied orientations):** 1.006, 0.999, 0.994, 1.013 — tight cluster within ±1.3% of true 1.0g, vs ~±8% uncalibrated scatter (~6-7x error reduction)