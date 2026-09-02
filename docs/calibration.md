# Accelerometer Calibration — 6-Position Method

Raw readings captured with `captureAccelPosition()`, 50-sample average per position.
Board held still for each orientation for ~1-2 seconds during capture.

## Raw readings

Sensitivity used for g conversion: 16384 LSB/g (default ±2g range)

| Position | X (raw) | X (g) | Y (raw) | Y (g) | Z (raw) | Z (g) |
|---|---|---|---|---|---|---|
| Flat (Z up) | 495 | 0.030 | 170 | 0.010 | 15415 | 0.941 |
| Upside down (Z down) | | | | | | |
| Side, X up | | | | | | |
| Side, X down | | | | | | |
| Side, Y up | | | | | | |
| Side, Y down | | | | | | |

## Computed offset and scale

Formula: `offset = (max + min) / 2`, `scale = (max - min) / 2`

| Axis | Max (raw) | Min (raw) | Offset | Scale |
|---|---|---|---|---|
| X | | | | |
| Y | | | | |
| Z | | | | |

## Notes

- Gyro bias calibration (separate, already done): X: -4.6°/s → -0.08°/s, Y: -2.7°/s → -0.17°/s, Z: -1.2°/s → -0.23°/s (~95% reduction, confirmed repeatable across 3 runs)
- Accel magnitude scatter observed pre-calibration: 0.93g–1.11g across various readings — expected to tighten once offset/scale correction applied