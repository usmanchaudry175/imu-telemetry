# IMU Sensor Fusion & Telemetry Logger

## Demo

*(coming in Step 13 — GIF of the 3D visualiser tracking live board orientation)*

## What this is

An embedded system that reads a MEMS accelerometer and gyroscope over I2C, fuses the
readings into a stable orientation estimate, and streams it to a live 3D visualiser.
Built as a bare-metal firmware project — no OS, no dynamic allocation — with a custom
I2C driver, interrupt-driven sampling, and a self-implemented complementary/Kalman
fusion filter.

## System overview

*(block diagram — Step 19)*

Sensor → MCU → filter → ring buffer → SD / CAN / serial → host visualiser

## Hardware

*(wiring diagram and parts list — Step 19)*

## Design decisions

Running log — added as each decision is made, not written retrospectively.

- *(example: "Chose burst read over per-register reads to cut I2C transaction
  overhead")*

## Results

Filled in as each phase produces a number.

| Metric | Before | After |
|---|---|---|
| Static yaw drift | | |
| Sample jitter (p50 / p99 / p99.9) | | |
| Binary vs JSON log size | | |

## Limitations and next steps

Running log — added the moment something is deliberately left unsolved.

- *(example: "No magnetometer — yaw will drift over long runs; documented, not fixed")*

## Build and run

*(instructions — Step 19)*