#!/usr/bin/env python3
"""
Verifies imu_log.bin against the ring-buffer-decoupled ImuSample
struct:

struct __attribute__((packed)) ImuSample {
  uint32_t timestamp_us;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t pitch_x100;
  int16_t roll_x100;
  uint8_t queue_depth_at_read;
};

Packed: 4 + (2*8) + 1 = 21 bytes per record.

Jitter/percentile analysis excludes records up through the last
detected session-reset break, so early-boot transients (e.g. brownout
resets during SD init) don't skew steady-state numbers. Session
resets are still reported in full, separately, for the record.
"""

import argparse
import struct
import sys

RECORD_FORMAT = "<IhhhhhhhhB"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)
assert RECORD_SIZE == 21, f"expected 21 bytes, got {RECORD_SIZE} — struct packing mismatch"


def percentile(sorted_values, pct):
    if not sorted_values:
        return None
    idx = int(len(sorted_values) * pct)
    idx = min(idx, len(sorted_values) - 1)
    return sorted_values[idx]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_path", help="Path to imu_log.bin")
    args = parser.parse_args()

    with open(args.log_path, "rb") as f:
        data = f.read()

    total_bytes = len(data)
    full_records = total_bytes // RECORD_SIZE
    trailing_bytes = total_bytes % RECORD_SIZE

    print(f"File size: {total_bytes} bytes")
    print(f"Record size: {RECORD_SIZE} bytes")
    print(f"Full records: {full_records}")
    if trailing_bytes:
        print(f"NOTE: {trailing_bytes} trailing bytes do not form a complete "
              f"record — likely a mid-write reset/power-off, not necessarily an error.")

    records = []
    for i in range(full_records):
        chunk = data[i * RECORD_SIZE:(i + 1) * RECORD_SIZE]
        timestamp_us, ax, ay, az, gx, gy, gz, pitch_x100, roll_x100, queue_depth = struct.unpack(RECORD_FORMAT, chunk)
        records.append({
            "timestamp_us": timestamp_us,
            "accel": (ax, ay, az),
            "gyro": (gx, gy, gz),
            "pitch": pitch_x100 / 100.0,
            "roll": roll_x100 / 100.0,
            "queue_depth": queue_depth,
        })

    if not records:
        print("No records found.")
        sys.exit(1)

    # --- Timestamp monotonicity / session-reset detection ---
    non_monotonic = 0
    wraps = 0
    session_breaks = []
    for i in range(1, len(records)):
        prev = records[i - 1]["timestamp_us"]
        curr = records[i]["timestamp_us"]
        if curr < prev:
            if prev > 0xFFFFFFFF - 1_000_000 and curr < 1_000_000:
                wraps += 1
            else:
                non_monotonic += 1
                if prev - curr > 100_000:
                    session_breaks.append(i)

    implausible = sum(1 for r in records if abs(r["pitch"]) > 180 or abs(r["roll"]) > 180)

    max_depth_seen = max(r["queue_depth"] for r in records)
    depth_histogram = {}
    for r in records:
        depth_histogram[r["queue_depth"]] = depth_histogram.get(r["queue_depth"], 0) + 1

    # --- Steady-state window: exclude everything through the last session break ---
    steady_state_start = max(session_breaks) + 1 if session_breaks else 0
    excluded_count = steady_state_start
    steady_state_duration_s = 0
    if steady_state_start < len(records):
        steady_state_duration_s = (records[-1]["timestamp_us"] - records[steady_state_start]["timestamp_us"]) / 1_000_000.0

    clean_intervals = []
    catchup_intervals = []
    for i in range(max(1, steady_state_start), len(records)):
        prev_ts = records[i - 1]["timestamp_us"]
        curr_ts = records[i]["timestamp_us"]
        dt = curr_ts - prev_ts
        if dt <= 0:
            continue  # still skip any residual reset/wrap points inside the window
        if records[i]["queue_depth"] > 1:
            catchup_intervals.append(dt)
        else:
            clean_intervals.append(dt)

    clean_intervals.sort()
    catchup_intervals.sort()
    all_intervals = sorted(clean_intervals + catchup_intervals)

    print(f"\nFirst record:  ts={records[0]['timestamp_us']}us  "
          f"accel={records[0]['accel']}  gyro={records[0]['gyro']}  "
          f"pitch={records[0]['pitch']:.2f}  roll={records[0]['roll']:.2f}  "
          f"queue_depth={records[0]['queue_depth']}")
    print(f"Last record:   ts={records[-1]['timestamp_us']}us  "
          f"accel={records[-1]['accel']}  gyro={records[-1]['gyro']}  "
          f"pitch={records[-1]['pitch']:.2f}  roll={records[-1]['roll']:.2f}  "
          f"queue_depth={records[-1]['queue_depth']}")

    print(f"\nTimestamp wraparounds (uint32, expected at ~71 min): {wraps}")
    print(f"Non-monotonic timestamps: {non_monotonic}")
    if session_breaks:
        print(f"  -> {len(session_breaks)} look like session resets "
              f"(>100ms backward jump), at record indices: {session_breaks}")
        print(f"  -> Excluding records 0-{steady_state_start-1} ({excluded_count} records) "
              f"from jitter analysis as early-boot transients.")
        print(f"  -> Steady-state window: {len(records) - steady_state_start} records "
              f"over {steady_state_duration_s:.1f}s")
    print(f"Implausible pitch/roll values (>180 deg): {implausible}")

    print(f"\n--- Queue depth distribution (full run, all records) ---")
    print(f"Max depth seen: {max_depth_seen} (queue capacity: 8, per TICK_QUEUE_SIZE)")
    for depth in sorted(depth_histogram.keys()):
        pct = 100 * depth_histogram[depth] / len(records)
        label = "clean" if depth <= 1 else "catch-up"
        print(f"  depth={depth} ({label}): {depth_histogram[depth]} samples ({pct:.3f}%)")

    print(f"\n--- Jitter analysis (STEADY STATE ONLY, post-reset-settling) ---")
    print(f"Clean samples (queue_depth<=1): {len(clean_intervals)}")
    print(f"Catch-up samples (queue_depth>1): {len(catchup_intervals)}  "
          f"({100*len(catchup_intervals)/max(1,len(all_intervals)):.3f}% of steady-state intervals)")

    if clean_intervals:
        print(f"\nClean-sample interval percentiles (target: 10000 us / 100 Hz):")
        print(f"  p50:   {percentile(clean_intervals, 0.50)} us")
        print(f"  p99:   {percentile(clean_intervals, 0.99)} us")
        print(f"  p99.9: {percentile(clean_intervals, 0.999)} us")
        print(f"  min:   {clean_intervals[0]} us   max: {clean_intervals[-1]} us")

    if catchup_intervals:
        print(f"\nCatch-up-sample interval percentiles:")
        print(f"  p50:   {percentile(catchup_intervals, 0.50)} us")
        print(f"  p99:   {percentile(catchup_intervals, 0.99)} us")
        print(f"  min:   {catchup_intervals[0]} us   max: {catchup_intervals[-1]} us")
    else:
        print(f"\nNo catch-up samples in the steady-state window.")

    print(f"\nIMPORTANT: cross-reference against the firmware's own serial-printed "
          f"missedSamples value for the definitive true-overflow-proof claim — "
          f"a dropped tick (queue completely full) leaves NO trace in this log "
          f"by definition, since it was never recorded at all.")

    if non_monotonic == 0 and implausible == 0:
        print("\nVERIFICATION PASSED: all records decode cleanly and pass plausibility checks.")
    else:
        print("\nVERIFICATION FLAGGED (see counts above) — session resets noted but "
              "not treated as failures; steady-state analysis above excludes them.")


if __name__ == "__main__":
    main()