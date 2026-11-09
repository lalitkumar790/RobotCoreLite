#!/usr/bin/env python3
"""Validate and summarize cross-process monotonic LaserScan observations.

Same Linux host only. Callback-entry minus pre-publish is *application-visible*
delivery delay, including ROS first hop, bridge work and transport scheduling.
This is NOT an isolated transport latency or a research-grade speedup.
"""
import argparse
import csv
import json
import math
import pathlib
import statistics


FIELDS = ["sequence", "monotonic_ns"]


def read_events(path):
    with pathlib.Path(path).open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != FIELDS:
            raise ValueError(f"{path}: unexpected event header")
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path}: empty event stream")
    result = []
    previous_ns = 0
    for expected_seq, row in enumerate(rows, 1):
        if set(row) != set(FIELDS):
            raise ValueError(f"{path}: malformed event row")
        try:
            seq = int(row["sequence"])
            ns = int(row["monotonic_ns"])
        except (ValueError, TypeError) as error:
            raise ValueError(f"{path}: non-integer event field") from error
        if seq != expected_seq:
            raise ValueError(f"{path}: missing, duplicate or reordered sequence #{expected_seq}")
        if ns <= previous_ns:
            raise ValueError(f"{path}: invalid/nonmonotonic timestamp on {seq}")
        result.append((seq, ns))
        previous_ns = ns
    return result


def percentile_nearest_rank(values, percentile):
    if not values:
        raise ValueError("no samples for percentile")
    ordered = sorted(values)
    return ordered[max(0, math.ceil(len(ordered) * percentile) - 1)]


def analyze(source_path, sink_path, expected, mode, requested_period_ms, output_dir):
    if expected < 1 or requested_period_ms < 1:
        raise ValueError("invalid count or requested period")
    if mode not in ("dds", "dds_bridge", "ipc", "shm", "persistent_shm"):
        raise ValueError("unsupported experiment mode")
    source = read_events(source_path)
    sink = read_events(sink_path)
    if len(source) != expected or len(sink) != expected:
        raise ValueError(f"delivery incomplete: source={len(source)} sink={len(sink)} expected={expected}")
    intervals = []
    delays_us = []
    rows = []
    for i, ((seq, sent_ns), (received_seq, received_ns)) in enumerate(zip(source, sink)):
        if seq != received_seq:
            raise ValueError("source/sink sequence mismatch")
        if received_ns < sent_ns:
            raise ValueError("negative delivery delay: clocks not comparable or malformed samples")
        delay = (received_ns - sent_ns) / 1000.0
        delays_us.append(delay)
        interval = None if i == 0 else (sent_ns - source[i - 1][1]) / 1000.0
        if interval is not None:
            intervals.append(interval)
        rows.append((seq, sent_ns, received_ns, f"{delay:.3f}",
                     "" if interval is None else f"{interval:.3f}"))
    summary = {
        "mode": mode,
        "requested_period_ms": requested_period_ms,
        "source_frames": len(source),
        "sink_frames": len(sink),
        "complete_contiguous_delivery": True,
        "delivery_delay_us_mean": statistics.mean(delays_us),
        "delivery_delay_us_p50": statistics.median(delays_us),
        "delivery_delay_us_p95_nearest_rank": percentile_nearest_rank(delays_us, .95),
        "delivery_delay_us_p99_nearest_rank": percentile_nearest_rank(delays_us, .99),
        "delivery_delay_us_max": max(delays_us),
        "observed_source_interval_us_mean": statistics.mean(intervals) if intervals else None,
        "observed_source_interval_us_p95_nearest_rank": (
            percentile_nearest_rank(intervals, .95) if intervals else None),
        "clock_basis": "Linux same-host std::chrono::steady_clock monotonic epoch",
        "measurement_kind": "pre_publish_to_sink_callback_entry",
        "performance_claim": False,
    }
    path = pathlib.Path(output_dir)
    path.mkdir(parents=True, exist_ok=True)
    with (path / "paired_events.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sequence", "source_monotonic_ns", "sink_monotonic_ns",
                         "observed_delivery_us", "source_interval_us"])
        writer.writerows(rows)
    (path / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    (path / "README.txt").write_text(
        "These are application-visible cross-process monotonic observations on one Linux host.\n"
        "They include ROS first-hop discovery (once established), middleware delivery,\n"
        "bridge serialization, transport and sink callback scheduling, but not full validation.\n"
        "The direct DDS two-process baseline is topologically different; compare only\n"
        "DDS bridge with native transports as a three-process *application* path, noting\n"
        "that DDS wraps the CDR frame in UInt8MultiArray and internal queueing differs.\n"
        "CI samples are functional smoke tests, NOT controlled performance evidence.\n"
    )
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True)
    parser.add_argument("--sink", required=True)
    parser.add_argument("--expected", required=True, type=int)
    parser.add_argument("--mode", required=True)
    parser.add_argument("--period-ms", required=True, type=int)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    result = analyze(args.source, args.sink, args.expected, args.mode,
                     args.period_ms, args.output_dir)
    print(f"validated events={result['sink_frames']} mode={result['mode']}, "
          "no performance claim")


if __name__ == "__main__":
    main()
