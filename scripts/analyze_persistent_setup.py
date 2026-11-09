#!/usr/bin/env python3
"""Strict descriptive analysis of *observed* persistent manager setup samples.

This does not assess ROS 2 RTT, latency, throughput or a zero-copy path.
"""
import csv
import math
import pathlib
import statistics
import sys

COLD = "cold_create_and_first_epoch"
WARM = "warm_reset_existing_region"


def parse_samples(path):
    with pathlib.Path(path).open(newline="") as source:
        reader = csv.DictReader(source)
        expected = {"iteration", "phase", "capacity_bytes", "duration_us", "epoch"}
        if set(reader.fieldnames or []) != expected:
            raise ValueError("unexpected setup CSV schema")
        rows = list(reader)
    if not rows:
        raise ValueError("no setup measurements")
    pairs = {}
    capacities = set()
    for row in rows:
        iteration = int(row["iteration"])
        capacity = int(row["capacity_bytes"])
        phase = row["phase"]
        epoch = int(row["epoch"])
        elapsed = float(row["duration_us"])
        if iteration < 1 or capacity < 1 or not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("invalid setup observation")
        if (phase, epoch) not in ((COLD, 1), (WARM, 2)):
            raise ValueError("invalid phase/epoch pair")
        capacities.add(capacity)
        pair = pairs.setdefault(iteration, {})
        if phase in pair:
            raise ValueError("duplicate setup measurement")
        pair[phase] = elapsed
    if len(capacities) != 1:
        raise ValueError("mixed capacities are not comparable in one report")
    if set(pairs) != set(range(1, len(pairs) + 1)):
        raise ValueError("missing or unexpected iteration IDs")
    if any(set(pair) != {COLD, WARM} for pair in pairs.values()):
        raise ValueError("incomplete cold/warm pair")
    return pairs, capacities.pop()


def report(pairs, capacity):
    cold = [pairs[i][COLD] for i in sorted(pairs)]
    warm = [pairs[i][WARM] for i in sorted(pairs)]
    def describe(data):
        return f"{statistics.mean(data):.3f} | {statistics.median(data):.3f} | {min(data):.3f} | {max(data):.3f}"
    return "\n".join([
        "# Persistent shared-memory manager setup experiment",
        "",
        f"**Measured setup pairs:** {len(pairs)} · **Capacity:** {capacity} bytes",
        "",
        "| Operation | Mean (µs) | Median (µs) | Min (µs) | Max (µs) |",
        "|---|---:|---:|---:|---:|",
        f"| Cold create + initial session | {describe(cold)} |",
        f"| Warm reset of existing region | {describe(warm)} |",
        "",
        "These are **different operations**, not interchangeable transport message paths.",
        "The cold operation includes POSIX mapping creation, initialization and the first epoch;",
        "warm includes only an epoch reset of the same region. Both omit endpoint",
        "startup and message delivery. Measurements are descriptive, not a statistical",
        "proof that a robotics workload is faster. No RTT or one-way latency was measured.",
        "Control machine state, repeat across independent runs, inspect raw data and",
        "include CPU/memory and endpoint lifecycle experiments before making claims.",
        "",
    ])


def main(argv):
    if len(argv) != 3:
        raise SystemExit("usage: analyze_persistent_setup.py INPUT.csv REPORT.md")
    pairs, capacity = parse_samples(argv[1])
    dest = pathlib.Path(argv[2])
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(report(pairs, capacity) + "\n")
    print("Created", dest)


if __name__ == "__main__":
    main(sys.argv)
