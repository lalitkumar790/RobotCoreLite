#!/usr/bin/env python3
"""Summarize observed /proc samples with explicit insufficient-data warnings.

These metrics cover the shared initiator-active sampling window, including DDS
discovery and warmup. They are not isolated steady-state RTT resource costs.
"""
import csv
import math
import pathlib
import sys
from collections import defaultdict


HEADER = ["elapsed_ms", "role", "pid", "start_ticks",
          "cpu_user_seconds", "cpu_system_seconds", "rss_kib"]
OUTPUT_HEADER = ["run_id", "role", "pid", "samples", "observed_seconds",
                 "sampled_cpu_core_equivalents", "sampled_max_rss_kib", "status"]


def read_profile(path):
    grouped = defaultdict(list)
    with pathlib.Path(path).open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != HEADER:
            raise ValueError(f"Unexpected resource sample CSV header: {path}")
        for row in reader:
            role = row["role"]
            if role not in ("initiator", "responder", "manager"):
                raise ValueError(f"Unknown role {role} in {path}")
            pid = int(row["pid"])
            ticks = int(row["start_ticks"])
            elapsed = float(row["elapsed_ms"]) / 1000
            user = float(row["cpu_user_seconds"])
            system = float(row["cpu_system_seconds"])
            rss = int(row["rss_kib"])
            if pid < 1 or ticks < 1 or rss < 0 or any(
                    not math.isfinite(x) or x < 0 for x in (elapsed, user, system)):
                raise ValueError(f"Invalid /proc sample: {path}")
            grouped[role].append((elapsed, user, system, rss, pid, ticks))
    for role, samples in grouped.items():
        for prior, next_ in zip(samples, samples[1:]):
            if (next_[0] <= prior[0] or next_[1] < prior[1] or
                    next_[2] < prior[2] or next_[4:6] != prior[4:6]):
                raise ValueError(f"Nonmonotonic readings or PID reuse in {path} ({role})")
    return grouped


def summarize_profile(path):
    group = read_profile(path)
    run_id = pathlib.Path(path).name.removesuffix(".resources.csv")
    expected = ["initiator", "responder"]
    if "_persistent_shared_memory_" in run_id:
        expected.append("manager")
    records = []
    for role in expected:
        observations = group.get(role, [])
        pid = observations[0][4] if observations else ""
        samples = len(observations)
        status = "sampled" if samples >= 2 else "insufficient_samples"
        if status == "sampled":
            first, last = observations[0], observations[-1]
            seconds = last[0] - first[0]
            if seconds <= 0:
                raise ValueError("zero observation window")
            usage = ((last[1] - first[1]) + (last[2] - first[2])) / seconds
            elapsed_str = f"{seconds:.3f}"
            usage_str = f"{usage:.4f}"
        else:
            elapsed_str, usage_str = "", ""
        peak_rss = max((s[3] for s in observations), default=None)
        records.append([run_id, role, pid, samples, elapsed_str, usage_str,
                        peak_rss if peak_rss is not None else "", status])
    return records


def summarize_directory(raw_dir, report_dir):
    paths = sorted(pathlib.Path(raw_dir).glob("*.resources.csv"))
    if not paths:
        raise ValueError("No resource CSVs found")
    rows = []
    for path in paths:
        rows.extend(summarize_profile(path))
    output = pathlib.Path(report_dir)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "resources.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(OUTPUT_HEADER)
        writer.writerows(rows)
    lines = [
        "# Process resource observations (descriptive, not performance claims)", "",
        "Process CPU is the change in user+system CPU seconds divided by the",
        "**observed** elapsed time (core equivalents, not a CPU percentage).",
        "RSS is the maximum **sampled** resident memory, not an exact peak.",
        "The same sampling interval covers initiator execution, DDS discovery,",
        "warmup and recorded attempts; responder/manager startup occurred before",
        "sampling. Idle work and startup differences can confound comparisons.",
        "Sampling also adds CPU overhead. Rows with fewer than two samples have",
        "no estimated CPU rate.", "",
        "| Run | Role | Samples | Observed (s) | CPU cores (sampled) | Sampled max RSS (KiB) | Status |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for run, role, pid, count, duration, cores, rss, status in rows:
        lines.append(f"| {run} | {role} | {count} | {duration or 'N/A'} | "
                     f"{cores or 'N/A'} | {rss if rss != '' else 'N/A'} | {status} |")
    lines.extend(["", "No research-grade CPU/memory advantage is implied.",
                  "Pair this file with raw RTT CSVs and the environment manifest.", ""])
    (output / "resources.md").write_text("\n".join(lines))
    return rows


def main(args):
    if len(args) != 3:
        raise SystemExit("usage: summarize_resources.py RAW_DIRECTORY REPORT_DIRECTORY")
    rows = summarize_directory(args[1], args[2])
    print(f"Reported {len(rows)} process-role samples")


if __name__ == "__main__":
    main(sys.argv)
