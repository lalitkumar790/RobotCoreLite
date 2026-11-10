#!/usr/bin/env python3
"""Validate and descriptively summarize copied/loaned SHM run CSVs.

Not a DDS comparison; report distributions per run, not pooled independent frames.
"""
import csv
import math
import pathlib
import statistics
import sys


def summarize(directory):
    root = pathlib.Path(directory)
    paths = sorted(root.glob("r*_*.csv"))
    if not paths:
        raise ValueError("no experiment CSV files")
    groups = {}
    for path in paths:
        fields = path.stem.split("_")
        if len(fields) != 3 or fields[2] not in ("copied", "loaned"):
            raise ValueError(f"unexpected file name: {path.name}")
        repeat, size, mode = fields
        if not repeat.startswith("r") or not repeat[1:].isdigit() or not size.isdigit():
            raise ValueError(f"invalid run identity: {path.name}")
        with path.open(newline="") as fp:
            reader = csv.DictReader(fp)
            if reader.fieldnames != ["mode", "seq", "payload_bytes", "status", "rtt_us"]:
                raise ValueError(f"invalid schema: {path}")
            values = []
            total = 0
            previous_seq = None
            for row in reader:
                total += 1
                seq = int(row["seq"])
                if seq <= 0 or (previous_seq is not None and seq != previous_seq + 1):
                    raise ValueError(f"noncontiguous sequence: {path.name}")
                previous_seq = seq
                if row["mode"] != mode or row["payload_bytes"] != size:
                    raise ValueError(f"metadata mismatch: {path.name}")
                if row["status"] != "ok":
                    raise ValueError(f"failed sample in {path.name}: {row['status']}")
                rtt = float(row["rtt_us"])
                if not math.isfinite(rtt) or not 0 <= rtt < 1e9:
                    raise ValueError(f"invalid latency: {path.name}")
                values.append(rtt)
        if not values:
            raise ValueError(f"empty run: {path.name}")
        ordered = sorted(values)
        p99 = ordered[math.ceil(0.99 * len(ordered)) - 1]
        groups[(repeat, size, mode)] = (statistics.median(values), p99, max(values), total)
    pairs = {}
    for repeat, size, mode in groups:
        pairs.setdefault((repeat, size), set()).add(mode)
    for key, modes in pairs.items():
        if modes != {"copied", "loaned"}:
            raise ValueError(f"unpaired run: {key}")
        if groups[(*key, "copied")][3] != groups[(*key, "loaned")][3]:
            raise ValueError(f"sample counts differ: {key}")
    lines = [
        "# SHM copied-versus-loaned experimental RTT",
        "",
        "**NOT a DDS comparison and NOT evidence of an end-to-end ROS 2 speedup.**",
        "Two OS processes, single in-flight frame and echo response. RTT includes",
        "payload generation, validation, callback scheduling and semaphore activity.",
        "The loaned echo still copies application bytes into its reply slot.",
        "Reported RTTs are steady_clock round trips in microseconds.",
        "Results are descriptive, not statistically conclusive.",
        "",
        "| Run | Bytes | Copied P50 (µs) | Loaned P50 (µs) | Copied P99 (µs) | Loaned P99 (µs) | Copied max (µs) | Loaned max (µs) |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for repeat, size in sorted(pairs, key=lambda k: (int(k[0][1:]), int(k[1]))):
        copied = groups[(repeat, size, "copied")]
        loaned = groups[(repeat, size, "loaned")]
        lines.append(f"| {repeat[1:]} | {size} | {copied[0]:.3f} | {loaned[0]:.3f} | {copied[1]:.3f} | {loaned[1]:.3f} | {copied[2]:.3f} | {loaned[2]:.3f} |")
    lines.extend(["", "Raw CSV and environment.txt are required for interpretation.", ""])
    output = root / "shm_path_report.md"
    output.write_text("\n".join(lines))
    return output


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: summarize_shm_paths.py RUN_DIRECTORY")
    try:
        print(summarize(sys.argv[1]))
    except (OSError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
