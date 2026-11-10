#!/usr/bin/env python3
"""Validate and descriptively summarize copied/loaned SHM run CSVs.

Not a DDS comparison; report distributions per run, not pooled independent frames.
"""
import csv
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
            for row in reader:
                total += 1
                if row["mode"] != mode or row["payload_bytes"] != size:
                    raise ValueError(f"metadata mismatch: {path.name}")
                if row["status"] != "ok":
                    raise ValueError(f"failed sample in {path.name}: {row['status']}")
                rtt = float(row["rtt_us"])
                if not 0 <= rtt < 1e9 or rtt != rtt:
                    raise ValueError(f"invalid latency: {path.name}")
                values.append(rtt)
        if not values:
            raise ValueError(f"empty run: {path.name}")
        groups[(repeat, size, mode)] = (statistics.median(values), max(values), total)
    pairs = {}
    for repeat, size, mode in groups:
        pairs.setdefault((repeat, size), set()).add(mode)
    for key, modes in pairs.items():
        if modes != {"copied", "loaned"}:
            raise ValueError(f"unpaired run: {key}")
        if groups[(*key, "copied")][2] != groups[(*key, "loaned")][2]:
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
        "| Repetition | Payload bytes | Copied median (µs) | Loaned median (µs) | Copied worst (µs) | Loaned worst (µs) |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for repeat, size in sorted(pairs, key=lambda k: (int(k[0][1:]), int(k[1]))):
        copied = groups[(repeat, size, "copied")]
        loaned = groups[(repeat, size, "loaned")]
        lines.append(f"| {repeat[1:]} | {size} | {copied[0]:.3f} | {loaned[0]:.3f} | {copied[1]:.3f} | {loaned[1]:.3f} |")
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
