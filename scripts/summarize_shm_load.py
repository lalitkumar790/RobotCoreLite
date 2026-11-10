#!/usr/bin/env python3
"""Strict validation and descriptive per-run summaries for SHM load observations.

No CI speedup assertions. Latency is source-intent -> callback entry on one
Linux kernel, NOT isolated transport propagation or DDS-equivalent timing.
"""
import csv
import math
import pathlib
import statistics
import sys


def quantile(values, fraction):
    s = sorted(values)
    return s[math.ceil(fraction * len(s)) - 1]


def summarize(directory, frames):
    root = pathlib.Path(directory)
    sender_files = sorted(root.glob("r*_*.sender.csv"))
    if not sender_files:
        raise ValueError("no sender CSV files")
    groups = {}
    rows = []
    for sender_path in sender_files:
        tag = sender_path.name.removesuffix(".sender.csv")
        parts = tag.split("_")
        if len(parts) != 4 or not parts[0].startswith("r") or not parts[1].startswith("b") or not parts[2].startswith("d") or parts[3] not in ("copied", "loaned"):
            raise ValueError(f"invalid experiment filename: {tag}")
        if not all(p[1:].isdigit() for p in parts[:3]):
            raise ValueError(f"invalid experiment dimensions: {tag}")
        run, payload, delay, mode = parts
        group = (run, payload, delay)
        if mode in groups.setdefault(group, set()):
            raise ValueError(f"duplicate mode: {tag}")
        groups[group].add(mode)
        receiver_path = root / (tag + ".receiver.csv")
        with sender_path.open(newline="") as src, receiver_path.open(newline="") as dst:
            send = csv.DictReader(src)
            receive = csv.DictReader(dst)
            if send.fieldnames != ["mode", "seq", "payload_bytes", "source_monotonic_ns", "publish_duration_us", "status"]:
                raise ValueError(f"wrong sender CSV schema: {tag}")
            if receive.fieldnames != ["mode", "seq", "source_monotonic_ns", "callback_monotonic_ns", "delivery_us", "valid"]:
                raise ValueError(f"wrong receiver CSV schema: {tag}")
            sent = list(send)
            received = list(receive)
        if len(sent) != frames or len(received) != frames:
            raise ValueError(f"incomplete frame sequence: {tag} {len(sent)}/{len(received)}/{frames}")
        publish_us = []
        delivery_us = []
        for i, (src, dst) in enumerate(zip(sent, received), 1):
            if (src["mode"] != mode or dst["mode"] != mode or
                src["seq"] != str(i) or dst["seq"] != str(i) or
                src["payload_bytes"] != payload[1:] or
                src["status"] != "ok" or dst["valid"] != "true"):
                raise ValueError(f"invalid sequence/status/metadata: {tag} seq={i}")
            origin = int(src["source_monotonic_ns"])
            if origin != int(dst["source_monotonic_ns"]):
                raise ValueError(f"source clock pairing mismatch: {tag} seq={i}")
            arrived = int(dst["callback_monotonic_ns"])
            measured = (arrived-origin)/1000.0
            reported = float(dst["delivery_us"])
            duration = float(src["publish_duration_us"])
            if (origin <= 0 or arrived < origin or
                not all(math.isfinite(x) for x in (reported, duration, measured)) or
                reported < 0 or duration < 0 or
                abs(measured-reported) > 0.02):
                raise ValueError(f"invalid timing: {tag} seq={i}")
            publish_us.append(duration)
            delivery_us.append(measured)
        rows.append((run, payload[1:], delay[1:], mode,
                     statistics.median(publish_us), quantile(publish_us, .99),
                     max(publish_us), statistics.median(delivery_us),
                     quantile(delivery_us, .99), max(delivery_us)))
    for group, modes in groups.items():
        if modes != {"copied", "loaned"}:
            raise ValueError(f"unpaired experiment: {group}")
    lines = [
        "# RobotCore-Lite SHM sustained-load characterization",
        "",
        "**NOT Fast DDS-equivalent performance evidence; CI results are functional only.**",
        "Source-intent-to-receiver callback-entry measurements on one Linux host.",
        "Sender publish duration includes waiting for a free shared-memory slot.",
        "Receiver-side optional callback delay produces deliberate backpressure.",
        "Single publisher, single subscriber, depth=1, no warmup, as-fast-as-possible source.",
        "The sender starts its timestamp before per-message payload work; copied and",
        "loaned paths do not yet match all allocation, production and checksum costs.",
        "No rate-based closed-loop control or verified hardware real-time bound.",
        "",
        "| Run | Bytes | Receiver delay (µs) | Mode | Publish P50 (µs) | Publish P99 (µs) | Publish max (µs) | Delivery P50 (µs) | Delivery P99 (µs) | Delivery max (µs) |",
        "|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in sorted(rows, key=lambda r: (int(r[0][1:]), int(r[1]), int(r[2]), r[3])):
        run, payload, delay, mode, *measurements = row
        lines.append("| " + " | ".join([run[1:], payload, delay, mode] +
                                     [f"{x:.3f}" for x in measurements]) + " |")
    lines.extend(["", "All frames must be contiguous, byte-valid and clock-paired.", "Raw producer/receiver CSV and environment.txt must accompany any result.", ""])
    output = root / "load_report.md"
    output.write_text("\n".join(lines))
    return output


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: summarize_shm_load.py OUTPUT_DIRECTORY FRAMES")
    try:
        n = int(sys.argv[2])
        if n < 1:
            raise ValueError("FRAMES must be positive")
        print(summarize(sys.argv[1], n))
    except (ValueError, OSError) as exc:
        raise SystemExit(str(exc)) from exc
