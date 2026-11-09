#!/usr/bin/env python3
"""Group independent measured runs; no fabricated sample generation or speedup claim."""
import json
import pathlib
import statistics
import sys
from collections import defaultdict


def render(records):
    groups = defaultdict(list)
    for r in records:
        if r["transport"] in ("dds", "ipc", "shared_memory", "persistent_shared_memory"):
            groups[(int(r["payload_bytes"]), r["transport"])].append(r)
    payloads = sorted({payload for payload, _ in groups})
    lines = [
        "# Observed four-mode RTT results (descriptive only)",
        "",
        "All values below summarize **measured, successful round-trip times**. "
        "They do not establish one-way latency, delivery guarantees, or statistical superiority.",
        "Timeout rate includes replies not received before the deadline (not confirmed packet loss).",
        "Reported CPU/memory, if present, is process-specific and is **not** a matched resource benchmark.",
        "",
        "| Payload (bytes) | Transport | Runs | Completed / Attempts | Timeout rate | "
        "Median of run means (µs) | Min–max run means (µs) | Median run P95 (µs) |",
        "|---:|---|---:|---:|---:|---:|---:|---:|",
    ]
    def fmt(value):
        return f"{value:.2f}" if value is not None else "N/A"
    for payload in payloads:
        for mode in ("dds", "ipc", "shared_memory", "persistent_shared_memory"):
            runs = groups.get((payload, mode), [])
            means = sorted(r["mean_rtt_us"] for r in runs if r["mean_rtt_us"] is not None)
            p95s = sorted(r["p95_rtt_us"] for r in runs if r["p95_rtt_us"] is not None)
            received = sum(r["received"] for r in runs)
            attempts = sum(r["attempted"] for r in runs)
            ratio = (attempts - received) / attempts if attempts else None
            spread = f"{fmt(min(means))}–{fmt(max(means))}" if means else "N/A"
            lines.append(
                f"| {payload} | {mode} | {len(runs)} | {received}/{attempts} | "
                f"{fmt(ratio * 100) + '%' if ratio is not None else 'N/A'} | "
                f"{fmt(statistics.median(means)) if means else 'N/A'} | "
                f"{spread} | {fmt(statistics.median(p95s)) if p95s else 'N/A'} |"
            )
        if any(len(groups.get((payload, mode), [])) < 3 for mode in ("dds", "ipc", "shared_memory", "persistent_shared_memory")):
            lines.append("")
            lines.append(f"Payload {payload}: fewer than three runs for at least one transport; "
                         "insufficient repetition even for a preliminary comparison.")
            lines.append("")
    if not payloads:
        lines.append("| — | — | 0 | 0/0 | N/A | N/A | N/A | N/A |")
        lines.append("")
        lines.append("No valid four-mode RTT runs were found.")
    lines.extend([
        "",
        "**Interpretation:** Medians across runs describe variability but are not confidence "
        "intervals. Different transport framing and callback scheduling remain confounders. "
        "Check raw CSV and environment metadata before making research claims.",
        "",
    ])
    return "\n".join(lines)


def main(args):
    if len(args) != 3:
        raise SystemExit("usage: summarize_comparison.py SUMMARY_JSON OUTPUT_REPORT_MD")
    source = pathlib.Path(args[1])
    target = pathlib.Path(args[2])
    records = json.loads(source.read_text())
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(render(records))
    print("Created", target)


if __name__ == "__main__":
    main(sys.argv)
