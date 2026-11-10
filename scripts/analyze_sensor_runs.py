#!/usr/bin/env python3
"""Validate replicated LaserScan collection and describe run-level uncertainty.

Unit of resampling is one *round* (one run per transport), NEVER a sensor frame.
This is exploratory descriptive evidence; it does not establish speedups or
research-grade statistical confidence, especially with a small number of rounds.
"""
import argparse
import csv
import json
import math
import pathlib
import random
import statistics

RUN_HEADER = [
    "round", "position", "mode", "frames",
    "run_median_application_visible_us", "run_mean_application_visible_us",
    "run_max_application_visible_us", "observed_source_interval_us_mean",
    "duration_wall_seconds",
]
DEFAULT_REFERENCE = "dds_bridge"


def quantile(values, probability):
    ordered = sorted(values)
    if not ordered or not 0 <= probability <= 1:
        raise ValueError("invalid percentile input")
    location = (len(ordered) - 1) * probability
    low, high = math.floor(location), math.ceil(location)
    return ordered[low] + (ordered[high] - ordered[low]) * (location - low)


def _valid_nonnegative(value, label):
    try:
        result = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"non-numeric {label}") from exc
    if not math.isfinite(result) or result < 0:
        raise ValueError(f"invalid/nonfinite {label}")
    return result


def load_completed_collection(directory):
    """Fail closed for failed, incomplete, duplicated, or inconsistent trials."""
    directory = pathlib.Path(directory)
    manifest = json.loads((directory / "manifest.json").read_text())
    if manifest.get("schema_version") != 1 or manifest.get("status") != "completed":
        raise ValueError("only completed schema-v1 collections can be analyzed")
    modes = manifest.get("modes")
    plan = manifest.get("order")
    rounds = manifest.get("repetitions")
    frames = manifest.get("frames_per_run")
    if (not isinstance(modes, list) or not modes or
            len(set(modes)) != len(modes) or
            not isinstance(rounds, int) or rounds < 1 or
            not isinstance(frames, int) or frames < 1 or
            not isinstance(plan, list) or len(plan) != rounds or
            any(not isinstance(order, list) or
                len(order) != len(modes) or set(order) != set(modes)
                for order in plan)):
        raise ValueError("malformed collection plan")
    total = rounds * len(modes)
    if (manifest.get("successful_runs") != total or
            manifest.get("expected_runs") != total):
        raise ValueError("collection manifest has missing/failed runs")

    path = directory / "run_summary.csv"
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != RUN_HEADER:
            raise ValueError("incorrect per-run CSV header")
        entries = list(reader)
    if len(entries) != total:
        raise ValueError("collection is missing per-run summaries")

    by_round = {}
    for row in entries:
        try:
            round_id = int(row["round"])
            position = int(row["position"])
            frame_count = int(row["frames"])
        except (ValueError, TypeError) as exc:
            raise ValueError("non-integer round/position/frame count") from exc
        if not 1 <= round_id <= rounds or not 1 <= position <= len(modes):
            raise ValueError("invalid round or position")
        mode = row["mode"]
        if mode != plan[round_id - 1][position - 1] or frame_count != frames:
            raise ValueError("run does not match randomized order/count")
        key = (round_id, mode)
        if key in by_round:
            raise ValueError("duplicate mode in round")
        for field in ("run_median_application_visible_us",
                      "run_mean_application_visible_us",
                      "run_max_application_visible_us",
                      "duration_wall_seconds"):
            _valid_nonnegative(row[field], field)
        if row["observed_source_interval_us_mean"] != "":
            _valid_nonnegative(
                row["observed_source_interval_us_mean"],
                "observed_source_interval_us_mean")
        if (float(row["run_max_application_visible_us"]) <
                float(row["run_median_application_visible_us"])):
            raise ValueError("max delivery delay is less than median")

        run_dir = directory / f"round{round_id:03d}_pos{position}_{mode}"
        status = json.loads((run_dir / "run_status.json").read_text())
        if (status.get("status") != "success" or
                status.get("mode") != mode or
                status.get("round") != round_id or
                status.get("position") != position or
                status.get("frames") != frames):
            raise ValueError(f"run status mismatch: {run_dir}")
        # Re-read raw event streams, not merely the summary produced by a
        # previous tool. A summary alone cannot prove message integrity.
        if __package__:
            from .run_sensor_experiment import _validated_run
        else:
            from run_sensor_experiment import _validated_run
        verified = _validated_run(run_dir, mode, frames,
                                  manifest["requested_period_ms"])
        for field in ("run_median_application_visible_us",
                      "run_mean_application_visible_us",
                      "run_max_application_visible_us"):
            if not math.isclose(float(row[field]), verified[field],
                                abs_tol=1e-9, rel_tol=1e-12):
                raise ValueError(f"summary/raw mismatch for {key}: {field}")
        by_round[key] = float(row["run_median_application_visible_us"])

    if len(by_round) != total:
        raise ValueError("incomplete round-by-mode table")
    return manifest, by_round


def paired_bootstrap(values, seed, resamples=4000):
    """Return mean paired difference and exploratory 95% resampling interval."""
    if len(values) < 4:
        return None
    if resamples < 100:
        raise ValueError("too few resamples")
    rng = random.Random(seed)
    count = len(values)
    draws = [
        sum(values[rng.randrange(count)] for _ in range(count)) / count
        for _ in range(resamples)
    ]
    return {
        "mean_difference_us": statistics.mean(values),
        "resampling_interval_95_us": [
            quantile(draws, .025), quantile(draws, .975)
        ],
        "resampling_unit": "complete collection round (paired transport runs)",
        "exploratory_only": True,
        "bootstrap_draws": resamples,
        "rounds": count,
    }


def analyze_collection(directory, reference=DEFAULT_REFERENCE,
                       seed=790, resamples=4000):
    manifest, lookup = load_completed_collection(directory)
    modes = manifest["modes"]
    rounds = manifest["repetitions"]
    if reference not in modes:
        raise ValueError(f"reference transport absent from collection: {reference}")
    result = {
        "analysis_method": "paired-round bootstrap of per-run median differences",
        "reference": reference,
        "rounds": rounds,
        "frames_per_run": manifest["frames_per_run"],
        "requested_period_ms": manifest["requested_period_ms"],
        "seed": seed,
        "controlled_host_evidence": False,
        "performance_claim": False,
        "warning": "Exploratory resampling, not proof of speedup or independent-frame uncertainty.",
        "transport_summary": {},
        "paired_differences": {},
    }
    ref_medians = [lookup[(i, reference)] for i in range(1, rounds + 1)]
    for mode in modes:
        medians = [lookup[(i, mode)] for i in range(1, rounds + 1)]
        result["transport_summary"][mode] = {
            "run_medians_us": medians,
            "median_of_run_medians_us": statistics.median(medians),
            "run_count": rounds,
        }
        if mode == reference:
            continue
        diffs = [x - y for x, y in zip(medians, ref_medians)]
        result["paired_differences"][mode] = {
            "per_round_median_difference_us": diffs,
            "exploratory_resampling": paired_bootstrap(diffs, seed, resamples),
            "insufficient_runs_for_resampling": len(diffs) < 4,
        }
    return result


def render_report(report):
    lines = [
        "# Exploratory run-level LaserScan uncertainty analysis",
        "",
        f"Reference: **{report['reference']}**; "
        f"**{report['rounds']} full paired rounds**; "
        f"**{report['frames_per_run']} frames per run**.",
        "",
        "All differences below are **per-run median application-visible delay "
        "minus the reference run median**, paired within the same collection round. "
        "The unit of replication is the **run/round**, not an individual frame.",
        "DDS bridge, Unix IPC, bounded SHM and persistent SHM have different "
        "internal queues/framing, and the persistent mode uses a manager.",
        "The source and receiver use one Linux clock domain, but this is not "
        "isolated transport latency or a real-robot experiment.",
        "",
        "| Mode | Runs | Median of run medians (µs) |",
        "|---|---:|---:|",
    ]
    for mode, values in report["transport_summary"].items():
        lines.append(
            f"| {mode} | {values['run_count']} | "
            f"{values['median_of_run_medians_us']:.3f} |")
    lines.extend([
        "",
        "## Differences from the DDS/CDR bridge, paired by round",
        "",
        "| Mode | Mean paired difference (µs) | "
        "Exploratory 95% round-resampling interval (µs) |",
        "|---|---:|---|",
    ])
    for mode, values in report["paired_differences"].items():
        interval = values["exploratory_resampling"]
        if interval is None:
            lines.append(f"| {mode} | N/A | N/A (fewer than 4 paired rounds) |")
        else:
            bounds = interval["resampling_interval_95_us"]
            lines.append(
                f"| {mode} | {interval['mean_difference_us']:+.3f} | "
                f"[{bounds[0]:+.3f}, {bounds[1]:+.3f}] |")
    lines.extend([
        "",
        "**Interpretation:** These resampling intervals describe variation between "
        "collected runs conditional on this host, workload and test design. "
        "They are **not** formal evidence of performance superiority or "
        "a guaranteed confidence level, especially for four rounds. "
        "No p-values, rankings or speedup claims are generated.",
        "Do not include failed runs in this analysis: failed or missing trials "
        "invalidate the collection and must be reported separately.",
        "Run repeated collections on a controlled reference host, "
        "report CPU/RSS, actual cadence and operating conditions, and "
        "replicate across independent blocks/days before drawing conclusions.",
        "",
    ])
    return "\n".join(lines)


def write_analysis(directory, reference=DEFAULT_REFERENCE,
                   seed=790, resamples=4000):
    directory = pathlib.Path(directory)
    analysis = analyze_collection(directory, reference, seed, resamples)
    (directory / "run_uncertainty.json").write_text(
        json.dumps(analysis, indent=2, sort_keys=True) + "\n")
    (directory / "run_uncertainty.md").write_text(render_report(analysis))
    return analysis


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("collection")
    parser.add_argument("--reference", default=DEFAULT_REFERENCE)
    parser.add_argument("--seed", type=int, default=790)
    parser.add_argument("--resamples", type=int, default=4000)
    args = parser.parse_args()
    if not 100 <= args.resamples <= 100000:
        parser.error("resamples must be 100..100000")
    report = write_analysis(args.collection, args.reference,
                            args.seed, args.resamples)
    print("Analyzed", report["rounds"], "paired rounds; NO performance claims.")


if __name__ == "__main__":
    main()
