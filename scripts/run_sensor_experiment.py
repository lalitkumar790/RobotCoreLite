#!/usr/bin/env python3
"""Orchestrate repeated, randomized same-application-topology LaserScan runs.

Runs only local ROS 2 Jazzy executables via test_ros_sensor_bridge.sh; records
*observed* paired source/sink timestamps and environment details. Does not
execute a synthetic performance simulation or claim any transport speedup.
"""
import argparse
import csv
import datetime as dt
import json
import os
import pathlib
import platform
import random
import shutil
import signal
import statistics
import subprocess
import sys
import time

# Support both unit-test package import and direct CLI invocation from scripts/.
if __package__:
    from .analyze_sensor_events import read_events
else:
    from analyze_sensor_events import read_events


MODES = ("dds_bridge", "ipc", "shm", "persistent_shm")
SCHEMA_VERSION = 1


def plan_rounds(modes, repetitions, seed):
    """Randomized Latin-style rotations: each arm appears once per round.

    For each block of n rounds, each mode appears in every order position.
    The base permutation and direction are randomized per block using seed.
    """
    if len(set(modes)) != len(modes) or not modes:
        raise ValueError("modes must be unique and nonempty")
    if repetitions < 1:
        raise ValueError("at least one repetition required")
    rng = random.Random(seed)
    plan = []
    n = len(modes)
    for block_start in range(0, repetitions, n):
        base = list(modes)
        rng.shuffle(base)
        if rng.choice((False, True)):
            base.reverse()
        for offset in range(min(n, repetitions - block_start)):
            # Rotations cover each position once per full block.
            rotation = base[offset:] + base[:offset]
            plan.append(rotation)
    return plan


def _safe_read(path):
    try:
        return pathlib.Path(path).read_text().strip()
    except OSError:
        return None


def environment_manifest():
    root = pathlib.Path(__file__).resolve().parent.parent
    git = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"],
        capture_output=True, text=True, check=False)
    cpu_governors = {}
    for path in sorted(pathlib.Path("/sys/devices/system/cpu").glob(
            "cpu[0-9]*/cpufreq/scaling_governor")):
        cpu_governors[path.parts[-3]] = _safe_read(path)
    try:
        affinity = sorted(os.sched_getaffinity(0))
    except (OSError, AttributeError):
        affinity = None
    return {
        "captured_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "git_commit": git.stdout.strip() if git.returncode == 0 else None,
        "uname": platform.uname()._asdict(),
        "os_release": _safe_read("/etc/os-release"),
        "python": sys.version,
        "cpu_model": next((line.strip() for line in
                           (_safe_read("/proc/cpuinfo") or "").splitlines()
                           if line.lower().startswith("model name")), None),
        "cpu_governors": cpu_governors,
        "affinity_cpu_ids": affinity,
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "cgroup": _safe_read("/proc/self/cgroup"),
        "note": "Environment snapshot, not full real-time reproducibility.",
    }


def _validated_run(run_directory, mode, count, period_ms):
    expected = ("source_events.csv", "sink_events.csv", "paired_events.csv",
                "summary.json", "environment.txt")
    for basename in expected:
        path = run_directory / basename
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"{mode}: missing/nonempty artifact required: {basename}")
    with (run_directory / "summary.json").open() as source:
        summary = json.load(source)
    if not isinstance(summary, dict):
        raise ValueError(f"{mode}: malformed JSON summary")
    if (summary.get("mode") != mode or
        summary.get("source_frames") != count or
        summary.get("sink_frames") != count or
        summary.get("requested_period_ms") != period_ms or
        summary.get("complete_contiguous_delivery") is not True or
        summary.get("performance_claim") is not False):
        raise ValueError(f"{mode}: inconsistent experiment summary")
    raw_source = read_events(run_directory / "source_events.csv")
    raw_sink = read_events(run_directory / "sink_events.csv")
    if len(raw_source) != count or len(raw_sink) != count:
        raise ValueError(f"{mode}: incomplete raw source/sink event streams")
    delays = []
    with (run_directory / "paired_events.csv").open(newline="") as source:
        reader = csv.DictReader(source)
        expected_columns = ["sequence", "source_monotonic_ns",
                            "sink_monotonic_ns", "observed_delivery_us",
                            "source_interval_us"]
        if reader.fieldnames != expected_columns:
            raise ValueError(f"{mode}: wrong paired-events schema")
        for seq, row in enumerate(reader, 1):
            if int(row["sequence"]) != seq:
                raise ValueError(f"{mode}: invalid paired sequence {seq}")
            source_ns = int(row["source_monotonic_ns"])
            sink_ns = int(row["sink_monotonic_ns"])
            if (seq, source_ns) != raw_source[seq - 1] or (
                seq, sink_ns) != raw_sink[seq - 1]:
                raise ValueError(f"{mode}: raw and paired event mismatch on {seq}")
            delay = float(row["observed_delivery_us"])
            if sink_ns < source_ns or abs(delay - (sink_ns - source_ns)/1000) > .00051:
                raise ValueError(f"{mode}: inconsistent paired event timestamp {seq}")
            delays.append(delay)
    if len(delays) != count:
        raise ValueError(f"{mode}: incomplete paired events")
    if not all(0 <= value < 1e12 for value in delays):
        raise ValueError(f"{mode}: implausible timing sample")
    return {
        "mode": mode,
        "frames": len(delays),
        "run_median_application_visible_us": statistics.median(delays),
        "run_mean_application_visible_us": statistics.mean(delays),
        "run_max_application_visible_us": max(delays),
        "observed_source_interval_us_mean":
            summary.get("observed_source_interval_us_mean"),
    }


def describe_completeness(records, expected_total):
    """Descriptive report only; never rank or claim faster transport."""
    lines = [
        "# RobotCore-Lite controlled-host experiment collection",
        "",
        f"Validated runs: **{len(records)}/{expected_total}**",
        "",
        "This report summarizes application-visible **source pre-publish to sink "
        "callback-entry** observations on one Linux host, not isolated transport latency.",
        "A matched source/bridge/sink topology is used, but the DDS bridge uses "
        "UInt8MultiArray middleware and native transports have different framing, "
        "queueing, CPU costs and synchronization. Persistent SHM also has a manager.",
        "Reported means/medians are descriptive. **No speedup, significance or "
        "real-time guarantee is established** without controlled host repetitions, "
        "uncertainty assessment and confounder inspection.",
        "",
        "| Mode | Valid runs | Per-run medians (µs) |",
        "|---|---:|---|",
    ]
    for mode in MODES:
        rows = [r for r in records if r["mode"] == mode]
        values = ", ".join(f'{r["run_median_application_visible_us"]:.3f}'
                           for r in rows)
        lines.append(f"| {mode} | {len(rows)} | {values or 'N/A'} |")
    lines.extend([
        "",
        "Do not interpret CI smoke measurements as controlled research results.",
        "Raw per-sequence observations, process logs, experiment plan, and "
        "environment manifests are retained in their individual run directories.",
        "",
    ])
    return "\n".join(lines)


def run_experiment(output_root, runner, repetitions, frames, period_ms,
                   seed, timeout_seconds, modes=MODES,
                   profile_resources=False, resource_period_ms=100):
    plan = plan_rounds(modes, repetitions, seed)
    if profile_resources and not 20 <= resource_period_ms <= 5000:
        raise ValueError("resource sampling period must be 20..5000ms")
    output_root = pathlib.Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    output = output_root / (
        dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        + f"-pid{os.getpid()}")
    output.mkdir(exist_ok=False)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "seed": seed,
        "modes": list(modes),
        "repetitions": repetitions,
        "frames_per_run": frames,
        "requested_period_ms": period_ms,
        "timeout_seconds": timeout_seconds,
        "order": plan,
        "environment": environment_manifest(),
        "status": "in_progress",
        "methodology": "No unverified performance claims; generated LaserScan",
        "resource_sampling_enabled": bool(profile_resources),
        "resource_sampling_period_ms": resource_period_ms if profile_resources else None,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    records = []
    expected_total = repetitions * len(modes)
    index = 0
    try:
        for round_index, ordered_modes in enumerate(plan, 1):
            for position, mode in enumerate(ordered_modes, 1):
                index += 1
                dest = output / f"round{round_index:03d}_pos{position}_{mode}"
                dest.mkdir()
                env = os.environ.copy()
                env["SENSOR_EVENTS_DIR"] = str(dest.resolve())
                if profile_resources:
                    env["SENSOR_PROFILE_PROCESSES"] = "1"
                else:
                    env.pop("SENSOR_PROFILE_PROCESSES", None)
                command = ["bash", str(runner), mode, str(frames), str(period_ms)]
                start = time.monotonic()
                status = {"round": round_index, "position": position,
                          "mode": mode, "command": command,
                          "status": "started"}
                (dest / "run_status.json").write_text(json.dumps(status, indent=2) + "\n")
                try:
                    with (dest / "runner.log").open("w") as log:
                        sampler = None
                        if profile_resources:
                            if __package__:
                                from .sample_sensor_resources import ResourceSampler
                            else:
                                from sample_sensor_resources import ResourceSampler
                            sampler = ResourceSampler(dest, mode, resource_period_ms)
                            sampler.start()
                        try:
                            child = subprocess.Popen(
                                command, stdout=log, stderr=subprocess.STDOUT,
                                cwd=pathlib.Path(__file__).resolve().parent.parent,
                                env=env, start_new_session=True)
                            try:
                                code = child.wait(timeout=timeout_seconds)
                            except subprocess.TimeoutExpired:
                                os.killpg(child.pid, signal.SIGTERM)
                                try:
                                    child.wait(timeout=3)
                                except subprocess.TimeoutExpired:
                                    os.killpg(child.pid, signal.SIGKILL)
                                    child.wait()
                                raise RuntimeError(f"{mode}: runner timed out")
                        finally:
                            if sampler is not None:
                                sampler.stop()
                    if code != 0:
                        raise RuntimeError(f"{mode}: runner exited {code} (see {dest}/runner.log)")
                    record = _validated_run(dest, mode, frames, period_ms)
                    record.update(round=round_index, position=position,
                                  duration_wall_seconds=time.monotonic()-start)
                    records.append(record)
                    status.update(status="success", **record)
                except Exception as exc:
                    status["status"] = "failed"
                    status["error"] = str(exc)
                    raise
                finally:
                    (dest / "run_status.json").write_text(
                        json.dumps(status, indent=2) + "\n")
    except BaseException as exc:
        manifest["status"] = "failed"
        manifest["error"] = str(exc)
        raise
    else:
        manifest["status"] = "completed"
    finally:
        manifest["successful_runs"] = len(records)
        manifest["expected_runs"] = expected_total
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        with (output / "run_summary.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=[
                "round", "position", "mode", "frames",
                "run_median_application_visible_us",
                "run_mean_application_visible_us", "run_max_application_visible_us",
                "observed_source_interval_us_mean", "duration_wall_seconds"])
            writer.writeheader()
            writer.writerows(records)
        (output / "report.md").write_text(
            describe_completeness(records, expected_total))
    if manifest["status"] == "completed":
        # Only analyze a complete balanced collection. Fail closed if any
        # raw stream or run summary changes after first-pass validation.
        if __package__:
            from .analyze_sensor_runs import write_analysis
        else:
            from analyze_sensor_runs import write_analysis
        try:
            reference = "dds_bridge" if "dds_bridge" in modes else modes[0]
            write_analysis(output, reference=reference, seed=seed)
        except Exception as exc:
            manifest["status"] = "failed"
            manifest["error"] = f"post-collection validation failed: {exc}"
            (output / "manifest.json").write_text(
                json.dumps(manifest, indent=2) + "\n")
            raise
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repetitions", type=int, default=4)
    parser.add_argument("--frames", type=int, default=250)
    parser.add_argument("--period-ms", type=int, default=10)
    parser.add_argument("--seed", type=int, default=790)
    parser.add_argument("--timeout-seconds", type=int, default=90)
    parser.add_argument("--output-root", default="results/sensor-experiments")
    parser.add_argument("--runner", default="scripts/test_ros_sensor_bridge.sh")
    parser.add_argument("--profile-resources", action="store_true",
                        help="Enable perturbing per-process Linux /proc sampling")
    parser.add_argument("--resource-period-ms", type=int, default=100)
    args = parser.parse_args()
    if not 1 <= args.repetitions <= 100:
        parser.error("repetitions must be between 1 and 100")
    if not 1 <= args.frames <= 1000 or not 1 <= args.period_ms <= 1000:
        parser.error("frames and period-ms must be in [1, 1000]")
    if not 10 <= args.timeout_seconds <= 3600:
        parser.error("timeout-seconds must be between 10 and 3600")
    runner = pathlib.Path(args.runner).resolve()
    if not runner.is_file():
        parser.error(f"runner not found: {runner}")
    if args.profile_resources and not 20 <= args.resource_period_ms <= 5000:
        parser.error("--resource-period-ms must be 20..5000")
    output = run_experiment(args.output_root, runner, args.repetitions,
                            args.frames, args.period_ms, args.seed,
                            args.timeout_seconds,
                            profile_resources=args.profile_resources,
                            resource_period_ms=args.resource_period_ms)
    print("Completed observation collection:", output)


if __name__ == "__main__":
    main()
