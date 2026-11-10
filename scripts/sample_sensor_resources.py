#!/usr/bin/env python3
"""Opt-in /proc CPU and RSS sampling for ROS LaserScan collection processes.

This is a supplementary, perturbing Linux-host observation. Process startup,
discovery, teardown and workloads need not occupy identical observation windows.
Never treat sampled RSS as exact peak memory, or core equivalents as CPU %.
"""
import csv
import json
import pathlib
import threading
import time

try:
    from .profile_processes import read_proc
except ImportError:
    from profile_processes import read_proc

HEADER = ["elapsed_ms", "role", "pid", "start_ticks",
          "cpu_user_seconds", "cpu_system_seconds", "rss_kib"]
ROLES = ("source", "bridge", "sink", "manager")


def read_roles(path):
    path = pathlib.Path(path)
    if not path.exists():
        return {}
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        # A concurrently created file can briefly exist before the header
        # is written. Retry at the next sampling tick rather than fail.
        if reader.fieldnames is None:
            return {}
        if reader.fieldnames != ["role", "pid"]:
            raise ValueError("wrong process ID schema")
        result = {}
        for row in reader:
            role = row["role"]
            pid = int(row["pid"])
            if role not in ROLES or role in result or pid < 1:
                raise ValueError("invalid or duplicated process role")
            result[role] = pid
        return result


def summarize_samples(rows, expected_roles):
    grouped = {role: [] for role in expected_roles}
    for row in rows:
        if row["role"] in grouped:
            grouped[row["role"]].append(row)
    result = {}
    for role, samples in grouped.items():
        sample_count = len(samples)
        rss = max((row["rss_kib"] for row in samples), default=None)
        summary = {
            "samples": sample_count,
            "sampled_max_rss_kib": rss,
            "cpu_core_equivalents": None,
            "observed_seconds": None,
            "status": "insufficient_samples",
        }
        if sample_count >= 2:
            start, end = samples[0], samples[-1]
            seconds = (end["elapsed_ms"] - start["elapsed_ms"]) / 1000
            cpu = ((end["cpu_user_seconds"] - start["cpu_user_seconds"]) +
                   (end["cpu_system_seconds"] - start["cpu_system_seconds"]))
            if seconds <= 0 or cpu < -1e-6 or (
                    start["start_ticks"], start["pid"]) != (
                    end["start_ticks"], end["pid"]):
                raise ValueError("invalid or reused PID in resource samples")
            summary.update(status="sampled", observed_seconds=seconds,
                           cpu_core_equivalents=max(cpu, 0) / seconds)
        result[role] = summary
    return result


class ResourceSampler:
    def __init__(self, directory, mode, interval_ms=100):
        if mode not in ("dds_bridge", "ipc", "shm", "persistent_shm"):
            raise ValueError("unknown sampled transport mode")
        if not 20 <= interval_ms <= 5000:
            raise ValueError("sampling period must be 20..5000ms")
        self.directory = pathlib.Path(directory)
        self.mode = mode
        self.interval_ms = interval_ms
        self.roles = ("source", "bridge", "sink") + (
            ("manager",) if mode == "persistent_shm" else ())
        self._stop = threading.Event()
        self._thread = None
        self._error = None
        self._records = []

    def start(self):
        if self._thread is not None:
            raise RuntimeError("sampler already started")
        self._thread = threading.Thread(
            target=self._loop, name="robotcore-resource-sampler", daemon=True)
        self._thread.start()

    def _loop(self):
        try:
            known_identity = {}
            first_ns = time.monotonic_ns()
            while True:
                stamp = (time.monotonic_ns() - first_ns) / 1e6
                for role, pid in read_roles(self.directory / "process_ids.csv").items():
                    if role not in self.roles:
                        raise ValueError(f"unexpected process role {role}")
                    snapshot = read_proc(pid)
                    if snapshot is None:
                        continue
                    start_ticks, user, system, rss = snapshot
                    identity = (pid, start_ticks)
                    if role in known_identity and known_identity[role] != identity:
                        raise ValueError(f"PID identity changed for {role}")
                    known_identity[role] = identity
                    self._records.append({
                        "elapsed_ms": stamp, "role": role, "pid": pid,
                        "start_ticks": start_ticks,
                        "cpu_user_seconds": user, "cpu_system_seconds": system,
                        "rss_kib": rss,
                    })
                if self._stop.wait(self.interval_ms / 1000):
                    break
        except Exception as exc:
            self._error = exc

    def stop(self):
        if self._thread is None:
            raise RuntimeError("sampler not started")
        self._stop.set()
        self._thread.join(timeout=10)
        if self._thread.is_alive():
            raise RuntimeError("resource sampler thread did not finish")
        if self._error:
            raise RuntimeError("resource sampler failed") from self._error
        raw = self.directory / "resource_samples.csv"
        with raw.open("w", newline="") as source:
            writer = csv.DictWriter(source, fieldnames=HEADER)
            writer.writeheader()
            for row in self._records:
                writer.writerow({
                    key: f"{value:.6f}" if key in (
                        "elapsed_ms", "cpu_user_seconds", "cpu_system_seconds")
                    else value
                    for key, value in row.items()
                })
        summary = {
            "mode": self.mode,
            "sample_period_ms": self.interval_ms,
            "process_roles": summarize_samples(self._records, self.roles),
            "measurement_kind": "perturbing_proc_samples_whole_trial",
            "performance_claim": False,
            "warnings": [
                "Sampling includes process startup, discovery, publication, and teardown.",
                "Roles have unequal observation durations; comparisons are not isolated message CPU.",
                "CPU core equivalents are cumulative CPU seconds / observed seconds, not percentages.",
                "Maximum sampled RSS is not exact peak RSS; missing/short roles have no CPU estimate.",
                "Sampler CPU overhead can perturb event timings.",
            ],
        }
        (self.directory / "resource_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n")
        return summary
