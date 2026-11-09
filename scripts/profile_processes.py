#!/usr/bin/env python3
"""Sample initiator, responder, and optional region manager over one aligned window.

Linux /proc resource samples are functional instrumentation, NOT performance
claims. Sampling perturbs workloads; a short run might have too few observations.
"""
import argparse
import csv
import os
import pathlib
import signal
import subprocess
import sys
import time


def read_proc(pid):
    """Return (process_start_ticks, user_cpu_seconds, system_cpu_seconds, rss_kib)."""
    try:
        with open(f"/proc/{pid}/stat", encoding="ascii") as source:
            stat = source.read()
        end = stat.rfind(")")
        if end < 0:
            raise ValueError("malformed /proc stat")
        fields = stat[end + 2:].split()
        # fields[0] is field 3 (state), fields[11:13] are fields 14/15.
        start_ticks = int(fields[19])
        user_ticks, system_ticks = int(fields[11]), int(fields[12])
        with open(f"/proc/{pid}/statm", encoding="ascii") as source:
            resident_pages = int(source.read().split()[1])
        clock_ticks = os.sysconf("SC_CLK_TCK")
        page_kib = os.sysconf("SC_PAGE_SIZE") / 1024
        return (start_ticks, user_ticks / clock_ticks, system_ticks / clock_ticks,
                int(resident_pages * page_kib))
    except (OSError, IndexError, ValueError):
        return None


def run(responder_pid, manager_pid, command, resource_csv, initiator_log,
        max_seconds, period_ms):
    if not command:
        raise ValueError("initiator command must not be empty")
    if responder_pid <= 0 or max_seconds <= 0 or not 10 <= period_ms <= 5000:
        raise ValueError("invalid process ID, timeout or sample period")
    roles = {"responder": responder_pid}
    if manager_pid:
        if manager_pid <= 0:
            raise ValueError("invalid manager process ID")
        roles["manager"] = manager_pid
    identities = {}
    for role, pid in roles.items():
        snapshot = read_proc(pid)
        if snapshot is None:
            raise RuntimeError(f"{role} pid {pid} is unavailable before sampling")
        identities[role] = snapshot[0]

    path = pathlib.Path(resource_csv)
    path.parent.mkdir(parents=True, exist_ok=True)
    log_path = pathlib.Path(initiator_log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    process = None
    try:
        with path.open("w", newline="") as output, log_path.open("w") as log:
            writer = csv.writer(output)
            writer.writerow(["elapsed_ms", "role", "pid", "start_ticks",
                             "cpu_user_seconds", "cpu_system_seconds", "rss_kib"])
            started = time.monotonic_ns()
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            roles["initiator"] = process.pid
            initial = read_proc(process.pid)
            if initial is not None:
                identities["initiator"] = initial[0]
            timed_out = False
            while True:
                instant = time.monotonic_ns()
                elapsed_ms = (instant - started) / 1_000_000
                for role, pid in roles.items():
                    snapshot = read_proc(pid)
                    # Reject PID reuse; never attribute another process's
                    # memory/CPU usage to this benchmark process.
                    if snapshot is None or identities.get(role) != snapshot[0]:
                        continue
                    writer.writerow([f"{elapsed_ms:.3f}", role, pid, snapshot[0],
                                     f"{snapshot[1]:.6f}", f"{snapshot[2]:.6f}",
                                     snapshot[3]])
                return_code = process.poll()
                if return_code is not None:
                    output.flush()
                    return return_code
                if elapsed_ms >= max_seconds * 1000:
                    timed_out = True
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    break
                time.sleep(period_ms / 1000)
            output.flush()
            if timed_out:
                print(f"initiator exceeded {max_seconds}s; terminated", file=sys.stderr)
                return 124
            return 1
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--responder-pid", type=int, required=True)
    parser.add_argument("--manager-pid", type=int, default=0)
    parser.add_argument("--resource-csv", required=True)
    parser.add_argument("--initiator-log", required=True)
    parser.add_argument("--max-run-secs", type=int, default=180)
    parser.add_argument("--period-ms", type=int, default=100)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command
    if command and command[0] == "--":
        command = command[1:]
    try:
        return run(args.responder_pid, args.manager_pid, command,
                   args.resource_csv, args.initiator_log,
                   args.max_run_secs, args.period_ms)
    except (ValueError, OSError, RuntimeError) as exc:
        print(f"resource sampler failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
