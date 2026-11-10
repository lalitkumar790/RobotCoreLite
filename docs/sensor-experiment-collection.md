# Repeated matched-topology LaserScan experiment collection

This is an **experiment runner**, not a publication of performance results.

## Purpose

The ROS 2 sensor demo now offers four matched **application process** paths:

- DDS/CDR bridge: native LaserScan publisher → subscriber/CDR encoder → DDS `UInt8MultiArray` → typed CDR receiver
- Unix IPC: native LaserScan publisher → subscriber/CDR encoder → Unix-domain socket → typed CDR receiver
- Session-scoped SHM: native LaserScan publisher → subscriber/CDR encoder → bounded POSIX SHM → typed CDR receiver
- Persistent SHM: same three application processes, plus a separate region manager

The direct two-process typed DDS path is intentionally **excluded**. It is a functional control with fewer application hops.

The new `scripts/run_sensor_experiment.py` runs each four-arm block in a seeded shuffled/rotated order: across each block of four rounds, every mode occurs once in each presentation position. With fewer than four repetitions, complete position balancing is impossible. This mitigates—but does not eliminate—warming, scheduling or temporal host-load bias.

## Running on a controlled Ubuntu 24.04 / ROS 2 Jazzy host

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robotcore_ros --packages-select robotcore_baseline \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 4 trials per mode, 250 generated scans per run, requested 10-ms cadence
python3 scripts/run_sensor_experiment.py \
  --repetitions 4 \
  --frames 250 \
  --period-ms 10 \
  --seed 790 \
  --timeout-seconds 90 \
  --output-root results/sensor-experiments
```

Run within **one Linux host / monotonic clock domain**. This does not support comparing processes across machines with unsynchronized clocks.

The output directory contains `manifest.json` (randomized order, settings, git SHA, kernel, CPU/governor, RMW vendor and selected affinity), `run_summary.csv`, `report.md`, and directories such as `round001_pos1_dds_bridge/`. Every run retains `source_events.csv`, `sink_events.csv`, `paired_events.csv`, `summary.json`, `environment.txt`, `run_status.json` and `runner.log`. The collector validates all frames and raw-to-paired timestamp consistency before marking the run successful. If a transport fails, it writes a `failed` status, preserves the partial outputs, and exits nonzero. Do not quietly discard unsuccessful runs or run only successful modes.

The suite reports a **per-run distribution of application-visible end-to-end delays**, with descriptive means and medians. It does not rank modes, compute a speculative speedup, or use CI smoke values as real-machine performance evidence. Workload generation, bridge serialization and dispatch, queues, middleware work, and reader callbacks remain material confounders; the SHM modes copy payloads rather than zero-copy loaning them.

## Critical methodology limits

- Requested source cadence is not automatically achieved cadence. Inspect actual source timestamp intervals in `paired_events.csv`.
- Observed end-to-end delay uses source pre-`publish()` and sink callback entry (before payload validation), *not* network/DDS transmission delay or one-way transport latency.
- A complete run does not prove system reliability at capacity. Test backpressure, concurrent clients, large messages, scheduling/load and crash windows separately.
- A timeout is a **failed experiment** and should be reported, not silently excluded.
- Retain raw event streams, manifests, process logs, exact ROS distro/RMW vendor and any CPU-governor changes.
- For a credible comparison use a documented dedicated host, warmup protocols, repeated independent collection blocks/days, fixed affinity/frequency governor if possible, actual CPU/memory profiling, and an uncertainty analysis that respects between-run clustering.
- The deterministic test fixture in `tests/test_sensor_experiment_runner.py` validates only collection/reporting behavior; it does not emulate ROS or establish performance numbers.

Follow up with a sustained-load publication experiment, ROS bag/real LiDAR data, and a robot application test before any research performance claim.

## Run-level uncertainty: completed collections only

After all trials succeed, the collector automatically runs `scripts/analyze_sensor_runs.py` and writes `run_uncertainty.json` and `run_uncertainty.md`. The analyzer checks the manifest against **every round/mode**, per-run status, frame count, unmodified raw source and sink monotonic timestamps, and paired observations. It refuses failed/incomplete datasets, duplicate observations, nonfinite timing metrics or mismatched summaries.

The output treats a **whole experimental round**—one observation for each transport—as the paired resampling unit, not each individual sensor frame. For every native transport, it calculates a per-round *median application-visible delay difference* relative to the DDS/CDR bridge. With at least four complete rounds, a seeded, exploratory 95% percentile bootstrap interval is shown for the **mean of these paired differences**. Fewer than four rounds yield no interval. The interval is sensitive to the few available runs; even four rounds, or an interval not crossing zero, must **not** be advertised as scientific proof of speedup or a nominally calibrated confidence guarantee. The output explicitly sets `performance_claim: false`.

To independently re-analyze a retained collection:

```bash
python3 scripts/analyze_sensor_runs.py results/sensor-experiments/<collection-dir> --seed 790 --resamples 4000
```

Use independent collection blocks/days, sufficient replication, and a prespecified hypothesis/analysis plan before submitting comparative results for publication.

## Optional per-process CPU and RSS sampling

For **supplementary** resource observations, run with:

```bash
python3 scripts/run_sensor_experiment.py \
  --repetitions 4 --frames 250 --period-ms 10 --seed 790 \
  --profile-resources --resource-period-ms 100
```

The collector records PID/start-time-verified Linux `/proc` observations for the source, bridge and sink in each three-process arm, plus the persistent region manager in that arm. It retains `process_ids.csv`, `resource_samples.csv` and `resource_summary.json` per run. For each role it reports the **observed** CPU core equivalents (change in user+system CPU seconds divided by that role's observation-window duration) and sampled maximum resident memory. Processes with fewer than two samples have **no CPU-rate estimate**. The resource sampling window includes process startup, ROS discovery and teardown with different role lifetimes; it is **not** a matched steady-state CPU-cost or exact peak-memory measurement. Sampling itself perturbs timing. To avoid conflation, compare both unprofiled and separately profiled collections rather than using profiled run times as primary unperturbed performance results.

Tests of these collectors use *explicitly synthetic fixture timestamps*. They verify bookkeeping, not ROS 2 performance.
