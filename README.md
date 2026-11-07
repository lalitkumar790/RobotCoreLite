# RobotCore-Lite

**Fresh research implementation.** This repository does not recover or reproduce historical source code. RobotCore Telemetry is an entirely separate project.

## Mission
Evaluate whether alternative local communication transports can reduce ROS 2 node communication overhead while preserving correctness, reliability, and integration usability. **No speedup is claimed.**

## Current milestone: Phase 1 — DDS baseline
- ROS 2 C++ ping/pong executables with sequence IDs and monotonic RTT timing
- CSV of each completed sample **and** timeout
- Parameterized payload size, sample count, warmup, timeout, QoS, and run ID
- Repeatable shell orchestration, captured environment metadata, analysis script, CI lint and Python self-tests
- Reports use **RTT** (round-trip time). `RTT/2` is a proxy, **not** an observed one-way latency; clocks are not synchronized.

### Environment
Target **Ubuntu 24.04 + ROS 2 Jazzy**, GCC / C++17. The runner checks the actual `ROS_DISTRO` and `ros2` availability; it does not assume what is installed. Jazzy is pinned as the reference distribution for reproducibility, but exact package versions and compiler are recorded per run.

### Quick start (Ubuntu 24.04 with ROS 2 Jazzy installed)
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
./scripts/run_benchmarks.sh
python3 scripts/analyze_results.py results/raw results/reports
```
Run in an isolated ROS_DOMAIN_ID; stop other high-load processes and record hardware configuration. The runner creates timestamped raw CSVs and metadata. Results are generated locally, **not committed as fabricated benchmarks**.

### Research plan
1. Measure baseline DDS under controlled conditions.
2. Introduce transport abstraction and explicit configuration.
3. Add optimized IPC, bounded framing and failure handling.
4. Add shared memory and persistent buffer ownership, cleanup and concurrent access tests.
5. Compare against ROS 2 baseline and available ROS 2 shared-memory/loaning options fairly.
6. Release raw data, plots, reproducibility notes, sensor workload demo and technical report.

Design: [architecture](docs/architecture.md) · [methodology](docs/benchmark-methodology.md) · [decisions](docs/design-decisions.md)

## Interpreting old numbers
659 µs baseline, 68 µs IPC, 112 µs SHM, 44 µs persistent IPC, 33 µs persistent SHM are **unverified historical leads**, not measurements from this codebase.

## License
Apache-2.0; see LICENSE.
