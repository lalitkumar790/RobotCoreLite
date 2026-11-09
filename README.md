# RobotCore-Lite

**Fresh research implementation.** This repository does not recover or reproduce historical source code. RobotCore Telemetry is an entirely separate project.

## Mission
Evaluate whether alternative local communication transports can reduce ROS 2 node communication overhead while preserving correctness, reliability, and integration usability. **No speedup is claimed.**

## Implemented foundations: DDS baseline, transport API, Linux IPC and initial POSIX shared memory
- ROS 2 C++ ping/pong executables with sequence IDs and monotonic RTT timing
- ROS-independent C++17 Transport interface, explicit DDS fallback and payload validation
- ROS 2 DDS adapter implementing the interface and a functional in-process smoke executable
- CSV of each completed sample **and** timeout
- Parameterized payload size, sample count, warmup, timeout, QoS, and run ID
- Repeatable shell orchestration, captured environment metadata, analysis script, CI lint and Python self-tests
- Reports use **RTT** (round-trip time). `RTT/2` is a proxy, **not** an observed one-way latency; clocks are not synchronized.

### Environment
Target **Ubuntu 24.04 + ROS 2 Jazzy**, GCC / C++17. The runner checks the actual `ROS_DISTRO` and `ros2` availability; it does not assume what is installed. Jazzy is pinned as the reference distribution for reproducibility, but exact package versions and compiler are recorded per run.

### Quick start (Ubuntu 24.04 with ROS 2 Jazzy installed)
```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robotcore_ros --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash scripts/run_benchmarks.sh
python3 scripts/analyze_results.py results/raw results/reports
```
Run in an isolated ROS_DOMAIN_ID; stop other high-load processes and record hardware configuration. The runner creates timestamped raw CSVs and metadata. Results are generated locally, **not committed as fabricated benchmarks**.

### Build and test the core interface
```bash
cmake -S robotcore_core -B build/core -DCMAKE_BUILD_TYPE=Debug
cmake --build build/core
ctest --test-dir build/core --output-on-failure
```

### DDS adapter smoke test
```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robotcore_ros --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run robotcore_baseline dds_adapter_smoke
```
This checks functional delivery only. It does not produce publishable performance results. See [transport API](docs/transport-api.md).

### IPC cross-process tests
```bash
cmake -S robotcore_transports/ipc -B build/ipc -DCMAKE_BUILD_TYPE=Debug
cmake --build build/ipc
ctest --test-dir build/ipc --output-on-failure
```
This validates functional framing, sequencing and lifecycle; it does **not** measure IPC speed. See [IPC design and limitations](docs/ipc-transport.md).

### Common four-transport comparison harness
All four transport executables compile the same stop-and-wait C++ test implementation. Both shared-memory variants have one slot per direction and use payload copies. The persistent mode additionally starts a separate manager process for each RTT experiment. For Ubuntu 24.04 and Jazzy:
```bash
source /opt/ros/jazzy/setup.bash
bash scripts/run_comparison.sh
```
The runner builds Release binaries, rotates transport order between repetitions, records raw RTT CSV and host metadata, and writes a summary report. It also records aligned Linux `/proc` CPU/RSS samples for the initiator and responder (plus the manager in persistent mode), saving `*.resources.csv` and `resources.md`; these are descriptive observations rather than controlled speedup claims. **No benchmark result is included in the repository.** The CI smoke checks are functional only; see [comparison methodology](docs/comparison-methodology.md).

### Phase 4: bounded POSIX shared memory (initial)
Linux POSIX shared memory now has a single-server/single-client C++ Transport implementation with two directional, bounded slots. Its process-shared semaphores synchronize reuse; each receive owns a copied payload. It reuses its two slots during each session, but does not yet implement a **distinct persistent shared-memory transport mode**, general crash recovery or production multi-client support. Single-client attachment is enforced, stale queued messages and unfinished callbacks block reattachment, and abrupt-server-exit leftovers are rejected rather than silently overwritten.

```bash
cmake -S robotcore_transports/shared_memory -B build/shm -DCMAKE_BUILD_TYPE=Debug
cmake --build build/shm --parallel
ctest --test-dir build/shm --output-on-failure --repeat until-fail:20
```

See [shared-memory design](docs/shared-memory.md). Successful functional tests are not latency measurements.

### Persistent shared memory (manager-owned region, experimental)

A distinct fourth transport has a region manager that keeps a bounded POSIX mapping alive across endpoint sessions. The manager explicitly starts new sessions with increasing epochs and resets abandoned synchronization only after verifying previous processes are gone. Endpoints are limited to one server and client per epoch, with owned receive copies (not zero-copy).

```bash
cmake -S robotcore_transports/persistent_shm -B build/pshm -DCMAKE_BUILD_TYPE=Debug
cmake --build build/pshm --parallel
ctest --test-dir build/pshm --output-on-failure
```

See [persistent SHM design and limitations](docs/persistent-shm-design.md). The fourth mode is integrated into the common comparison runner for **RTT functional validation**. The runner starts a fresh region manager per measured trial, so it does **not** measure warm-restart allocation savings.

### Persistent manager setup-cost experiment

The persistent SHM module also provides a **separate** cold-create versus warm-epoch-reset microbenchmark (not RTT, not robotics throughput). Run on a controlled Linux host:

```bash
bash scripts/run_persistent_setup.sh
```

It builds Release mode, records raw paired setup times and machine metadata, and writes a descriptive report. The operations deliberately differ; they are **not** an end-to-end apples-to-apples latency comparison. See [setup study design](docs/persistent-shm-design.md).


### Native sanitizer and partial-crash validation

The native Linux IPC, shared-memory and persistent-shared-memory test suites are additionally compiled with Clang AddressSanitizer and UndefinedBehaviorSanitizer in GitHub Actions. The persistent-memory suite includes a deterministic child-process crash after acquiring a slot but before marking the frame ready, verifying manager reset is blocked while the writer is live and old work is not delivered after an explicit epoch reset.

Sanitizer checks test selected memory/undefined-behavior paths; they do **not** prove thread-race freedom, crash-safe automatic recovery, or real-time determinism. ThreadSanitizer, multi-process fuzzing, and controlled physical-host benchmarks remain future work.

### Research plan
1. Measure baseline DDS under controlled conditions.
2. Introduce transport abstraction and explicit configuration.
3. Add optimized IPC, bounded framing and failure handling (initial implementation; hardening pending).
4. Validate the initial bounded shared-memory implementation and design a distinct persistent shared-memory mode with robust ownership, cleanup and concurrent access tests.
5. Compare against ROS 2 baseline and available ROS 2 shared-memory/loaning options fairly.
6. Release raw data, plots, reproducibility notes, sensor workload demo and technical report.

Design: [transport API](docs/transport-api.md) · [architecture](docs/architecture.md) · [methodology](docs/benchmark-methodology.md) · [decisions](docs/design-decisions.md)

## Interpreting old numbers
659 µs baseline, 68 µs IPC, 112 µs SHM, 44 µs persistent IPC, 33 µs persistent SHM are **unverified historical leads**, not measurements from this codebase.

## License
Apache-2.0; see LICENSE.
