# Four-mode communication comparison (measurement protocol)

## Scope and truthfulness

This is a **cross-process functional and benchmarking harness**, not proof that either transport is faster. It compiles the same C++ initiator/responder workload from `benchmarks/latency/transport_benchmark.cpp` for four backends. A sequence-zero discovery probe is repeated before warmup until a reply is received or the configurable discovery deadline expires; probes are excluded from CSV. Each initiator then uses a single in-flight sequence-numbered application message, captures a `steady_clock` timestamp before publish, and records RTT on callback arrival. All successful RTT samples and timed-out requests are written to CSV. Warmup timeouts cause an invalid run and immediate abort; no fabricated samples.

The payload size is the **application payload**, excluding transport headers. All transports process identical fixed bytes and a 64-bit sequence. DDS uses ROS 2 `UInt8MultiArray`, current RMW/default middleware and configured reliable QoS. IPC uses `AF_UNIX` stream sockets, a 20-byte framing header and FNV-1a payload integrity check. Session-scoped shared memory uses a one-message-per-direction buffer with process-shared semaphores, copying payload into and out of POSIX memory. Persistent shared memory uses similarly bounded and copied payloads, but an independently running manager owns the region and controls session epochs. Those different framing/integrity semantics and middleware implementations remain **confounders**, not identical protocols. The RTT includes transport serialization, executor/callback dispatch, queueing, kernel scheduling, and echoed reply. RTT/2 remains an unverified proxy for one-way delay.

## Reference environment

Ubuntu 24.04 with ROS 2 Jazzy installed and sourced. Run on a dedicated otherwise-idle machine, ideally bare metal. Record RMW implementation, actual ROS package versions, CPU governor, host topology, machine load, virtualisation/container status, build flags and compiler version, with a controlled `ROS_DOMAIN_ID`. CI smoke results are **not** publishable comparative performance claims.

## Procedure

1. Confirm the selected Jazzy environment, build Release binaries and capture metadata.
2. Choose identical payload sizes (default 64, 1024, 65536 bytes), warmup (default 100), recorded attempts (default 1000), timeout (1000ms) and reliability (reliable).
3. Launch separate server/client OS processes for each transport. Rotate all four transport orders between repetitions; avoid concurrent benchmark pairs.
4. Run at least three repeats at each payload. Include **all** timed-out recorded attempts in failure rate, distinguish them from confirmed packet loss, and preserve raw CSV.
5. Compare mean, median, P95, P99 **RTT**, and per-run missing-reply rate. Inspect variability across repeats. A Linux `/proc` sampler now records cumulative user/system CPU seconds and resident memory for both initiator and responder, and also the independent region manager in persistent mode. The samples share the initiator-active window (including discovery and warmup), not an isolated steady-state phase. The sampler produces aligned raw CSV streams and descriptive CPU-rate/observed-RSS summaries, but may perturb short timings. Separate sustained-load resource profiling is still needed.
6. Performance assertions require reviewing the raw data, experimental setup and variance, not a single impressive number.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/run_comparison.sh
# Results appear under results/raw/comparison-<UTC>/ and results/reports/comparison-<UTC>/
```

The comparison runner requires `QOS_DEPTH=1` and `RELIABILITY=reliable` for all four modes. Neither SHM mode supports deeper queues, and DDS/IPC still have differing callback and backpressure semantics.

Adjust with environment variables `RUNS`, `PAYLOADS`, `SAMPLES`, `WARMUP`, `TIMEOUT_MS`, `DISCOVERY_TIMEOUT_MS`, `RELIABILITY`, `QOS_DEPTH`, `PROFILE_PERIOD_MS` (default 100 ms) and `ROS_DOMAIN_ID`. Do not publish results without recording and controlling these.

## Known limitations

- One outstanding request: this experiment measures stop-and-wait RTT, **not peak throughput**.
- Callback/reply logic is shared but the middleware and socket worker scheduling differ.
- Not a sensor-message/ROS robotics end-to-end workload.
- No CPU pinning, CPU frequency control, cross-host testing, security parity, DDS implementation sweep, or statistical confidence intervals yet.
- A callback reply after its deadline is ignored by sequence number; the missing reply is not necessarily a dropped packet.
- CI smoke checks compile and exercise delivery, but performance comparisons must come from dedicated controlled runs.

## Persistent memory experimental limitation

The four-mode runner currently creates a **fresh persistent manager for each recorded RTT run**, then launches a responder and initiator as separate processes. Manager region initialization and session setup occur **before the unmeasured discovery probe**, so recorded RTT samples exclude initial allocation. This tests that messages are delivered through the persistent-mode protocol under the same stop-and-wait workload. It does **not** measure the hypothesized benefit of avoiding region allocations across repeated endpoint sessions; a separate cold-start versus warm-restart experiment is needed. The dedicated multi-session functional tests verify that the manager can retain a region across restarts. Do not interpret an RTT comparison as proof of persistence-related setup savings.

The persistent manager is terminated only after both initiator and responder exit; it never resets a live session. Results continue to represent **round-trip time** in microseconds, not observed one-way latency, nor resource-equivalent throughput. Each transport's internal framing, buffering and thread wakeups differ and remain confounders.

## CPU/memory sampling and limitations

The runner now writes `<run_id>.resources.csv` beside each RTT CSV, plus `resources.csv` and `resources.md` in the report directory. Samples use Linux `/proc/<pid>/stat` CPU counters, `statm` resident pages, PID and process-start-time checks, and timestamps from a monotonic clock. Processes are sampled at the **same observation instants** while the initiator is running; persistent SHM has an additional manager role. Responder and manager start before the window; their startup costs are therefore **not** included in these samples. A process may finish between samples, and sampled maximum RSS is not an exact peak. CPU "core equivalents" means the change in user+system CPU seconds divided by the *observed* time between that role's first and last sample, **not** a CPU percentage. Runs with fewer than two samples get no CPU-rate estimate.

These measurements are not necessarily comparable to another machine, RMW implementation, DDS QoS configuration, or experiment with different process lifecycles. The sampler itself has overhead that can perturb short RTTs. Use dedicated hosts, long workloads, multiple repetitions and comparable observation windows before drawing performance conclusions. Raw data and environmental metadata should be published alongside any findings.

## Deterministic synthetic robotics-shaped payload profiles

The runner also supports `PAYLOAD_PROFILE=laser360`, `gray_image_vga`, or `depth16_vga` (default `uniform`). These profiles encode, respectively, 360 synthetic float32 range samples (1,440 bytes), a changing 640 × 480 single-channel uint8 image (307,200 bytes), and a changing 640 × 480 little-endian uint16 depth image (614,400 bytes). Frame bytes vary with the application sequence number, are generated **before** the RTT clock starts, and the initiator validates the full returned byte payload against that exact sequence. This is a deterministic *sensor-shaped byte workload*, **not** live sensor data, `sensor_msgs/LaserScan`, `sensor_msgs/Image`, DDS loaned messages, or a full perception/control pipeline. The DDS adapter continues using `std_msgs/UInt8MultiArray`.

Examples (Ubuntu 24.04 / ROS 2 Jazzy):
```bash
source /opt/ros/jazzy/setup.bash
RUNS=3 PAYLOAD_PROFILE=laser360 SAMPLES=1000 WARMUP=100 bash scripts/run_comparison.sh
RUNS=3 PAYLOAD_PROFILE=gray_image_vga SAMPLES=1000 WARMUP=100 bash scripts/run_comparison.sh
RUNS=3 PAYLOAD_PROFILE=depth16_vga SAMPLES=1000 WARMUP=100 bash scripts/run_comparison.sh
```
Each profile selects its exact application byte size. An explicit `PAYLOADS` must match that size; mixing profiles in the same summarized group is prohibited. Individual RTT CSV files include `payload_profile` alongside status. An incorrect full-payload echo is recorded as `invalid_payload` rather than incorrectly counted as a timeout; any such run exits unsuccessfully and must be investigated. A missing reply after the deadline is recorded as `timeout`, not verified packet loss.

**Experimental caveat:** The synthetic frames change data and message sizes, but stop-and-wait RTT remains a single-in-flight workload. It is neither realistic sensor cadence nor throughput/load validation. Encoding work occurs before timestamping, while payload checking after callback arrival may affect inter-message pacing. Do not claim sensor integration or speedup based on these profiles alone.
