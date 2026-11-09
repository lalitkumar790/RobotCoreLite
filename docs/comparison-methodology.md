# DDS versus IPC comparison (measurement protocol)

## Scope and truthfulness

This is a **cross-process functional and benchmarking harness**, not proof that either transport is faster. It compiles the same C++ initiator/responder workload from `benchmarks/latency/transport_benchmark.cpp` for the two backends. A sequence-zero discovery probe is repeated before warmup until a reply is received or the configurable discovery deadline expires; probes are excluded from CSV. Each initiator then uses a single in-flight sequence-numbered application message, captures a `steady_clock` timestamp before publish, and records RTT on callback arrival. All successful RTT samples and timed-out requests are written to CSV. Warmup timeouts cause an invalid run and immediate abort; no fabricated samples.

The payload size is the **application payload**, excluding transport headers. Both transports process identical fixed bytes and a 64-bit sequence. DDS uses ROS 2 `UInt8MultiArray`, current RMW/default middleware and configured reliable QoS. IPC uses `AF_UNIX` stream sockets, a 20-byte framing header and FNV-1a payload integrity check. Those different framing/integrity semantics and middleware implementations remain **confounders**, not identical protocols. The RTT includes transport serialization, executor/callback dispatch, queueing, kernel scheduling, and echoed reply. RTT/2 remains an unverified proxy for one-way delay.

## Reference environment

Ubuntu 24.04 with ROS 2 Jazzy installed and sourced. Run on a dedicated otherwise-idle machine, ideally bare metal. Record RMW implementation, actual ROS package versions, CPU governor, host topology, machine load, virtualisation/container status, build flags and compiler version, with a controlled `ROS_DOMAIN_ID`. CI smoke results are **not** publishable comparative performance claims.

## Procedure

1. Confirm the selected Jazzy environment, build Release binaries and capture metadata.
2. Choose identical payload sizes (default 64, 1024, 65536 bytes), warmup (default 100), recorded attempts (default 1000), timeout (1000ms) and reliability (reliable).
3. Launch separate server/client OS processes for each transport. Alternate transport order between repetitions; avoid concurrent benchmark pairs.
4. Run at least three repeats at each payload. Include **all** timed-out recorded attempts in failure rate, distinguish them from confirmed packet loss, and preserve raw CSV.
5. Compare mean, median, P95, P99 **RTT**, and per-run missing-reply rate. Inspect variability across repeats. CPU/memory measurement is a separate experiment requiring process-level counters at matching time intervals; this harness records GNU `time` process resource summaries if available but does not yet provide sustained resource profiling.
6. Performance assertions require reviewing the raw data, experimental setup and variance, not a single impressive number.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/run_comparison.sh
# Results appear under results/raw/comparison-<UTC>/ and results/reports/comparison-<UTC>/
```

Adjust with environment variables `RUNS`, `PAYLOADS`, `SAMPLES`, `WARMUP`, `TIMEOUT_MS`, `DISCOVERY_TIMEOUT_MS`, `RELIABILITY`, `QOS_DEPTH`, and `ROS_DOMAIN_ID`. Do not publish results without recording and controlling these.

## Known limitations

- One outstanding request: this experiment measures stop-and-wait RTT, **not peak throughput**.
- Callback/reply logic is shared but the middleware and socket worker scheduling differ.
- Not a sensor-message/ROS robotics end-to-end workload.
- No CPU pinning, CPU frequency control, cross-host testing, security parity, DDS implementation sweep, or statistical confidence intervals yet.
- A callback reply after its deadline is ignored by sequence number; the missing reply is not necessarily a dropped packet.
- CI smoke checks compile and exercise delivery, but performance comparisons must come from dedicated controlled runs.
