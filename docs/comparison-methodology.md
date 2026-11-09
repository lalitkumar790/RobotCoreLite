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
5. Compare mean, median, P95, P99 **RTT**, and per-run missing-reply rate. Inspect variability across repeats. CPU/memory measurement is a separate experiment requiring process-level counters at matching time intervals; this harness records GNU `time` process resource summaries if available but does not yet provide sustained resource profiling.
6. Performance assertions require reviewing the raw data, experimental setup and variance, not a single impressive number.

## Commands

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/run_comparison.sh
# Results appear under results/raw/comparison-<UTC>/ and results/reports/comparison-<UTC>/
```

The comparison runner requires `QOS_DEPTH=1` and `RELIABILITY=reliable` for all four modes. Neither SHM mode supports deeper queues, and DDS/IPC still have differing callback and backpressure semantics.

Adjust with environment variables `RUNS`, `PAYLOADS`, `SAMPLES`, `WARMUP`, `TIMEOUT_MS`, `DISCOVERY_TIMEOUT_MS`, `RELIABILITY`, `QOS_DEPTH`, and `ROS_DOMAIN_ID`. Do not publish results without recording and controlling these.

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
