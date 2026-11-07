# Phase 1 methodology: DDS baseline

## Setup

- Ubuntu 24.04 / ROS 2 Jazzy target; check actual installed ROS_DISTRO, RMW_IMPLEMENTATION, CPU and OS.
- Build release with `colcon build --base-paths robotcore_ros --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release`.
- `source install/setup.bash`, run `./scripts/run_benchmarks.sh`.
- Use stable machine frequency and thermal conditions; record CPU topology, RMW vendor, ROS_DOMAIN_ID, container/VM status, and competing loads. Retest on dedicated bare metal.

## What is actually measured

One requester in one process emits a fixed-size `UInt8MultiArray` containing a little-endian 64-bit sequence and a deterministic payload. Separate responder process echoes the exact message. Requester records `steady_clock` elapsed from before `publish()` through reply callback. Reports are **RTT**, including publish/callback/scheduling overhead. `RTT/2` is included in raw CSV only as a descriptive proxy; not a directly measured one-way latency.

- One outstanding message at a time (stop-and-wait). Initial target 100 warmups + 1000 recorded samples per run, 3 runs per 64, 1024 and 65536 bytes.
- Timeout is recorded as `status=timeout`, not silently excluded. The timeout rate reflects missing acknowledgements within a deadline; it cannot distinguish dropped packets from process overload or failures.
- Use same reliability/depth, payload sizes and process topology across modes. A future throughput benchmark must use a pipelined sender and independent receiver counting, not these stop-and-wait results.
- Record median, mean, P95, P99 and timeout rate for successful RTTs; summarize per run and preserve all sample rows.
- Exclude warmup rows from CSV; if warmup fails, run still proceeds but this must be called out as a limitation. Future work: connection handshake, warmup error rate and CPU/memory sampling.

## Scope limitations

This baseline does **not** yet measure sustained throughput, CPU and memory consumption, actual DDS network packet loss, one-way latency, or alternative transport speed. Results must not be compared to historical average numbers without identical conditions.
