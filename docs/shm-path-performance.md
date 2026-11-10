# SHM loan-versus-copy experiment (performance research, not DDS)

This experiment isolates the *native session shared-memory* payload access
paths. It does not claim ROS 2 zero-copy, outperform Fast DDS, or bound
real-time latency. Neither the existing DDS UInt8MultiArray comparison nor
this microbenchmark is a Fast DDS loaned-message control.

## Run on Ubuntu Linux

```bash
RUNS=6 SAMPLES=2000 WARMUP=200 PAYLOADS="64 4096 65536 1048576" \
  bash scripts/run_shm_path_benchmark.sh
```

The runner builds `shm_path_bench` in Release, uses separate responder and
initiator OS processes, alternates copied/loaned order across repetitions,
requires a complete all-success CSV per run, records an environment snapshot,
and creates a descriptive per-run median/worst RTT report. A timeout or corrupt
payload fails the experiment rather than being silently discarded.

The copied mode uses `publish(Message)` and `start(ReceiveCallback)`.
The loaned mode uses `acquire_write_loan()`, `commit()` and
`start_borrowed()` on *both* sides. Each response is checked against
sequence-dependent bytes. The responder must echo bytes to its opposite
directional slot: the borrowed callback therefore still performs an
application-level read/write copy while avoiding intermediate owned vectors.
Frames are generated inside the timestamped send operation in both modes;
this comparison **includes producer work** and is not a pure transport
wake-up benchmark. Each responder checks bytes before echoing.

## Important limitations

- Single slot per direction, one in-flight frame, reliable stop-and-wait RTT.
  No burst throughput, contention, multiple subscribers or bounded P99.99
  under load is measured here.
- The borrowed slot is held until callback return, so slow subscribers can
  increase backpressure. The hash still scans the payload.
- Linux scheduling, CPU frequency, host placement and system noise can vary.
  The runner does not pin CPUs, enforce governors, allocate huge pages, or
  control allocator/kernel cache effects.
- Order alternation helps but does not replace a pre-registered paired
  experimental protocol with many independent runs.
- **Never compare this result directly with a DDS test using different
  types/topology, serialization, QoS or process counts.**
- Do not use CI virtualized-runner timing as research-grade performance.

## Next acceptance gates

1. Establish a bounded, fixed-layout native sensor frame with no allocation
   in the publish/receive hot path, validated across two OS processes.
2. Implement a *separate* optimized Fast DDS reference with actual Data-sharing
   enabled and compatible loaned data types, recording the real configuration
   and verifying loan support at runtime. Do not assume loans are active.
3. Compare matching process topology, size, rates, QoS, pool capacity,
   payload generation/validation location, CPU placement and failure policy.
4. Test burst and sustained load at offered rates below, near and above
   saturation. Preserve drop/backpressure/timeout counts and per-run P50,
   P95, P99, P99.9 and maximum latency; inspect multi-second stalls.
5. Only claim improvement when independent host runs, reproducible raw data,
   uncertainty intervals, load behavior, correctness and resource costs
   justify the claim.

Start with this SHM-only comparison to see whether our own loaned path
actually improves on our copied path. Winning that comparison is **necessary
for this particular optimization rationale**, but **not sufficient** to claim
competitive performance against Fast DDS.
