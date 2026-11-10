# Sustained producer load: SHM copied vs loaned (experimental)

This is a **same-host cross-process** measurement of the existing one-slot,
single-publisher/single-subscriber SHM implementation. It is **not** a ROS 2
middleware benchmark, a DDS loaned-message control, an end-to-end zero-copy
claim, or a real-time scheduling guarantee.

## Execute

On a controlled Ubuntu Linux host:

```bash
RUNS=6 FRAMES=5000 PAYLOADS="64 4096 65536" DELAYS_US="0 250 1000" \
  bash scripts/run_shm_load.sh
```

The runner builds the experimental binary in Release, alternates the order
of copied and borrowed modes, executes sender and receiver in **different OS
processes**, and writes raw sender/receiver CSVs plus an environment snapshot.
The analyzer rejects missing, out-of-order, unpaired, timestamp-inconsistent,
or corrupt frames rather than discarding them.

A **source-intent timestamp** is sampled immediately before the sender's API
work; the receiver samples `CLOCK_MONOTONIC` on callback entry. These clocks
share a time base on the same Linux kernel. This yields *source intent to
callback entry* (including publisher-slot blocking), not pure propagation or
network one-way time. Sender API duration includes payload construction and
slot acquisition/commit, and therefore needs separate interpretation.
The copied mode reuses its owned message buffer between frames. The loaned
mode writes directly into mapped slot bytes. Both paths retain hash validation.

`DELAYS_US` introduces explicit post-entry receiver callback sleep. The
borrowed receive callback keeps the slot occupied during its work, whereas the
copied callback returns its input slot before invoking application code.
Consequently the modes expose different backpressure semantics; comparing
them is a trade-off study, not an apples-to-apples abstraction benchmark.

The sender publishes as fast as possible (open-loop in that it does not await
application acknowledgments, but **blockable** at the single-slot transport);
there is no constant-rate source or burst scheduling guarantee. Both modes
require reliable, contiguous delivery. The output contains per-run publish
P50/P99/max and source-to-callback P50/P99/max. CI tests are **functional
only**, regardless of the numerical timing values they produce.

## Known limitations and next design tests

- One slot per direction and one reader; the publisher is limited by receiver
  consumption and cannot have an unbounded number of outstanding frames.
- The receiver records samples in memory and writes the CSV only after delivery
  completes; failures are rejected, not hidden.
- Publication paths differ in application-level memory manipulation. Source
  stamping is intentionally before producer work; separate post-commit
  timestamps would be needed to isolate time between slot readiness and
  callback entry.
- These measurements do not prove bounded worst-case latency; OS scheduling,
  page faults, and a stalled receiver can still dominate.
- Future tests must use rate-controlled sources with overload/drop policy,
  multiple fixed-layout slots, explicit latency budgets, and CPU affinity.
- Fast DDS needs a **distinct validated Data-sharing and loaned-sample
  implementation**, with comparable type bounds, process counts, QoS, and
  resource budgets before any competition claim.
