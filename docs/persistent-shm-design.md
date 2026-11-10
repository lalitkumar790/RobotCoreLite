# Persistent shared-memory design and initial implementation

## Research question

Does an independently managed, preallocated shared-memory pool that survives **client and server endpoint sessions** measurably reduce allocation/setup overhead or RTT, compared with the current one-session `shared_memory` mode, Unix sockets, and ROS 2 DDS?

The current SHM implementation already reuses two slots within one session, so changing only its name, a preallocated vector, or a cached pointer would not constitute a distinct research contribution.

## Proposed distinctive semantics

- A **region manager** owns the named memory, not an ordinary server endpoint. Endpoint stop/restart does not unlink or reinitialize the region; the manager performs explicit cleanup and enforces byte/slot quotas.
- Each connection obtains a **session epoch/lease ID**. A message tagged with a prior epoch cannot be delivered into the next session after reconnect or crash.
- A fixed ring of **preallocated slots** is leased and returned; producer writes only to free slots. The manager must fence stale ownership before reassigning memory after process death. Process-shared POSIX semaphores alone are not crash-recoverable.
- A shared **robust process-shared mutex** or separate manager-coordinated state machine must detect owner death and validate/repair invariants. Recovery resets or discards unfinished slots; it never silently replays a partially written frame.
- Persistent mode should use a separate **loan/return message API** to avoid extra application copies. The existing `Transport::publish(const Message&)` requires ownership/copy semantics, so the current API cannot support a proven end-to-end zero-copy claim.
- Use process identity (pidfd plus process-start-time), permissions/credentials and monotonic epoch changes. Never reclaim a slot or endpoint using PID alone because PIDs can be reused.
- Explicitly document what is preserved: buffer capacity is persistent, **message delivery is not**. No durability guarantee or packet-loss inference.

## Test gates before benchmark comparison

1. Distinct manager and endpoints survive multiple start/stop sessions without reallocating the mapped region; record allocation counts/epochs rather than assuming savings.
2. Kill each role while owning or writing a slot; verify fail-closed behavior, bounded reconnect and no cross-epoch delivery.
3. Run concurrent producer stress, sanitizer tests, corrupted-size injection and ownership lease exhaustion tests.
4. Release outstanding loaned messages on every exception/timeout path, and prove no buffer reuse before consumer releases it.
5. Run matched DDS/IPC/SHM/persistent-SHM experiments on a controlled Ubuntu 24.04 + Jazzy host and retain raw results.

## Current status

The initial persistent SHM transport is implemented, using a separate manager and epoch-fenced shared region. Persistent pool/loan APIs and automatic manager-failure recovery are **not** implemented. No persistent-mode latency, throughput or zero-copy performance benefit has been established.

## Initial implementation — manager-controlled persistent epochs

The dedicated `robotcore_transports/persistent_shm` C++ module now supplies `PersistentRegionManager` and `PersistentSharedMemoryTransport`. The manager creates and owns the POSIX named mapping independently of endpoint lifetimes, and `begin_session()` resets both slots and increments an epoch **only when no endpoint process from the previous session is alive**. Each server/client role may attach once per epoch; a separate process can act as the manager. Cross-process tests verify repeated sessions reuse the same mapped object (device and inode remain unchanged), and simulate a crashed client with a stale lease and potentially abandoned slot. The manager checks process PID plus start-time to prevent PID reuse false positives; stale messages are discarded at an explicit epoch reset. This is **not** transparent reconnection or zero-copy: endpoints copy payloads into and out of preallocated buffers, and the manager performs explicit recovery. Sessions require coordinated shutdown/restart of both roles. The manager rejects destruction while endpoints remain live.

A future advanced mode still needs loan/return buffers, manager crash recovery, access-control hardening, multiple clients, and real-world measurements. Do not interpret initial functional correctness as a speedup.

## Fault injection and manager death detection

A separate test suite kills a running **manager process** after the client and server complete an echo. Both attached endpoints must fail closed on subsequent publish. The orphaned shared-memory name must not be silently adopted by another manager; only the test deliberately unlinks it after the manager is reaped and endpoints detach.

Additional fault tests inject an oversized length, a stale generation tag and a mismatched integrity hash into the server's inbound slot. A receiver that finds any of these rejects the frame **without invoking the application callback** and marks the whole session poisoned (`ready=2`). The other endpoint can then detect session closure rather than filling an abandoned slot. Only an explicit manager `begin_session()` can reset the poisoned semaphores, after the previous endpoints are stopped. These tests cover selected failure paths; they are not fuzzing or proof of full crash consistency.

The `RCP1` wire-layout magic has not changed because the added behavior uses its existing `ready` flag; manager and transport processes must use compatible binaries. The FNV-1a hash protects accidental integrity errors only, not malicious tampering. The manager still has no automatic resurrection or authenticated remote control.

## Isolated setup-cost benchmark (not transport RTT)

The `persistent_region_setup` executable measures two deliberately different manager operations with `steady_clock`: (1) cold `start()` plus the first `begin_session()` (POSIX mapping creation, initialization and first epoch), and (2) an additional `begin_session()` **on that same mapping** (warm synchronization reset, no region allocation). It records each observed pair in CSV after warmup. The companion Python analyzer rejects duplicate or incomplete pairs and generates descriptive means/medians and minimum/maximum timings. The repeatable runner builds Release binaries and writes environment metadata.

```bash
bash scripts/run_persistent_setup.sh
# Environment options: SAMPLES=1000 WARMUP=50 CAPACITY=1024
```

CI tests only compilation, the smoke CSV schema and report generation. Neither a fast warm reset nor a ratio of the two operations demonstrates improved communication latency: the operations are intentionally different, there are no endpoints and no message RTT, and OS page cache, allocator state, scheduling and frequency scaling remain confounders. A valid claim about lifecycle benefits requires multi-session endpoint startup tests on dedicated controlled machines, with raw data and resource metrics retained.

## Partial-write crash regression

A separate crash-injection test attaches a real client in a child process, acquires the outbound `free` semaphore and writes an **uncommitted, incomplete slot header** without posting the `filled` semaphore. While the writer is alive, `PersistentRegionManager::begin_session()` must refuse to reset the region. The test kills the writer with `SIGKILL`, reaps the process, explicitly begins a new epoch, and verifies a subsequent request/reply does not deliver the abandoned frame.

This checks one deliberately constructed crash window, **not** exhaustive crash consistency. It does not prove automatic recovery or correct behavior for every possible concurrent publish/stop interleaving.

## Concurrent publish/stop lifetime guard

Both Linux SHM endpoint implementations now serialize each `publish()` against final mapping release in `stop()`. The shutdown path first marks the endpoint stopped and **joins the receive worker without holding the publish mutex** (a receive callback may itself call `publish()`). It then acquires the publish mutex, waits for any blocked publisher to leave the semaphore wait, and only then clears the lease/unmaps memory. This prevents a same-object `stop()` from unmapping a slot semaphore under an in-flight local publisher.

A shared test source creates a receiver callback that intentionally blocks, fills the sender's single-slot buffer, and stops the sender concurrently with a third, blocked publication. Each backend repeats the check twelve times in normal CI and under ASan/UBSan. The blocked publication must fail and a subsequent publish on the stopped endpoint must not succeed.

**Scope:** This is not a claim that arbitrary lifecycle concurrency is safe: callers must still serialize `start()`, simultaneous `stop()` calls, destruction and manager session resets. Stop must not be invoked inside a transport receive callback. External process death, callback exceptions, and cross-process synchronization remain separately tested and constrained; the publisher mutex is per endpoint and is not a cross-process lock. A publish operation already committed immediately before shutdown can still be in flight at the peer.
