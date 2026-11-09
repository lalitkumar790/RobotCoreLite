# Persistent shared-memory design proposal — not implemented

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

**Design only.** The `persistent_shared_memory` transport kind remains unsupported and resolves to DDS only with an explicit fallback. No persistent-mode latency, throughput or zero-copy performance has been measured.

## Initial implementation — manager-controlled persistent epochs

The dedicated `robotcore_transports/persistent_shm` C++ module now supplies `PersistentRegionManager` and `PersistentSharedMemoryTransport`. The manager creates and owns the POSIX named mapping independently of endpoint lifetimes, and `begin_session()` resets both slots and increments an epoch **only when no endpoint process from the previous session is alive**. Each server/client role may attach once per epoch; a separate process can act as the manager. Cross-process tests verify repeated sessions reuse the same mapped object (device and inode remain unchanged), and simulate a crashed client with a stale lease and potentially abandoned slot. The manager checks process PID plus start-time to prevent PID reuse false positives; stale messages are discarded at an explicit epoch reset. This is **not** transparent reconnection or zero-copy: endpoints copy payloads into and out of preallocated buffers, and the manager performs explicit recovery. Sessions require coordinated shutdown/restart of both roles. The manager rejects destruction while endpoints remain live.

A future advanced mode still needs loan/return buffers, manager crash recovery, access-control hardening, multiple clients, and real-world measurements. Do not interpret initial functional correctness as a speedup.
