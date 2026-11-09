# Phase 4 — bounded POSIX shared-memory transport (initial implementation)

This experimental Linux-only candidate implements the ROS-independent C++17 Transport API. It is not persistent shared memory and it has not demonstrated a latency advantage.

## Synchronization and ownership

- One server creates a named object with shm_open(O_CREAT|O_EXCL, 0600); one client attaches using the same name, payload capacity and layout version.
- Two independent one-message slots represent client-to-server and server-to-client directions. Each uses process-shared POSIX semaphores (free, filled).
- Each frame has a 64-bit sequence, bounded length and FNV-1a checksum; FNV-1a detects accidental corruption but is not authentication.
- Input payloads are copied into the mapping, and received payloads are copied out into owned vectors *before* the shared slot is released. This is not zero-copy end-to-end.
- A configuration requires transport=shared_memory, queue_depth=1 and reliable=true. A single slot bounds concurrent producers through a semaphore; publishing waits at most approximately two seconds for capacity.
- A successful publish only means bytes have been placed in the shared slot, not that another process handled or acknowledged them.
- The creator unlinks the name on normal stop. Existing names are not overwritten. Server process death may leave an orphaned shared-memory name and a semaphore in an unusable state; operator-controlled recovery after verifying ownership is required.
- This version has one server and one client only. Callback-driven echo is supported but concurrent stop/publish, callbacks invoking stop, crash-time lock recovery and multi-client operations are not supported.

## Build and run tests

```bash
cmake -S robotcore_transports/shared_memory -B build/shm -DCMAKE_BUILD_TYPE=Debug
cmake --build build/shm --parallel
ctest --test-dir build/shm --output-on-failure --repeat until-fail:20
```

The cross-process test forks before starting worker threads, exchanges 60 sequenced binary messages with replies, checks oversized messages, duplicate name refusal, cleanup, restart and invalid queue configuration. No microsecond performance claim is made.

## Next steps

Add process-crash and malformed-header injection, owner/generation verification, concurrency/sanitizer testing, controlled transport comparisons, persistent buffer reuse and ROS 2 sensor workloads.
