# Phase 3: Unix socket IPC (first implementation)

Linux AF_UNIX/SOCK_STREAM framed transport, implementing the existing `robotcore::Transport` API. This is a functional candidate, **not a benchmark result**.

- One server endpoint and one client peer at a time, with sequential client reconnection supported. The server owns and removes its socket path on normal shutdown. Existing paths are never silently deleted on bind.
- 20-byte frame header: 4-byte magic (little-endian 0x52434c31), 4-byte payload length, 8-byte message sequence, 4-byte FNV-1a integrity hash over payload, then byte payload.
- Per-message receive-side bound check happens **before allocation**; global upper limit 16 MiB.
- `publish()` serializes concurrent writers; a two-second socket send timeout limits ordinary backpressure stalls, but a successful send only means bytes accepted by the kernel, **not delivered or processed**.
- One receive worker dispatches callbacks. Avoid concurrent `start()`/`stop()`; `stop()` must not be invoked from a callback. If peer closes or frame is invalid, call `stop()` before restarting.
- Socket path permissions are limited to owner read/write. For multi-user threat models, place socket in a private 0700 directory and use peer-credential checks; this implementation does not yet authenticate peers or provide encryption.
- No automatic client reconnect/backoff, endpoint discovery, multiple subscribers, durable queues, message-level ACK, or QoS equivalence to DDS yet.

## Build and test (no ROS required)
```sh
cmake -S robotcore_transports/ipc -B build/ipc -DCMAKE_BUILD_TYPE=Debug
cmake --build build/ipc
ctest --test-dir build/ipc --output-on-failure --repeat until-fail:20
```

## Use

```cpp
robotcore::Configuration c;
c.transport = robotcore::TransportKind::ipc;
c.max_payload_bytes = 4096;
robotcore::UnixSocketTransport endpoint("/run/user/1000/robotcore-example.sock", true, c);
endpoint.start([](robotcore::Message message) { /* consume owned message */ });
// Client endpoint: same socket path, server=false, started after server listens.
endpoint.stop();
```

The example path is illustrative and must be changed to a private directory belonging to the current user. The test creates a unique socket path under /tmp.

## Validation scope
The test forks **before** starting transport threads, exchanges sequenced binary messages in two distinct processes, checks replies and oversize-payload rejection, rejects duplicate endpoint ownership, and verifies cleanup/restart. No published microsecond claims are implied. Follow-up work: malformed-frame injection, fuzzing, cancellation under load, process-crash recovery, multi-client design, and controlled DDS-vs-IPC benchmark runs.

## Fault-injection validation
CI now tests idle-server cancellation before any client connects and injects malformed raw socket frames (truncated header, invalid magic, length above the allowed limit, bad checksum and short payload). These must never dispatch application callbacks; tests do not measure performance. The server accept worker polls at bounded intervals to ensure stop does not rely on Linux-specific listening-socket shutdown behavior.

Concurrent-publish stress testing now exercises 4 producer threads (150 messages each), validates payload integrity and exactly-once observation **within this local test**, and checks clean peer disconnect. This is not a general exactly-once delivery guarantee or a throughput benchmark.

## Sequential reconnect and crash recovery
The server now keeps its listening socket alive after a client disconnect and accepts a new client in sequence. The client must call `stop()` then `start()` to reconnect, explicitly retrying if the server is unavailable. A crashed server may leave a filesystem socket path: the library **does not** automatically unlink it, to avoid deleting another process's endpoint. Recovery requires operator-controlled cleanup after verifying ownership, peer inactivity and directory permissions. Automated tests exercise two sequential clients, a SIGKILL orphaned socket, refusal to overwrite the orphan, explicit fixture cleanup, and successful subsequent reconnect. This is not automatic crash recovery or multi-client support.
