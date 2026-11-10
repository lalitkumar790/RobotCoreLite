# First native loaned shared-memory path (experimental)

## Why this matters

The previous RobotCore-Lite SHM implementation **copied** the caller's
`Message.payload` into the mapped region on publish, and copied it back into
a new `std::vector` on receive. Its one-slot, one-peer architecture was
therefore not even trying to match a modern loan-based DDS data path.

Session SHM now has an **opt-in** loaned-buffer interface:

- `SharedMemoryTransport::acquire_write_loan(WriteLoan&)` reserves the
  outgoing slot and returns its **actual mapped writable bytes**.
- `WriteLoan::commit(sequence, length)` computes the same FNV-1a integrity
  hash and publishes the shared slot without copying a user payload into it.
  `discard()` or RAII destruction returns an unwritten slot.
- `start_borrowed(BorrowedReceiveCallback)` invokes the callback with a
  **read-only view of that exact shared slot**. It holds the slot's semaphore
  until callback return, instead of copying the payload to a new vector.
  This may increase backpressure for slow callbacks.

This eliminates the two **application payload memory copies** in this
single-slot native SHM path, but it does *not* eliminate validation passes
(FNV-1a reads the entire payload), kernel scheduling, semaphore overhead,
cache-coherency traffic, or ROS 2 serialization. It is **not** an
end-to-end ROS 2 zero-copy path and cannot be presented as faster than DDS.

## Usage sketch

```cpp
robotcore::Configuration config;
config.transport = robotcore::TransportKind::shared_memory;
config.queue_depth = 1;
config.max_payload_bytes = 4096;

robotcore::SharedMemoryTransport server("/robotcore-loaned", true, config);
auto status = server.start_borrowed([](robotcore::BorrowedMessage frame) {
  // frame.data points into mapped shared memory. Read synchronously only.
  inspect(frame.sequence, frame.data, frame.size);
});
// Separate process must attach with same name and configuration.
robotcore::SharedMemoryTransport client("/robotcore-loaned", false, config);
client.start([](robotcore::Message) {});

robotcore::SharedMemoryTransport::WriteLoan loan;
if (client.acquire_write_loan(loan)) {
  produce_directly_into(loan.data(), loan.capacity());
  status = loan.commit(1, produced_size);
  // If commit fails, discard()/destructor releases a still-held reservation.
}
client.stop();
server.stop();
```

The callback above uses illustrative application functions `inspect` and
`produce_directly_into` that the caller must implement. In a real deployment
the server and client are **separate processes**, with the server running
first. The regression test runs them in separate forked processes.

## Ownership and safety contract

- `WriteLoan` is **move-only**. A single active reservation holds a
  per-endpoint mutex, so `stop()` waits for it before unmapping memory.
  Calling `stop()` in the same thread while retaining the reservation
  will deadlock; discard or commit the loan first. Do not destroy the
  transport before its last loan has been discarded.
- `commit()` consumes a successful loan. Calling it a second time fails.
  Invalid oversized commits are rejected **without** releasing the loan,
  allowing the caller to correct the size or discard explicitly.
- `BorrowedMessage::data` is valid **only inside the callback**. Never save
  the pointer or pass it to asynchronous work. To retain data, copy it in
  the application. The slot is released as soon as the callback returns.
- Borrowed receive callbacks may call `publish()` on the *opposite*
  directional slot, but **must not** invoke `stop()`.
- Two directions still have only **one slot each**. There is one server and
  at most one client per region. No automatic recovery, loan revocation,
  thread-safe lifecycle reentrancy, multi-reader fanout, or multi-slot
  pipeline is implemented. The loan API is optional; old `publish()` and
  `start()` behavior remains for compatibility.
- The hash detects some accidental corruption; it is not authentication.
  A process killed while holding a loan can strand the semaphore. This
  SHM mode deliberately fails closed rather than silently reusing it.

## Test gates

`shared_memory_borrowed_cross_process` exercises native mapped publisher
writes and borrowed receiver reads in **two Linux OS processes** with
variable sizes including an empty and a full-capacity payload; it verifies
contents and order for 40 messages. It covers move-only loan ownership,
automatic discard, repeated discard, oversized commit rejection and
correction, double-commit rejection, loan after stop, and shutdown while a
loan is held. CTest includes the test in the existing native
ASan/UBSan CI matrix.

These tests do **not** compare runtime performance or assert true hardware
zero-copy. The next performance milestone is to integrate an end-to-end
bounded/fixed-layout robotics sensor type directly into this loaned API,
and compare against **Fast DDS Data-sharing + DataWriter/DataReader loans**
on the same system, build settings, sample sizes and matched topology.

See: https://fast-dds.docs.eprosima.com/en/stable/fastdds/use_cases/zero_copy/zero_copy.html
