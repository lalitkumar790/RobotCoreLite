# Phase 2: transport contract

The plain C++17 contract in `robotcore_core/include/robotcore/core.hpp` is intentionally ROS-independent.

- `Transport::start(callback)` registers one consumer and begins delivery.
- `Transport::publish(const Message&)` completes only once transport ownership/copy is safe. It does **not** imply delivery or acknowledgement.
- `Transport::stop()` stops delivery and must release transport resources. Implementations must define stop/callback concurrency before enabling multithread use.
- Each `Message` contains a 64-bit sequence and owned payload vector. Wire framing, timestamps, checksums, and endpoint naming are **not** yet standardized.
- `validate_message` bounds payload size against configuration (maximum 16 MiB). A production transport must validate the receive-side frame as well.
- Unsupported transports fail explicitly; `allow_dds_fallback=true` resolves to DDS but requires the application to construct the DDS adapter. Resolution by itself does not instantiate a transport.
- `Loopback` in the tests is deliberately a contract test double **not** a transport implementation or latency benchmark.

Build core contract tests, without ROS 2:
```sh
cmake -S robotcore_core -B build/core -DCMAKE_BUILD_TYPE=Release
cmake --build build/core
ctest --test-dir build/core --output-on-failure
```

**Integration status:** The existing DDS RTT benchmark is still independent from this API. A ROS 2 DDS adapter that implements `Transport` and covers publishers/subscribers is the next step. Do not count the API itself as a working alternative transport.
