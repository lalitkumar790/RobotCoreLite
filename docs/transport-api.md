# Phase 2: transport contract

The C++17 contract in `robotcore_core/include/robotcore/core.hpp` is independent of ROS 2. The reference DDS adapter is in `robotcore_ros/robotcore_baseline/include/robotcore_ros/dds_transport.hpp`.

- `Transport::start(callback)` registers one consumer and begins delivery.
- `Transport::publish(const Message&)` completes only after the input may safely be reused. It does **not** mean delivery or acknowledgement.
- `Transport::stop()` releases transport resources. Start/stop and the application's ROS executor must not race: this is not a concurrent-stop-safe interface yet.
- Each `Message` contains a 64-bit sequence and owned payload vector. The DDS adapter encodes the sequence as the first 8 little-endian bytes of a `UInt8MultiArray`; the rest is the payload.
- Maximum payload size is 16 MiB (user-configurable downward); receive-side DDS frames are checked as well.
- Unsupported transport modes fail explicitly; `allow_dds_fallback=true` resolves them to DDS. The caller must construct the adapter matching the resolved kind.
- `Loopback` is a test double, not a transport or benchmark.
- `dds_adapter_smoke` demonstrates the API over DDS with two ROS nodes in **one process**. It establishes functional delivery only, not multi-process performance, deterministic latency, or packet loss guarantees.

Build core contract tests without ROS:
```sh
cmake -S robotcore_core -B build/core -DCMAKE_BUILD_TYPE=Debug
cmake --build build/core
ctest --test-dir build/core --output-on-failure
```

Build and run functional ROS smoke test on Jazzy:
```sh
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robotcore_ros --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run robotcore_baseline dds_adapter_smoke
```

**Still outstanding:** cross-process ROS API integration, persistent shared-memory buffer reuse, multi-client delivery, crash-safe synchronization, concurrent callback lifecycle semantics and research-grade performance measurements. An initial bounded POSIX shared-memory candidate is available; its queue depth is fixed at one.
