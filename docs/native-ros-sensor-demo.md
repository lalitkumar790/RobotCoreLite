# Native LaserScan reference and complete-message transport validation

## What is tested

The same C++ ROS test source builds **four executables**: a native typed-DDS reference `sensor_dds_demo`, plus `sensor_ipc_demo`, `sensor_shm_demo`, and `sensor_persistent_shm_demo`. The native DDS reference has two independent processes (publisher and subscriber). IPC and session-scoped SHM use three processes (publisher, typed ROS subscriber/bridge, native sink); persistent SHM adds a fourth region-manager process. These are **different communication topologies** and are not sufficient for a fair latency comparison.

The four roles/transport paths are:

- `sensor_dds_demo`: native ROS 2 `sensor_msgs/msg/LaserScan` publisher and typed DDS subscriber; no RobotCore-Lite forwarding hop.
- `sensor_ipc_demo`: Unix-domain socket; sink owns the server socket.
- `sensor_shm_demo`: bounded, one-client POSIX shared-memory region; sink creates and owns the region and unlinks it during normal shutdown.
- `sensor_persistent_shm_demo`: manager-owned persistent POSIX shared-memory region; manager starts an epoch, remains alive until both endpoints exit, and explicitly cleans up the region.

The functional roles for the native transport paths are:

1. **Source:** Publishes a native `sensor_msgs/msg/LaserScan` message on `/robotcore/native_scan`. The message contains a ROS header timestamp and frame ID, scan geometry, range bounds, 360 generated float32 ranges, and a demonstration sequence encoded in `intensities[0]`.
2. **Bridge:** Subscribes to typed `LaserScan`, validates the test message, then uses ROS 2 `rclcpp::Serialization<LaserScan>` to serialize the **complete message** into CDR. A bounded versioned envelope carries the serialized data through the selected `robotcore::Transport` implementation.
3. **Sink:** Decodes the envelope, reconstructs a typed `LaserScan` with ROS 2 deserialization, and checks the timestamp, frame ID, scan geometry, ranges, intensities, sequence and monotonic ordering before acknowledging the frame.

The codec is implemented in `include/robotcore_ros/scan_codec.hpp`. The wire envelope uses 12 little-endian bytes: magic (`RCLS`), version 1, and serialized CDR payload length. Full frame size is limited to 65,536 bytes, including the envelope. The Unix IPC transport adds its own framing and FNV-1a integrity checksum; shared-memory variants also check their slot integrity and session ownership. None is cryptographic authentication. Incompatible versions, malformed lengths and oversized envelopes are rejected.

The `scan_codec_test` executable verifies contiguous-sequence rejection (missing, duplicated, out-of-order or extra frames) and a complete typed CDR roundtrip with nondefault frame ID, timestamp, all scan geometry fields, variable-length ranges and intensities; it also rejects invalid envelope magic, version, CDR length, truncation, trailing data and oversized frames. ROS 2 Jazzy CI runs the shared codec tests and **strict 250-frame delivery trials at a requested 10 ms source period** for native DDS and all three native transports. The publisher waits until a ROS subscription matches, sends exactly 250 scans, and must report that count; the receiver must validate scans **1 through 250 without gaps, duplicates, reordering or invalid payloads**. The trial fails if any expected scan is missing. This tests modest-duration correctness at a controlled *requested application cadence*, not guaranteed physical publish timing, throughput saturation, or a trustworthy latency benchmark.

## Build and run

Requires Ubuntu 24.04, ROS 2 Jazzy, `rclcpp`, `sensor_msgs`, and Linux Unix-domain sockets.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robotcore_ros --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run robotcore_baseline scan_codec_test

bash scripts/test_ros_sensor_bridge.sh dds 250 10
bash scripts/test_ros_sensor_bridge.sh ipc 250 10
bash scripts/test_ros_sensor_bridge.sh shm 250 10
bash scripts/test_ros_sensor_bridge.sh persistent_shm 250 10

# Optional longer stress checks (10 seconds of requested 100 Hz scans each)
bash scripts/test_ros_sensor_bridge.sh dds 1000 10
bash scripts/test_ros_sensor_bridge.sh ipc 1000 10
bash scripts/test_ros_sensor_bridge.sh shm 1000 10
bash scripts/test_ros_sensor_bridge.sh persistent_shm 1000 10
```

The script starts the sink before the bridge and source, waits for the IPC socket or SHM region, and reserves a DDS discovery window. The source additionally waits for at least one matched ROS subscription and exits unsuccessfully if it cannot send the requested count. A strict sink rejects gaps/duplicates/out-of-order messages. DDS uses a direct typed subscriber rather than a native bridge. Persistent mode starts its manager first and stops it only after both endpoints exit; the test also checks that the named region is unlinked on clean shutdown. The source generates scans; no sensor driver or hardware is required.

## Precise limitations

- **Typed and complete message transfer:** all fields within the ROS 2 `LaserScan` message are transported through CDR. ROS 2/DDS transport metadata that is not part of `LaserScan` itself (for example, DDS publication identity) is not included.
- The scan contents are **generated**, not captured from a physical LiDAR. The sequence-in-intensity convention is a demo fixture, not a general `LaserScan` protocol or recommended application design.
- Each native transport carries the complete CDR representation but is **not ROS middleware discovery** on the sink side. The sink is a RobotCore-Lite transport application, not a ROS topic republisher.
- The native DDS reference validates the same generated typed `LaserScan` fields but has **one fewer application hop** than the ROS → native transport bridge paths. It provides a functional control, not a matched performance comparison. Each native backend uses the same CDR codec, while DDS uses its middleware's typed message serialization.
- The CI trials prove success for a finite 250-frame sequence **only**. The requested period is not measured publication jitter; no saturated throughput, resource-controlled latency, full backpressure characterization, or physical robot control loop is claimed. Higher-rate/larger-volume trials may expose losses and should not be silently excluded.
- The bounded decoder is versioned, but malformed CDR itself is handed to the ROS 2 CDR deserializer. It is not a security boundary for hostile untrusted IPC peers.

Next milestones: longer and variable-rate sequences with retained per-sequence timing/logs, an explicit backpressure experiment, a native ROS sink-side topic republisher, a matched **same-topology** latency comparison against DDS, and a real sensor bag/robot application run on a documented host.
