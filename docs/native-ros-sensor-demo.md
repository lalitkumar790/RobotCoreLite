# Complete ROS 2 LaserScan → RobotCore-Lite IPC and shared-memory functional demonstration

## What is tested

The same C++ ROS integration source builds **three executables**, one per native transport: `sensor_ipc_demo`, `sensor_shm_demo`, and `sensor_persistent_shm_demo`. IPC and session-scoped SHM run in three **separate Linux OS processes**, while persistent SHM also requires a separate manager process before its endpoints attach:

- `sensor_ipc_demo`: Unix-domain socket; sink owns the server socket.
- `sensor_shm_demo`: bounded, one-client POSIX shared-memory region; sink creates and owns the region and unlinks it during normal shutdown.
- `sensor_persistent_shm_demo`: manager-owned persistent POSIX shared-memory region; manager starts an epoch, remains alive until both endpoints exit, and explicitly cleans up the region.

The three functional roles are:

1. **Source:** Publishes a native `sensor_msgs/msg/LaserScan` message on `/robotcore/native_scan`. The message contains a ROS header timestamp and frame ID, scan geometry, range bounds, 360 generated float32 ranges, and a demonstration sequence encoded in `intensities[0]`.
2. **Bridge:** Subscribes to typed `LaserScan`, validates the test message, then uses ROS 2 `rclcpp::Serialization<LaserScan>` to serialize the **complete message** into CDR. A bounded versioned envelope carries the serialized data through the selected `robotcore::Transport` implementation.
3. **Sink:** Decodes the envelope, reconstructs a typed `LaserScan` with ROS 2 deserialization, and checks the timestamp, frame ID, scan geometry, ranges, intensities, sequence and monotonic ordering before acknowledging the frame.

The codec is implemented in `include/robotcore_ros/scan_codec.hpp`. The wire envelope uses 12 little-endian bytes: magic (`RCLS`), version 1, and serialized CDR payload length. Full frame size is limited to 65,536 bytes, including the envelope. The Unix IPC transport adds its own framing and FNV-1a integrity checksum; shared-memory variants also check their slot integrity and session ownership. None is cryptographic authentication. Incompatible versions, malformed lengths and oversized envelopes are rejected.

The `scan_codec_test` executable verifies a complete typed CDR roundtrip with nondefault frame ID, timestamp, all scan geometry fields, variable-length ranges and intensities; it also rejects invalid envelope magic, version, CDR length, truncation, trailing data and oversized frames. ROS 2 Jazzy CI runs the shared codec test and cross-process integration for all three transports, requiring 20 validated IPC frames and 50 validated frames through each SHM transport. This tests correctness of the full typed payload, not a sustained-rate throughput benchmark.

## Build and run

Requires Ubuntu 24.04, ROS 2 Jazzy, `rclcpp`, `sensor_msgs`, and Linux Unix-domain sockets.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robotcore_ros --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run robotcore_baseline scan_codec_test

bash scripts/test_ros_sensor_bridge.sh ipc 20
bash scripts/test_ros_sensor_bridge.sh shm 50
bash scripts/test_ros_sensor_bridge.sh persistent_shm 50
```

The script starts the sink before the bridge and source, waits for the IPC socket or SHM region, and reserves a DDS discovery window. Persistent mode starts its manager first and stops it only after both endpoints exit; the test also checks that the named region is unlinked on clean shutdown. The source generates scans; no sensor driver or hardware is required.

## Precise limitations

- **Typed and complete message transfer:** all fields within the ROS 2 `LaserScan` message are transported through CDR. ROS 2/DDS transport metadata that is not part of `LaserScan` itself (for example, DDS publication identity) is not included.
- The scan contents are **generated**, not captured from a physical LiDAR. The sequence-in-intensity convention is a demo fixture, not a general `LaserScan` protocol or recommended application design.
- Each native transport carries the complete CDR representation but is **not ROS middleware discovery** on the sink side. The sink is a RobotCore-Lite transport application, not a ROS topic republisher.
- The typed bridge now supports Unix IPC, bounded SHM and persistent SHM using one CDR codec; the existing four-mode RTT harness additionally includes DDS. This demonstration does not replace a native DDS `LaserScan` round-trip benchmark and does not ensure that each backend's queueing, ownership and overhead are equivalent.
- This is not a high-rate sensor-throughput evaluation or physical robot control loop. The test verifies delivery and type/field correctness, **not** latency superiority, reliable operation under sustained backpressure, or crash-resilient reconnection.
- The bounded decoder is versioned, but malformed CDR itself is handed to the ROS 2 CDR deserializer. It is not a security boundary for hostile untrusted IPC peers.

Next milestones: sustained-rate delivery/load tests, a native ROS sink-side topic republisher, a controlled comparison with DDS using the same typed message path, and a measured application run on a reference host.
