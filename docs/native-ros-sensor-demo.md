# Complete ROS 2 LaserScan → RobotCore-Lite IPC functional demonstration

## What is tested

The `sensor_ipc_demo` executable runs in three **separate Linux OS processes**:

1. **Source:** Publishes a native `sensor_msgs/msg/LaserScan` message on `/robotcore/native_scan`. The message contains a ROS header timestamp and frame ID, scan geometry, range bounds, 360 generated float32 ranges, and a demonstration sequence encoded in `intensities[0]`.
2. **Bridge:** Subscribes to typed `LaserScan`, validates the test message, then uses ROS 2 `rclcpp::Serialization<LaserScan>` to serialize the **complete message** into CDR. A bounded versioned envelope carries the serialized data through `robotcore::UnixSocketTransport`.
3. **Sink:** Decodes the envelope, reconstructs a typed `LaserScan` with ROS 2 deserialization, and checks the timestamp, frame ID, scan geometry, ranges, intensities, sequence and monotonic ordering before acknowledging the frame.

The codec is implemented in `include/robotcore_ros/scan_codec.hpp`. The wire envelope uses 12 little-endian bytes: magic (`RCLS`), version 1, and serialized CDR payload length. Full frame size is limited to 65,536 bytes, including the envelope. The IPC transport adds its own framing and FNV-1a integrity checksum, which is **not cryptographic authentication**. Incompatible versions, malformed lengths and oversized envelopes are rejected.

The `scan_codec_test` executable verifies a complete typed CDR roundtrip with nondefault frame ID, timestamp, all scan geometry fields, variable-length ranges and intensities; it also rejects invalid envelope magic, version, CDR length, truncation, trailing data and oversized frames. ROS 2 Jazzy CI runs that test and the three-process integration, requiring 20 valid received frames.

## Build and run

Requires Ubuntu 24.04, ROS 2 Jazzy, `rclcpp`, `sensor_msgs`, and Linux Unix-domain sockets.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robotcore_ros --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run robotcore_baseline scan_codec_test

socket="/tmp/robotcore-native-demo-$$.sock"
ros2 run robotcore_baseline sensor_ipc_demo --role sink --socket "$socket" --samples 20 &
sink=$!
ros2 run robotcore_baseline sensor_ipc_demo --role bridge --socket "$socket" &
bridge=$!
sleep 2  # allow the ROS discovery/subscription to settle
ros2 run robotcore_baseline sensor_ipc_demo --role source --samples 500 &
source=$!
wait "$sink"
kill "$bridge" "$source" 2>/dev/null || true
wait "$bridge" "$source" 2>/dev/null || true
```

Start the sink before the bridge and source. CI also waits for the socket to appear before starting the bridge. The source generates scans; no sensor driver or hardware is required.

## Precise limitations

- **Typed and complete message transfer:** all fields within the ROS 2 `LaserScan` message are transported through CDR. ROS 2/DDS transport metadata that is not part of `LaserScan` itself (for example, DDS publication identity) is not included.
- The scan contents are **generated**, not captured from a physical LiDAR. The sequence-in-intensity convention is a demo fixture, not a general `LaserScan` protocol or recommended application design.
- The IPC link contains the complete CDR representation but is **not ROS middleware discovery** on the sink side. The sink is a RobotCore-Lite native IPC application, not a ROS topic republisher.
- The current bridge targets optimized Unix-socket IPC only. Shared memory and persistent memory have separate functional RTT harnesses, not equivalent typed ROS bridges.
- This is not a high-rate sensor-throughput evaluation or physical robot control loop. The test verifies delivery and type/field correctness, **not** latency superiority, reliable operation under sustained backpressure, or crash-resilient reconnection.
- The bounded decoder is versioned, but malformed CDR itself is handed to the ROS 2 CDR deserializer. It is not a security boundary for hostile untrusted IPC peers.

Next milestones: sustained-rate delivery/load tests, a type-preserving bridge backed by other transports, and a measured robotics application run on a controlled reference host.
