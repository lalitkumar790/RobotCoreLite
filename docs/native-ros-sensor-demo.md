# ROS 2 typed LaserScan → RobotCore-Lite IPC functional demonstration

## What this proves

The executable `sensor_ipc_demo` in `robotcore_baseline` runs in three **independent Linux OS processes**:

1. `source`: publishes generated, structurally valid ROS 2 `sensor_msgs/msg/LaserScan` on `/robotcore/native_scan`, with 360 float32 ranges, ROS header/frame ID, angles and range bounds. For test purposes, `intensities[0]` carries an exactly representable sequence number.
2. `bridge`: a real ROS 2 subscription validates the native LaserScan fields, copies 360 little-endian float32 ranges into a `robotcore::Message` and publishes through `robotcore::UnixSocketTransport`.
3. `sink`: a distinct IPC server verifies exact range content against the reported sequence, rejects out-of-order or repeated sequence numbers, and exits successfully after a specified number of valid frames.

The demo checks **typed ROS subscription → transport abstraction → cross-process IPC delivery and validation**, without changing the research benchmarking code. It does not transport an entire native `sensor_msgs` message, because the IPC payload intentionally includes only the 360 range measurements and a separate sequence number, omitting frame timestamp, TF frame, angles and intensity metadata.

## Build and run

Requires Ubuntu 24.04 and ROS 2 Jazzy (`rclcpp`, `sensor_msgs`), plus a Linux Unix socket. Source the Jazzy setup, then:

```bash
colcon build --base-paths robotcore_ros --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release
source /opt/ros/jazzy/setup.bash
source install/setup.bash

socket="/tmp/robotcore-native-demo-$$.sock"
ros2 run robotcore_baseline sensor_ipc_demo --role sink --socket "$socket" --samples 20 &
sink=$!
ros2 run robotcore_baseline sensor_ipc_demo --role bridge --socket "$socket" &
bridge=$!
ros2 run robotcore_baseline sensor_ipc_demo --role source --samples 500 &
source=$!
wait "$sink"
kill "$bridge" "$source" 2>/dev/null || true
wait "$bridge" "$source" 2>/dev/null || true
```

**Start the sink before the bridge** and launch the source after the bridge subscribes. CI performs this startup ordering, waiting for the Unix socket before connecting and allowing DDS discovery before starting the publisher. In production, use unique socket names and a cleanup trap; the code rejects existing non-owned server socket paths.

## Limitations

- Synthetic deterministic LiDAR ranges, **not** an actual physical robot, real sensor driver, or sensor log.
- The IPC bridge serializes **only range data and sequence**. Native `sensor_msgs` metadata is validated at ingress, then deliberately omitted on the IPC link.
- Only the optimized IPC backend is integrated in this demo. The existing four-mode RTT runner remains a distinct, byte-level transport comparison.
- No timestamps/RTT or throughput result is reported by the demo. Do not infer lower latency from successful delivery.
- No TF, robot trajectory, simultaneous clients, backpressure analysis, or end-to-end ROS 2 control loop yet.

The next steps are a type-preserving serialization schema for native ROS 2 messages, a consistent bridge implementation for the other transport modes, and real sensor bag/robotics application testing with retained raw data.
