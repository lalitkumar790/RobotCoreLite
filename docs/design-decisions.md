# Design decisions

1. **Research first; product-quality engineering.** Evidence before optimization claims.
2. **Target ROS 2 Jazzy on Ubuntu 24.04** because it is an officially supported LTS combination. Scripts fail if ROS_DISTRO is wrong. Record actual toolchain and RMW for each run.
3. **Monotonic RTT before one-way timing.** Initial baseline measures RTT without clocks synchronized between processes. RTT/2 is a proxy only.
4. **Separate processes.** Publisher and echo responder are independent ROS nodes and OS processes. Avoid confusing intra-process optimization with cross-process transport.
5. **Explicit backpressure for baseline:** stop-and-wait, one outstanding message, so initial numbers are latency, not peak throughput. Throughput is a distinct experiment later.
6. **Same payload and QoS across compared transports.** Future implementations must declare payload format and copy semantics; no silent changes.
7. **No results invented.** Raw CSV and host metadata are generated on a ROS-enabled machine and ignored by Git. Historical figures are unverified.
8. **No unrelated projects.** RobotCore Telemetry is separate, with no dependency or migration implied.
