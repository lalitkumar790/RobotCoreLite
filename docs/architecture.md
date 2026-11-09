# Architecture (planned and implemented boundaries)

ROS 2 application -> RobotCore-Lite API (Phase 2) -> configurable transport:

- Standard ROS 2 DDS (Phase 1 reference, currently implemented with rclcpp)
- Local IPC (Phase 3 candidate)
- Shared memory (Phase 4 initial bounded bidirectional implementation; one peer, queue depth 1)
- Persistent shared memory (Phase 4 candidate)

The DDS baseline is intentionally a standalone ROS 2 package, not an abstraction pretending that transports already exist. Future interface should explicitly define message framing, endpoint discovery, buffer ownership, delivery semantics, backpressure, shutdown and fallback.

**Measurement distinction:** ping/pong initiator and responder use distinct processes and one steady clock for RTT. This avoids cross-process clock synchronization errors but includes subscriber callbacks, echo dispatch and process scheduling in RTT. No assumption that half-RTT equals actual one-way delivery.

**Security:** future IPC endpoints must validate permissions and message lengths; shared-memory allocations must be bounded, permissioned and cleaned after crashes.
