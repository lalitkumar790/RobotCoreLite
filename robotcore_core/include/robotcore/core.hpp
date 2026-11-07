#pragma once
// RobotCore-Lite transport contract, independent of ROS 2 and any OS transport.
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robotcore {

constexpr std::size_t kMaxPayloadBytes = 16 * 1024 * 1024;

enum class TransportKind { dds, ipc, shared_memory, persistent_shared_memory };
enum class Error { ok, invalid_argument, too_large, unsupported, not_started, already_started, closed, io_error };

struct Status {
  Error error{Error::ok};
  std::string detail;
  explicit operator bool() const noexcept { return error == Error::ok; }
  static Status success() { return {}; }
  static Status failure(Error error, std::string detail) { return {error, std::move(detail)}; }
};

struct Configuration {
  TransportKind transport{TransportKind::dds};
  std::size_t max_payload_bytes{kMaxPayloadBytes};
  std::size_t queue_depth{10};
  bool reliable{true};
  bool allow_dds_fallback{false};
};

inline Status validate(const Configuration& c) {
  if (c.max_payload_bytes == 0 || c.max_payload_bytes > kMaxPayloadBytes)
    return Status::failure(Error::invalid_argument, "max_payload_bytes outside 1..16 MiB");
  if (c.queue_depth == 0 || c.queue_depth > 10000)
    return Status::failure(Error::invalid_argument, "queue_depth outside 1..10000");
  return Status::success();
}

// Message bytes are owned by the caller while publish() executes. A transport
// must copy or take ownership before returning; it must never retain a pointer
// into the caller's mutable buffer. Received messages own their byte vectors.
struct Message {
  std::uint64_t sequence{0};
  std::vector<std::uint8_t> payload;
};

using ReceiveCallback = std::function<void(Message)>;

class Transport {
public:
  virtual ~Transport() = default;
  virtual Status start(ReceiveCallback on_receive) = 0;
  virtual Status publish(const Message& message) = 0;
  virtual Status stop() = 0;
  virtual TransportKind kind() const noexcept = 0;
};

// Resolve requested transport explicitly. Unimplemented transports are
// rejected unless the user has opted into DDS fallback. Never silently
// claim that shared memory is operating.
inline std::pair<Status, TransportKind> resolve_transport(const Configuration& c) {
  const auto validity = validate(c);
  if (!validity) return {validity, c.transport};
  if (c.transport == TransportKind::dds || c.transport == TransportKind::ipc)
    return {Status::success(), c.transport};
  if (c.allow_dds_fallback) return {Status::success(), TransportKind::dds};
  return {Status::failure(Error::unsupported, "requested transport is not implemented"), c.transport};
}

inline Status validate_message(const Message& m, const Configuration& c) {
  const auto validity = validate(c);
  if (!validity) return validity;
  if (m.payload.size() > c.max_payload_bytes)
    return Status::failure(Error::too_large, "message exceeds configured maximum");
  return Status::success();
}

}  // namespace robotcore
