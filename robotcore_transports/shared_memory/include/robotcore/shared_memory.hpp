#pragma once
#include "robotcore/core.hpp"
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
namespace robotcore {
// Linux POSIX shm with one server/client and two directional, bounded slots.
// Lifecycle methods are not concurrent-safe with publish. Do not stop in a callback.
class SharedMemoryTransport final : public Transport {
public:
  SharedMemoryTransport(std::string name, bool server, Configuration config);
  ~SharedMemoryTransport() override;
  SharedMemoryTransport(const SharedMemoryTransport&) = delete;
  SharedMemoryTransport& operator=(const SharedMemoryTransport&) = delete;
  Status start(ReceiveCallback callback) override;
  Status publish(const Message& message) override;
  Status stop() override;
  TransportKind kind() const noexcept override { return TransportKind::shared_memory; }
private:
  void receive_loop();
  void release_resources();
  std::string name_;
  bool server_;
  Configuration config_;
  ReceiveCallback callback_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  void* mapping_{nullptr};
  std::size_t mapping_size_{0};
  bool owns_name_{false};
  bool owns_client_claim_{false};
};
} // namespace robotcore
