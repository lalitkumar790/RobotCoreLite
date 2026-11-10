#pragma once
#include "robotcore/core.hpp"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <mutex>

namespace robotcore {

// Owns a single POSIX shared-memory region across multiple endpoint sessions.
// A manager must outlive its endpoints. Each begin_session fences old messages.
class PersistentRegionManager {
public:
  PersistentRegionManager(std::string name, std::size_t capacity);
  ~PersistentRegionManager();
  PersistentRegionManager(const PersistentRegionManager&) = delete;
  PersistentRegionManager& operator=(const PersistentRegionManager&) = delete;
  Status start();
  Status begin_session();
  Status stop();
  std::uint64_t epoch() const;
private:
  std::string name_;
  std::size_t capacity_;
  void* mapping_{nullptr};
  std::size_t mapping_size_{0};
  int fd_{-1};
};

class PersistentSharedMemoryTransport final : public Transport {
public:
  PersistentSharedMemoryTransport(std::string name, bool server, Configuration config);
  ~PersistentSharedMemoryTransport() override;
  PersistentSharedMemoryTransport(const PersistentSharedMemoryTransport&) = delete;
  PersistentSharedMemoryTransport& operator=(const PersistentSharedMemoryTransport&) = delete;
  Status start(ReceiveCallback callback) override;
  Status publish(const Message& message) override;
  Status stop() override;
  TransportKind kind() const noexcept override { return TransportKind::persistent_shared_memory; }
  std::uint64_t session_epoch() const noexcept { return epoch_; }
private:
  bool manager_alive() const;
  void receive_loop();
  void release_resources();
  std::string name_;
  bool server_;
  Configuration config_;
  ReceiveCallback callback_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  // Serialize publishes and prevent stop() from unmapping a live publisher.
  // Do NOT hold this mutex while joining the receiver thread: a callback may publish.
  std::mutex publish_mutex_;
  void* mapping_{nullptr};
  std::size_t mapping_size_{0};
  int fd_{-1};
  int manager_pidfd_{-1};
  std::uint64_t epoch_{0};
  std::uint64_t identity_ticks_{0};
};
} // namespace robotcore
