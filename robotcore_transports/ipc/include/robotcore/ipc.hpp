#pragma once
#include "robotcore/core.hpp"
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace robotcore {
// Linux Unix stream sockets; one peer and one receive worker.
// Lifecycle methods are not reentrant; stop must not run inside callback.
class UnixSocketTransport final : public Transport {
public:
  UnixSocketTransport(std::string path, bool server, Configuration config = {});
  ~UnixSocketTransport() override;
  UnixSocketTransport(const UnixSocketTransport&) = delete;
  UnixSocketTransport& operator=(const UnixSocketTransport&) = delete;
  Status start(ReceiveCallback callback) override;
  Status publish(const Message& message) override;
  Status stop() override;
  TransportKind kind() const noexcept override { return TransportKind::ipc; }
private:
  void run();
  void receive_loop(int fd);
  std::string path_;
  bool server_;
  Configuration config_;
  ReceiveCallback callback_;
  std::mutex mutex_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<int> listener_{-1};
  std::atomic<int> peer_{-1};
  bool owns_path_{false};
};
} // namespace robotcore
