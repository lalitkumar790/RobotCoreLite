#pragma once
#include "robotcore/core.hpp"
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <mutex>
#include <functional>
#include <utility>
namespace robotcore {
// Linux POSIX shm with one server/client and two directional, bounded slots.
// stop() coordinates with concurrent publish() before unmapping. Do not stop in a callback.
// start() and simultaneous stop() calls still require external serialization.
// Loaned-buffer APIs are local-only, experimental, and NOT a ROS middleware replacement.
// Borrowed bytes remain valid ONLY while the receive callback is executing.
// The slot is not returned to the producer until that callback returns.
struct BorrowedMessage {
  std::uint64_t sequence{0};
  const std::uint8_t* data{nullptr};
  std::size_t size{0};
};
using BorrowedReceiveCallback = std::function<void(BorrowedMessage)>;

class SharedMemoryTransport final : public Transport {
public:
  // Move-only writer reservation. Holding it prevents stop() from unmapping
  // the region; release/discard before calling stop() on the SAME thread.
  // It must never outlive the SharedMemoryTransport instance.
  class WriteLoan {
  public:
    WriteLoan() = default;
    ~WriteLoan() { discard(); }
    WriteLoan(const WriteLoan&) = delete;
    WriteLoan& operator=(const WriteLoan&) = delete;
    WriteLoan(WriteLoan&& other) noexcept;
    WriteLoan& operator=(WriteLoan&& other) noexcept;

    explicit operator bool() const noexcept { return owner_ != nullptr; }
    std::uint8_t* data() const noexcept { return data_; }
    std::size_t capacity() const noexcept { return capacity_; }
    // A successful commit consumes the reservation; do not use data() after.
    Status commit(std::uint64_t sequence, std::size_t size);
    // Return an unwritten slot. Safe to call repeatedly.
    void discard() noexcept;
  private:
    friend class SharedMemoryTransport;
    WriteLoan(SharedMemoryTransport* owner, std::unique_lock<std::mutex>&& lock,
              void* slot, std::uint8_t* data, std::size_t capacity)
      : owner_(owner), lock_(std::move(lock)), slot_(slot),
        data_(data), capacity_(capacity) {}
    SharedMemoryTransport* owner_{nullptr};
    std::unique_lock<std::mutex> lock_;
    void* slot_{nullptr};
    std::uint8_t* data_{nullptr};
    std::size_t capacity_{0};
  };

  SharedMemoryTransport(std::string name, bool server, Configuration config);
  ~SharedMemoryTransport() override;
  SharedMemoryTransport(const SharedMemoryTransport&) = delete;
  SharedMemoryTransport& operator=(const SharedMemoryTransport&) = delete;
  Status start(ReceiveCallback callback) override;
  // Receive directly from the shared slot without constructing a payload
  // vector. Callback MUST NOT retain data, block indefinitely, or call stop().
  Status start_borrowed(BorrowedReceiveCallback callback);
  // Reserve a writable shared slot. Write directly into loan.data() then
  // commit; abandoning the loan returns the slot automatically.
  Status acquire_write_loan(WriteLoan& loan);
  Status publish(const Message& message) override;
  Status stop() override;
  TransportKind kind() const noexcept override { return TransportKind::shared_memory; }
private:
  Status start_impl(ReceiveCallback callback, BorrowedReceiveCallback borrowed);
  void receive_loop();
  bool owner_alive() const;
  void release_resources();
  std::string name_;
  bool server_;
  Configuration config_;
  ReceiveCallback callback_;
  BorrowedReceiveCallback borrowed_callback_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  // Serialize publishes and prevent stop() from unmapping a live publisher.
  // Do NOT hold this mutex while joining the receiver thread: a callback may publish.
  std::mutex publish_mutex_;
  void* mapping_{nullptr};
  std::size_t mapping_size_{0};
  bool owns_name_{false};
  bool owns_client_claim_{false};
  int owner_pidfd_{-1};
};
} // namespace robotcore
