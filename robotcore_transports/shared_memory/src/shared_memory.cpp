#include "robotcore/shared_memory.hpp"
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace robotcore {
namespace {
constexpr std::uint32_t kMagic = 0x52435331; // RCS1, layout version one.
constexpr int kPollMs = 100;
constexpr int kWriteTimeoutMs = 2000;

struct Slot {
  sem_t free;
  sem_t filled;
  std::uint64_t seq;
  std::uint32_t size;
  std::uint32_t hash;
};
struct Region {
  std::uint32_t magic;
  std::uint32_t ready;
  std::uint32_t capacity;
  std::uint32_t client_claim;
  Slot slots[2];
};
std::uint8_t* bytes(Region* region, int slot) {
  return reinterpret_cast<std::uint8_t*>(region) + sizeof(Region)
       + static_cast<std::size_t>(slot) * region->capacity;
}
std::uint32_t hash_payload(const std::uint8_t* ptr, std::size_t n) {
  std::uint32_t h = 2166136261u;
  for (std::size_t i = 0; i < n; ++i) { h ^= ptr[i]; h *= 16777619u; }
  return h;
}
// sem_timedwait expects a CLOCK_REALTIME absolute deadline.
int wait_one(sem_t* sem, int timeout_ms) {
  timespec deadline{};
  if (::clock_gettime(CLOCK_REALTIME, &deadline) != 0) return -1;
  deadline.tv_sec += timeout_ms / 1000;
  deadline.tv_nsec += static_cast<long>(timeout_ms % 1000) * 1000000L;
  if (deadline.tv_nsec >= 1000000000L) {
    ++deadline.tv_sec;
    deadline.tv_nsec -= 1000000000L;
  }
  int result;
  do { result = ::sem_timedwait(sem, &deadline); } while (result < 0 && errno == EINTR);
  return result;
}
bool valid_name(const std::string& name) {
  return name.size() > 1 && name.size() <= 200 && name[0] == '/'
      && name.find('/', 1) == std::string::npos;
}
} // namespace

SharedMemoryTransport::SharedMemoryTransport(std::string name, bool server,
                                             Configuration config)
  : name_(std::move(name)), server_(server), config_(config) {}
SharedMemoryTransport::~SharedMemoryTransport() { (void)stop(); }

Status SharedMemoryTransport::start(ReceiveCallback callback) {
  if (running_ || worker_.joinable())
    return Status::failure(Error::already_started, "stop required before restart");
  const auto valid = validate(config_);
  if (!valid) return valid;
  if (config_.transport != TransportKind::shared_memory ||
      config_.queue_depth != 1 || !config_.reliable)
    return Status::failure(Error::invalid_argument,
                           "shared memory currently requires depth=1 and reliable=true");
  if (!valid_name(name_) || !callback)
    return Status::failure(Error::invalid_argument, "invalid shm name or missing callback");
  if (config_.max_payload_bytes > (SIZE_MAX - sizeof(Region)) / 2)
    return Status::failure(Error::too_large, "region size overflow");
  mapping_size_ = sizeof(Region) + 2 * config_.max_payload_bytes;
  const int flags = O_RDWR | O_CLOEXEC | (server_ ? (O_CREAT | O_EXCL) : 0);
  const int fd = ::shm_open(name_.c_str(), flags, S_IRUSR | S_IWUSR);
  if (fd < 0) return Status::failure(Error::io_error, std::strerror(errno));
  owns_name_ = server_;
  if (server_) {
    if (::ftruncate(fd, static_cast<off_t>(mapping_size_)) != 0) {
      const auto error = std::string(std::strerror(errno));
      ::close(fd); ::shm_unlink(name_.c_str()); owns_name_ = false;
      return Status::failure(Error::io_error, error);
    }
  } else {
    struct stat st{};
    if (::fstat(fd, &st) != 0 || st.st_size != static_cast<off_t>(mapping_size_)) {
      ::close(fd);
      return Status::failure(Error::io_error, "shared region size mismatch or not ready");
    }
  }
  mapping_ = ::mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  const int map_errno = errno;
  ::close(fd);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    if (owns_name_) { ::shm_unlink(name_.c_str()); owns_name_ = false; }
    return Status::failure(Error::io_error, std::strerror(map_errno));
  }
  auto* region = static_cast<Region*>(mapping_);
  if (server_) {
    std::memset(region, 0, mapping_size_);
    region->magic = kMagic;
    region->capacity = static_cast<std::uint32_t>(config_.max_payload_bytes);
    int initialized = 0;
    for (int i = 0; i < 2; ++i) {
      if (::sem_init(&region->slots[i].free, 1, 1) != 0) break;
      ++initialized;
      if (::sem_init(&region->slots[i].filled, 1, 0) != 0) break;
      ++initialized;
    }
    if (initialized != 4) {
      for (int i = 0; i < initialized; ++i) {
        auto& slot = region->slots[i / 2];
        ::sem_destroy(i % 2 == 0 ? &slot.free : &slot.filled);
      }
      release_resources();
      return Status::failure(Error::io_error, "sem_init failed");
    }
    __atomic_store_n(&region->ready, 1u, __ATOMIC_RELEASE);
  } else {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (__atomic_load_n(&region->ready, __ATOMIC_ACQUIRE) != 1u &&
           std::chrono::steady_clock::now() < deadline)
      ::usleep(1000);
    if (__atomic_load_n(&region->ready, __ATOMIC_ACQUIRE) != 1u ||
        region->magic != kMagic || region->capacity != config_.max_payload_bytes) {
      release_resources();
      return Status::failure(Error::io_error, "incompatible or uninitialized region");
    }
    // Enforce one client. A crashed client leaves the claim set: fail closed.
    std::uint32_t expected = 0;
    if (!__atomic_compare_exchange_n(&region->client_claim, &expected, 1u,
                                     false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
      release_resources();
      return Status::failure(Error::already_started, "client already attached");
    }
    owns_client_claim_ = true;
  }
  callback_ = std::move(callback);
  running_ = true;
  try {
    worker_ = std::thread([this] { receive_loop(); });
  } catch (...) {
    running_ = false;
    callback_ = {};
    release_resources();
    return Status::failure(Error::io_error, "unable to start receive worker");
  }
  return Status::success();
}

Status SharedMemoryTransport::publish(const Message& message) {
  if (!running_ || !mapping_)
    return Status::failure(Error::not_started, "shared-memory endpoint not started");
  const auto valid = validate_message(message, config_);
  if (!valid) return valid;
  auto* region = static_cast<Region*>(mapping_);
  const int index = server_ ? 1 : 0;
  auto& slot = region->slots[index];
  const auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::milliseconds(kWriteTimeoutMs);
  bool acquired = false;
  while (running_ && std::chrono::steady_clock::now() < deadline) {
    if (wait_one(&slot.free, kPollMs) == 0) { acquired = true; break; }
    if (errno != ETIMEDOUT && errno != EINTR)
      return Status::failure(Error::io_error, "shared-memory free semaphore error");
  }
  if (!acquired)
    return Status::failure(Error::io_error, "shared-memory slot unavailable (peer stalled or stopped)");
  // The producer owns this slot until posting filled. The peer cannot read it
  // until then; the next producer cannot reuse it until consumption completes.
  slot.seq = message.sequence;
  slot.size = static_cast<std::uint32_t>(message.payload.size());
  std::uint8_t* ptr = bytes(region, index);
  if (!message.payload.empty()) std::memcpy(ptr, message.payload.data(), message.payload.size());
  slot.hash = hash_payload(ptr, message.payload.size());
  if (::sem_post(&slot.filled) != 0)
    return Status::failure(Error::io_error, "shared-memory filled semaphore error");
  return Status::success();
}

void SharedMemoryTransport::receive_loop() {
  auto* region = static_cast<Region*>(mapping_);
  const int index = server_ ? 0 : 1;
  auto& slot = region->slots[index];
  while (running_) {
    if (wait_one(&slot.filled, kPollMs) != 0) {
      if (errno == ETIMEDOUT || errno == EINTR) continue;
      break;
    }
    if (!running_) { ::sem_post(&slot.free); break; }
    const auto size = slot.size;
    bool valid = size <= region->capacity && size <= config_.max_payload_bytes;
    Message message;
    if (valid) {
      message.sequence = slot.seq;
      auto* ptr = bytes(region, index);
      valid = slot.hash == hash_payload(ptr, size);
      if (valid) message.payload.assign(ptr, ptr + size);
    }
    ::sem_post(&slot.free);
    if (!valid) { running_ = false; break; }
    try { callback_(std::move(message)); } catch (...) { running_ = false; break; }
  }
}

void SharedMemoryTransport::release_resources() {
  if (mapping_) {
    if (owns_client_claim_) {
      auto* region = static_cast<Region*>(mapping_);
      __atomic_store_n(&region->client_claim, 0u, __ATOMIC_RELEASE);
      owns_client_claim_ = false;
    }
    ::munmap(mapping_, mapping_size_);
    mapping_ = nullptr;
  }
  if (owns_name_) {
    ::shm_unlink(name_.c_str());
    owns_name_ = false;
  }
}
Status SharedMemoryTransport::stop() {
  if (worker_.joinable() && worker_.get_id() == std::this_thread::get_id())
    return Status::failure(Error::invalid_argument, "stop forbidden from receive callback");
  running_ = false;
  if (worker_.joinable()) worker_.join();
  callback_ = {};
  release_resources();
  return Status::success();
}
} // namespace robotcore
