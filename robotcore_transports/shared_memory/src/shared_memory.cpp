#include "robotcore/shared_memory.hpp"
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/syscall.h>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace robotcore {
namespace {
constexpr std::uint32_t kMagic = 0x52435333; // RCS3: active callback/session cleanliness tracking.
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
  std::uint32_t owner_pid;
  std::uint32_t reserved;
  std::uint64_t owner_start_ticks;
  std::uint32_t callbacks_active[2];
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

// Linux PID alone is not sufficient: a crashed owner's PID may be recycled.
// /proc/<pid>/stat field 22 identifies its process start time in clock ticks.
std::uint64_t start_ticks(pid_t pid) {
  std::ifstream proc("/proc/" + std::to_string(pid) + "/stat");
  std::string stat;
  if (!std::getline(proc, stat)) return 0;
  const auto end_name = stat.rfind(')');
  if (end_name == std::string::npos || end_name + 2 >= stat.size()) return 0;
  std::istringstream fields(stat.substr(end_name + 2));
  std::string token;
  for (int field = 3; field <= 22; ++field) {
    if (!(fields >> token)) return 0;
  }
  try { return std::stoull(token); } catch (...) { return 0; }
}
bool pidfd_alive(int fd) {
  if (fd < 0) return false;
  pollfd p{}; p.fd = fd; p.events = POLLIN;
  // A pidfd becomes readable when that specific process exits.
  const int ready = ::poll(&p, 1, 0);
  return ready == 0;
}
// Fail closed when an earlier client left a frame or callback in flight.
// sem_getvalue is only a snapshot, so this relies on documented non-concurrent
// client stop/publish and the atomic exclusive client claim.
bool clean_slots(Region* region) {
  for (int i = 0; i < 2; ++i) {
    if (__atomic_load_n(&region->callbacks_active[i], __ATOMIC_ACQUIRE) != 0u)
      return false;
    int free_count = -1, filled_count = -1;
    auto& slot = region->slots[i];
    if (::sem_getvalue(&slot.free, &free_count) != 0 ||
        ::sem_getvalue(&slot.filled, &filled_count) != 0 ||
        free_count != 1 || filled_count != 0)
      return false;
  }
  return true;
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
    region->owner_pid = static_cast<std::uint32_t>(::getpid());
    region->owner_start_ticks = start_ticks(::getpid());
    if (!region->owner_start_ticks) {
      release_resources();
      return Status::failure(Error::io_error, "could not establish server process identity");
    }
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
    // pin the exact server process before accepting its semaphores.
    // Fail closed if PID was reused or the server has already exited.
    const auto pid = static_cast<pid_t>(region->owner_pid);
    const auto expected_ticks = region->owner_start_ticks;
    owner_pidfd_ = static_cast<int>(::syscall(SYS_pidfd_open, pid, 0u));
    if (pid <= 0 || expected_ticks == 0 || !pidfd_alive(owner_pidfd_) ||
        start_ticks(pid) != expected_ticks || 
        __atomic_load_n(&region->ready, __ATOMIC_ACQUIRE) != 1u) {
      release_resources();
      return Status::failure(Error::closed, "shared-memory server identity is no longer active");
    }
    // Enforce one client. A crashed client leaves the claim set: fail closed.
    std::uint32_t expected = 0;
    if (!__atomic_compare_exchange_n(&region->client_claim, &expected, 1u,
                                     false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
      release_resources();
      return Status::failure(Error::already_started, "client already attached");
    }
    owns_client_claim_ = true;
    if (!owner_alive()) {
      release_resources();
      return Status::failure(Error::closed, "shared-memory server closed during attach");
    }
    // Permit reattachment only when the prior session's two slots are idle
    // and neither receiver is still executing the prior callback.
    const auto quiesce_deadline = std::chrono::steady_clock::now() +
                                  std::chrono::milliseconds(200);
    while (owner_alive() && !clean_slots(region) &&
           std::chrono::steady_clock::now() < quiesce_deadline)
      ::usleep(1000);
    if (!owner_alive() || !clean_slots(region)) {
      const auto closed = !owner_alive();
      release_resources();
      return Status::failure(closed ? Error::closed : Error::io_error,
                             closed ? "shared-memory server exited during attach"
                                    : "previous session has pending data or active callbacks");
    }
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
  // Keep mapping, pidfd, slot semaphores and session data alive until
  // the publish attempt finishes, including semaphore wait timeout paths.
  std::lock_guard<std::mutex> publish_lock(publish_mutex_);
  if (!mapping_) return Status::failure(Error::not_started, "shared-memory endpoint not started");
  if (!server_ && !owner_alive())
    return Status::failure(Error::closed, "shared-memory server exited or closed");
  if (!running_)
    return Status::failure(Error::not_started, "shared-memory endpoint not running");
  const auto valid = validate_message(message, config_);
  if (!valid) return valid;
  auto* region = static_cast<Region*>(mapping_);
  const int index = server_ ? 1 : 0;
  auto& slot = region->slots[index];
  const auto deadline = std::chrono::steady_clock::now()
                      + std::chrono::milliseconds(kWriteTimeoutMs);
  bool acquired = false;
  while (running_ && std::chrono::steady_clock::now() < deadline) {
    if (!server_ && !owner_alive())
      return Status::failure(Error::closed, "shared-memory server exited or closed");
    if (wait_one(&slot.free, kPollMs) == 0) { acquired = true; break; }
    if (errno != ETIMEDOUT && errno != EINTR)
      return Status::failure(Error::io_error, "shared-memory free semaphore error");
  }
  if (!acquired)
    return !server_ && !owner_alive()
      ? Status::failure(Error::closed, "shared-memory server exited or closed")
      : Status::failure(Error::io_error, "shared-memory slot unavailable (peer stalled or stopped)");
  if (!server_ && !owner_alive()) {
    ::sem_post(&slot.free);
    return Status::failure(Error::closed, "shared-memory server closed after slot acquisition");
  }
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
    if (!server_ && !owner_alive()) { running_ = false; break; }
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
    // A message is copied before slot reuse, but a callback may still
    // generate a response. Fence new clients until that callback finishes.
    if (valid)
      __atomic_store_n(&region->callbacks_active[index], 1u, __ATOMIC_RELEASE);
    ::sem_post(&slot.free);
    if (!valid) {
      if (server_) __atomic_store_n(&region->ready, 3u, __ATOMIC_RELEASE);
      running_ = false;
      break;
    }
    try {
      callback_(std::move(message));
    } catch (...) {
      if (server_) __atomic_store_n(&region->ready, 3u, __ATOMIC_RELEASE);
      running_ = false;
    }
    __atomic_store_n(&region->callbacks_active[index], 0u, __ATOMIC_RELEASE);
    if (!running_) break;
  }
}

bool SharedMemoryTransport::owner_alive() const {
  if (server_) return true;
  if (!mapping_ || !pidfd_alive(owner_pidfd_)) return false;
  const auto* region = static_cast<const Region*>(mapping_);
  return __atomic_load_n(&region->ready, __ATOMIC_ACQUIRE) == 1u;
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
  if (owner_pidfd_ >= 0) {
    ::close(owner_pidfd_);
    owner_pidfd_ = -1;
  }
  if (owns_name_) {
    ::shm_unlink(name_.c_str());
    owns_name_ = false;
  }
}
Status SharedMemoryTransport::stop() {
  if (worker_.joinable() && worker_.get_id() == std::this_thread::get_id())
    return Status::failure(Error::invalid_argument, "stop forbidden from receive callback");
  if (server_ && mapping_) {
    auto* region = static_cast<Region*>(mapping_);
    // A clean owner shutdown is distinct from process death; advertise it
    // before the region is unmapped so clients fail closed rather than stall.
    __atomic_store_n(&region->ready, 2u, __ATOMIC_RELEASE);
  }
  running_ = false;
  if (worker_.joinable()) worker_.join();
  callback_ = {};
  // Receiver is joined first; it may publish from a callback. Only after
  // its exit can we wait for in-flight publishers and release the mapping.
  {
    std::lock_guard<std::mutex> publish_lock(publish_mutex_);
    release_resources();
  }
  return Status::success();
}
} // namespace robotcore
