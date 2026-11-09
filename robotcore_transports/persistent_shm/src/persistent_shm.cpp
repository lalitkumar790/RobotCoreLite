#include "robotcore/persistent_shm.hpp"

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <limits>
#include <poll.h>
#include <semaphore.h>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace robotcore {
namespace {
constexpr std::uint32_t kMagic = 0x52435031; // RCP1, separate from session SHM layout.
constexpr int kWaitMs = 100;
constexpr int kPublishTimeoutMs = 2000;

struct Lease {
  std::uint32_t seen;
  std::uint32_t pid;
  std::uint64_t start_ticks;
};
struct Slot {
  sem_t free;
  sem_t filled;
  std::uint64_t epoch;
  std::uint64_t sequence;
  std::uint32_t size;
  std::uint32_t hash;
};
struct Region {
  std::uint32_t magic;
  std::uint32_t ready;
  std::uint32_t capacity;
  std::uint32_t manager_pid;
  std::uint64_t manager_start_ticks;
  std::uint64_t epoch;
  Lease leases[2];
  Slot slots[2];
};

bool valid_name(const std::string& name) {
  return name.size() > 1 && name.size() <= 200 && name[0] == '/'
      && name.find('/', 1) == std::string::npos;
}
std::size_t region_bytes(std::size_t capacity) {
  return sizeof(Region) + 2 * capacity;
}
std::uint8_t* bytes(Region* region, int index) {
  return reinterpret_cast<std::uint8_t*>(region) + sizeof(Region)
       + static_cast<std::size_t>(index) * region->capacity;
}
std::uint32_t checksum(const std::uint8_t* p, std::size_t n) {
  std::uint32_t h = 2166136261u;
  for (std::size_t i=0; i<n; ++i) { h ^= p[i]; h *= 16777619u; }
  return h;
}
struct Identity { std::uint64_t start{0}; bool alive{false}; };
Identity inspect_pid(pid_t pid) {
  if (pid <= 0) return {};
  std::ifstream f("/proc/" + std::to_string(pid) + "/stat");
  std::string stat;
  if (!std::getline(f, stat)) return {};
  const auto end_name = stat.rfind(')');
  if (end_name == std::string::npos || end_name + 2 >= stat.size()) return {};
  std::istringstream fields(stat.substr(end_name + 2));
  std::string field;
  bool live = false;
  for (int number=3; number<=22; ++number) {
    if (!(fields >> field)) return {};
    if (number == 3) live = field != "Z" && field != "X" && field != "x";
  }
  try { return {std::stoull(field), live}; } catch (...) { return {}; }
}
bool process_alive(const Lease& lease) {
  const auto state = inspect_pid(static_cast<pid_t>(lease.pid));
  return lease.pid != 0 && lease.start_ticks != 0 &&
         state.alive && state.start == lease.start_ticks;
}
bool lock_file(int fd) {
  int rc;
  do { rc = ::flock(fd, LOCK_EX); } while (rc < 0 && errno == EINTR);
  return rc == 0;
}
void unlock_file(int fd) { (void)::flock(fd, LOCK_UN); }
bool initialize_slots(Region* r) {
  int count = 0;
  for (int i=0; i<2; ++i) {
    if (::sem_init(&r->slots[i].free, 1, 1) != 0) break;
    ++count;
    if (::sem_init(&r->slots[i].filled, 1, 0) != 0) break;
    ++count;
  }
  if (count == 4) return true;
  for (int j=0; j<count; ++j) {
    auto& slot = r->slots[j / 2];
    ::sem_destroy(j % 2 == 0 ? &slot.free : &slot.filled);
  }
  return false;
}
void destroy_slots(Region* r) {
  for (auto& slot : r->slots) {
    (void)::sem_destroy(&slot.free);
    (void)::sem_destroy(&slot.filled);
  }
}
int wait_for(sem_t* semaphore, int duration_ms) {
  timespec deadline{};
  if (::clock_gettime(CLOCK_REALTIME, &deadline) != 0) return -1;
  deadline.tv_sec += duration_ms / 1000;
  deadline.tv_nsec += static_cast<long>(duration_ms % 1000) * 1000000L;
  if (deadline.tv_nsec >= 1000000000L) {
    ++deadline.tv_sec;
    deadline.tv_nsec -= 1000000000L;
  }
  int rc;
  do { rc = ::sem_timedwait(semaphore, &deadline); }
  while (rc < 0 && errno == EINTR);
  return rc;
}
bool pidfd_alive(int fd) {
  if (fd < 0) return false;
  pollfd p{}; p.fd = fd; p.events = POLLIN;
  return ::poll(&p, 1, 0) == 0;
}
bool no_live_endpoints(Region* region) {
  return !process_alive(region->leases[0]) && !process_alive(region->leases[1]);
}
} // namespace

PersistentRegionManager::PersistentRegionManager(std::string name, std::size_t capacity)
  : name_(std::move(name)), capacity_(capacity) {}
PersistentRegionManager::~PersistentRegionManager() { (void)stop(); }

Status PersistentRegionManager::start() {
  if (mapping_) return Status::failure(Error::already_started, "manager already started");
  if (!valid_name(name_) || capacity_ == 0 || capacity_ > kMaxPayloadBytes)
    return Status::failure(Error::invalid_argument, "invalid shm name or capacity");
  if (capacity_ > (SIZE_MAX - sizeof(Region)) / 2)
    return Status::failure(Error::too_large, "region size overflow");
  mapping_size_ = region_bytes(capacity_);
  fd_ = ::shm_open(name_.c_str(), O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0600);
  if (fd_ < 0) return Status::failure(Error::io_error, std::strerror(errno));
  auto rollback = [this] {
    if (mapping_) { ::munmap(mapping_, mapping_size_); mapping_ = nullptr; }
    ::close(fd_); fd_ = -1;
    ::shm_unlink(name_.c_str());
  };
  if (::ftruncate(fd_, static_cast<off_t>(mapping_size_)) != 0) {
    const std::string reason = std::strerror(errno);
    rollback();
    return Status::failure(Error::io_error, reason);
  }
  mapping_ = ::mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    const std::string reason = std::strerror(errno);
    rollback();
    return Status::failure(Error::io_error, reason);
  }
  auto* region = static_cast<Region*>(mapping_);
  std::memset(region, 0, mapping_size_);
  const auto identity = inspect_pid(::getpid());
  if (!identity.alive || !identity.start) {
    rollback();
    return Status::failure(Error::io_error, "cannot verify manager process identity");
  }
  region->magic = kMagic;
  region->capacity = static_cast<std::uint32_t>(capacity_);
  region->manager_pid = static_cast<std::uint32_t>(::getpid());
  region->manager_start_ticks = identity.start;
  if (!initialize_slots(region)) {
    rollback();
    return Status::failure(Error::io_error, "unable to initialize process-shared semaphores");
  }
  __atomic_store_n(&region->ready, 1u, __ATOMIC_RELEASE);
  return Status::success();
}

std::uint64_t PersistentRegionManager::epoch() const {
  if (!mapping_) return 0;
  const auto* region = static_cast<const Region*>(mapping_);
  return __atomic_load_n(&region->epoch, __ATOMIC_ACQUIRE);
}

Status PersistentRegionManager::begin_session() {
  if (!mapping_) return Status::failure(Error::not_started, "manager not started");
  if (!lock_file(fd_)) return Status::failure(Error::io_error, "manager lease lock failed");
  auto* region = static_cast<Region*>(mapping_);
  if (!no_live_endpoints(region)) {
    unlock_file(fd_);
    return Status::failure(Error::already_started, "previous session still has a live endpoint");
  }
  const auto old = region->epoch;
  if (old == std::numeric_limits<std::uint64_t>::max()) {
    unlock_file(fd_);
    return Status::failure(Error::io_error, "session epoch exhausted");
  }
  __atomic_store_n(&region->ready, 0u, __ATOMIC_RELEASE);
  // This is safe only after verifying both endpoints are dead/detached.
  destroy_slots(region);
  if (!initialize_slots(region)) {
    unlock_file(fd_);
    return Status::failure(Error::io_error, "failed to reset persistent semaphores");
  }
  region->leases[0] = {};
  region->leases[1] = {};
  for (auto& slot : region->slots) {
    slot.epoch = 0; slot.sequence = 0; slot.size = 0; slot.hash = 0;
  }
  __atomic_store_n(&region->epoch, old + 1, __ATOMIC_RELEASE);
  __atomic_store_n(&region->ready, 1u, __ATOMIC_RELEASE);
  unlock_file(fd_);
  return Status::success();
}

Status PersistentRegionManager::stop() {
  if (!mapping_) return Status::success();
  if (!lock_file(fd_)) return Status::failure(Error::io_error, "manager lease lock failed");
  auto* region = static_cast<Region*>(mapping_);
  if (!no_live_endpoints(region)) {
    unlock_file(fd_);
    return Status::failure(Error::already_started, "cannot destroy region while endpoints are live");
  }
  __atomic_store_n(&region->ready, 0u, __ATOMIC_RELEASE);
  destroy_slots(region);
  ::shm_unlink(name_.c_str());
  unlock_file(fd_);
  ::munmap(mapping_, mapping_size_); mapping_ = nullptr;
  ::close(fd_); fd_ = -1;
  return Status::success();
}

PersistentSharedMemoryTransport::PersistentSharedMemoryTransport(std::string name, bool server,
                                                                  Configuration config)
  : name_(std::move(name)), server_(server), config_(config) {}
PersistentSharedMemoryTransport::~PersistentSharedMemoryTransport() { (void)stop(); }

bool PersistentSharedMemoryTransport::manager_alive() const {
  if (!mapping_ || !pidfd_alive(manager_pidfd_)) return false;
  const auto* region = static_cast<const Region*>(mapping_);
  return __atomic_load_n(&region->ready, __ATOMIC_ACQUIRE) == 1u &&
         __atomic_load_n(&region->epoch, __ATOMIC_ACQUIRE) == epoch_;
}

Status PersistentSharedMemoryTransport::start(ReceiveCallback callback) {
  if (running_ || worker_.joinable())
    return Status::failure(Error::already_started, "stop required before restart");
  if (!callback || !valid_name(name_))
    return Status::failure(Error::invalid_argument, "invalid shm name or callback");
  const auto valid = validate(config_);
  if (!valid) return valid;
  if (config_.transport != TransportKind::persistent_shared_memory ||
      !config_.reliable || config_.queue_depth != 1)
    return Status::failure(Error::invalid_argument, "persistent transport requires reliable, depth=1");
  mapping_size_ = region_bytes(config_.max_payload_bytes);
  fd_ = ::shm_open(name_.c_str(), O_RDWR | O_CLOEXEC, 0600);
  if (fd_ < 0) return Status::failure(Error::io_error, std::strerror(errno));
  struct stat st{};
  if (::fstat(fd_, &st) != 0 || st.st_size != static_cast<off_t>(mapping_size_)) {
    ::close(fd_); fd_ = -1;
    return Status::failure(Error::io_error, "shared-memory layout/capacity mismatch");
  }
  mapping_ = ::mmap(nullptr, mapping_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    const std::string reason = std::strerror(errno);
    ::close(fd_); fd_ = -1;
    return Status::failure(Error::io_error, reason);
  }
  auto* region = static_cast<Region*>(mapping_);
  if (!lock_file(fd_)) {
    release_resources();
    return Status::failure(Error::io_error, "cannot lock persistent session");
  }
  const auto manager_pid = static_cast<pid_t>(region->manager_pid);
  const auto manager_start = region->manager_start_ticks;
  const auto state = inspect_pid(manager_pid);
  manager_pidfd_ = static_cast<int>(::syscall(SYS_pidfd_open, manager_pid, 0u));
  if (region->magic != kMagic || region->capacity != config_.max_payload_bytes ||
      !manager_start || !state.alive || state.start != manager_start ||
      !pidfd_alive(manager_pidfd_) ||
      __atomic_load_n(&region->ready, __ATOMIC_ACQUIRE) != 1u ||
      region->epoch == 0) {
    unlock_file(fd_);
    release_resources();
    return Status::failure(Error::closed, "manager inactive or no initialized session");
  }
  const int role = server_ ? 0 : 1;
  if (region->leases[role].seen != 0) {
    unlock_file(fd_);
    release_resources();
    return Status::failure(Error::already_started, "endpoint role already used in this epoch");
  }
  const auto me = inspect_pid(::getpid());
  if (!me.alive || !me.start) {
    unlock_file(fd_);
    release_resources();
    return Status::failure(Error::io_error, "cannot verify endpoint identity");
  }
  epoch_ = __atomic_load_n(&region->epoch, __ATOMIC_ACQUIRE);
  identity_ticks_ = me.start;
  region->leases[role] = {1u, static_cast<std::uint32_t>(::getpid()), identity_ticks_};
  unlock_file(fd_);
  callback_ = std::move(callback);
  running_ = true;
  try {
    worker_ = std::thread([this] { receive_loop(); });
  } catch (...) {
    running_ = false;
    callback_ = {};
    // The role is intentionally fenced for the rest of this session.
    release_resources();
    return Status::failure(Error::io_error, "failed to start persistent receiver");
  }
  return Status::success();
}

Status PersistentSharedMemoryTransport::publish(const Message& message) {
  if (!mapping_) return Status::failure(Error::not_started, "transport not attached");
  if (!manager_alive()) return Status::failure(Error::closed, "manager or session ended");
  if (!running_) return Status::failure(Error::not_started, "transport is stopped");
  const auto valid = validate_message(message, config_);
  if (!valid) return valid;
  auto* region = static_cast<Region*>(mapping_);
  auto& slot = region->slots[server_ ? 1 : 0];
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(kPublishTimeoutMs);
  bool acquired = false;
  while (running_ && std::chrono::steady_clock::now() < deadline) {
    if (!manager_alive()) return Status::failure(Error::closed, "manager or session ended");
    if (wait_for(&slot.free, kWaitMs) == 0) { acquired = true; break; }
    if (errno != ETIMEDOUT && errno != EINTR)
      return Status::failure(Error::io_error, "free-slot semaphore failed");
  }
  if (!acquired) return Status::failure(Error::io_error, "slot unavailable or peer stalled");
  if (!running_ || !manager_alive()) {
    ::sem_post(&slot.free);
    return Status::failure(Error::closed, "session ended after slot acquisition");
  }
  slot.epoch = epoch_;
  slot.sequence = message.sequence;
  slot.size = static_cast<std::uint32_t>(message.payload.size());
  auto* ptr = bytes(region, server_ ? 1 : 0);
  if (!message.payload.empty()) std::memcpy(ptr, message.payload.data(), message.payload.size());
  slot.hash = checksum(ptr, message.payload.size());
  if (::sem_post(&slot.filled) != 0)
    return Status::failure(Error::io_error, "filled-slot semaphore failed");
  return Status::success();
}

void PersistentSharedMemoryTransport::receive_loop() {
  auto* region = static_cast<Region*>(mapping_);
  const int index = server_ ? 0 : 1;
  auto& slot = region->slots[index];
  while (running_ && manager_alive()) {
    if (wait_for(&slot.filled, kWaitMs) != 0) {
      if (errno == ETIMEDOUT || errno == EINTR) continue;
      break;
    }
    if (!running_ || !manager_alive()) {
      ::sem_post(&slot.free);
      break;
    }
    const auto count = slot.size;
    bool good = slot.epoch == epoch_ && count <= region->capacity &&
                count <= config_.max_payload_bytes;
    Message message;
    if (good) {
      message.sequence = slot.sequence;
      auto* ptr = bytes(region, index);
      good = slot.hash == checksum(ptr, count);
      if (good) message.payload.assign(ptr, ptr + count);
    }
    ::sem_post(&slot.free);
    if (!good) {
      // Poison the whole session. If only the receiver stopped, other peers
      // could continue enqueueing into an abandoned slot indefinitely.
      // A manager may reset this region only after both leases are inactive.
      __atomic_store_n(&region->ready, 2u, __ATOMIC_RELEASE);
      running_ = false;
      break;
    }
    try {
      callback_(std::move(message));
    } catch (...) {
      // A crashing application callback invalidates the current session too.
      __atomic_store_n(&region->ready, 2u, __ATOMIC_RELEASE);
      running_ = false;
      break;
    }
  }
  running_ = false;
}

void PersistentSharedMemoryTransport::release_resources() {
  if (mapping_) { ::munmap(mapping_, mapping_size_); mapping_ = nullptr; }
  if (manager_pidfd_ >= 0) { ::close(manager_pidfd_); manager_pidfd_ = -1; }
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}
Status PersistentSharedMemoryTransport::stop() {
  if (worker_.joinable() && worker_.get_id() == std::this_thread::get_id())
    return Status::failure(Error::invalid_argument, "stop forbidden inside callback");
  running_ = false;
  if (worker_.joinable()) worker_.join();
  if (mapping_ && fd_ >= 0 && lock_file(fd_)) {
    auto* region = static_cast<Region*>(mapping_);
    const int role = server_ ? 0 : 1;
    auto& lease = region->leases[role];
    if (region->epoch == epoch_ && lease.pid == static_cast<std::uint32_t>(::getpid()) &&
        lease.start_ticks == identity_ticks_) {
      lease.pid = 0;
      lease.start_ticks = 0;
      // "seen" deliberately remains set until manager begins next epoch.
    }
    unlock_file(fd_);
  }
  callback_ = {};
  release_resources();
  epoch_ = 0;
  identity_ticks_ = 0;
  return Status::success();
}
} // namespace robotcore
