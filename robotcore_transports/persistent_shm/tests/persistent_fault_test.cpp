#include "robotcore/persistent_shm.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <fcntl.h>
#include <functional>
#include <semaphore.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace robotcore;
using namespace std::chrono_literals;

static Configuration config() {
  Configuration c;
  c.transport = TransportKind::persistent_shared_memory;
  c.max_payload_bytes = 64;
  c.queue_depth = 1;
  return c;
}
static bool eventually(const std::function<bool()>& predicate) {
  for (int i = 0; i < 300; ++i) {
    if (predicate()) return true;
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

// Mirrors the RCP1 layout solely for deliberate fault injection. A future
// layout change must update this test, not silently skip memory corruption.
struct TestLease {
  std::uint32_t seen, pid;
  std::uint64_t start_ticks;
};
struct TestSlot {
  sem_t free, filled;
  std::uint64_t epoch, sequence;
  std::uint32_t size, hash;
};
struct TestRegion {
  std::uint32_t magic, ready, capacity, manager_pid;
  std::uint64_t manager_start_ticks, epoch;
  TestLease leases[2];
  TestSlot slots[2];
};
static std::uint8_t* data(TestRegion* r, int index) {
  return reinterpret_cast<std::uint8_t*>(r) + sizeof(TestRegion) +
         static_cast<std::size_t>(index) * r->capacity;
}
static std::uint32_t checksum(const std::uint8_t* p, std::size_t len) {
  std::uint32_t h = 2166136261u;
  for (std::size_t i=0; i<len; ++i) { h ^= p[i]; h *= 16777619u; }
  return h;
}

static void manager_crash_is_detected() {
  const std::string name = "/robotcore-pshm-manager-death-" + std::to_string(::getpid());
  int ready_pipe[2]; assert(::pipe(ready_pipe) == 0);
  const pid_t manager_pid = ::fork();
  assert(manager_pid >= 0);
  if (manager_pid == 0) {
    ::close(ready_pipe[0]);
    PersistentRegionManager manager(name, 64);
    if (!manager.start() || !manager.begin_session()) _exit(2);
    const char ready='r';
    if (::write(ready_pipe[1], &ready, 1) != 1) _exit(3);
    ::close(ready_pipe[1]);
    while (true) ::pause();
  }
  ::close(ready_pipe[1]);
  char ack = 0;
  assert(::read(ready_pipe[0], &ack, 1) == 1 && ack == 'r');
  ::close(ready_pipe[0]);
  PersistentSharedMemoryTransport responder(name, true, config());
  PersistentSharedMemoryTransport initiator(name, false, config());
  std::atomic<bool> got_echo{false};
  assert(responder.start([&](Message msg) { assert(responder.publish(msg)); }));
  assert(initiator.start([&](Message msg) {
    if (msg.sequence == 7 && msg.payload == std::vector<std::uint8_t>{1, 2, 3})
      got_echo = true;
  }));
  assert(initiator.publish({7, {1, 2, 3}}));
  assert(eventually([&] { return got_echo.load(); }));

  assert(::kill(manager_pid, SIGKILL) == 0);
  int status = 0;
  assert(::waitpid(manager_pid, &status, 0) == manager_pid);
  assert(WIFSIGNALED(status) && WTERMSIG(status) == SIGKILL);
  assert(eventually([&] {
    return initiator.publish({8, {4}}).error == Error::closed;
  }));
  assert(responder.publish({9, {4}}).error == Error::closed);
  assert(initiator.stop());
  assert(responder.stop());

  // Dead manager's name cannot be hijacked or silently reset by another
  // manager. Cleanup is test-only, after reaping and detaching both endpoints.
  PersistentSharedMemoryTransport stale(name, false, config());
  assert(stale.start([](Message) {}).error == Error::closed);
  PersistentRegionManager replacement(name, 64);
  assert(replacement.start().error == Error::io_error);
  assert(::shm_unlink(name.c_str()) == 0);
  assert(replacement.start());
  assert(replacement.begin_session());
  assert(replacement.stop());
}

static void malformed_frames_poison_epoch() {
  const std::string name = "/robotcore-pshm-corrupt-" + std::to_string(::getpid());
  PersistentRegionManager manager(name, 64);
  assert(manager.start());
  int fd = ::shm_open(name.c_str(), O_RDWR, 0600);
  assert(fd >= 0);
  struct stat st{};
  assert(::fstat(fd, &st) == 0);
  assert(st.st_size == static_cast<off_t>(sizeof(TestRegion) + 2 * 64));
  void* mapped = ::mmap(nullptr, static_cast<std::size_t>(st.st_size),
                        PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  assert(mapped != MAP_FAILED);
  ::close(fd);
  auto* region = static_cast<TestRegion*>(mapped);
  assert(region->magic == 0x52435031);
  assert(region->capacity == 64);

  for (int fault = 0; fault < 3; ++fault) {
    assert(manager.begin_session());
    std::atomic<int> delivered{0};
    PersistentSharedMemoryTransport server(name, true, config());
    assert(server.start([&](Message) { ++delivered; }));
    TestSlot& slot = region->slots[0];  // client -> server receive direction
    assert(::sem_wait(&slot.free) == 0);
    slot.epoch = manager.epoch();
    slot.sequence = 42;
    slot.size = 1;
    data(region, 0)[0] = 7;
    slot.hash = checksum(data(region, 0), 1);
    if (fault == 0) slot.size = 65;             // oversize length
    if (fault == 1) slot.epoch = manager.epoch()-1;  // stale generation
    if (fault == 2) slot.hash ^= 0x1000u;       // corruption
    assert(::sem_post(&slot.filled) == 0);

    assert(eventually([&] {
      return __atomic_load_n(&region->ready, __ATOMIC_ACQUIRE) == 2u;
    }));
    assert(server.publish({50, {9}}).error == Error::closed);
    assert(delivered.load() == 0);
    PersistentSharedMemoryTransport late(name, false, config());
    assert(late.start([](Message) {}).error == Error::closed);
    assert(server.stop());
    // Explicit manager reset discards the poisoned frame and increments epoch.
  }
  assert(::munmap(mapped, static_cast<std::size_t>(st.st_size)) == 0);
  assert(manager.stop());
  assert(::shm_open(name.c_str(), O_RDWR, 0600) < 0);
}

int main() {
  manager_crash_is_detected();
  malformed_frames_poison_epoch();
}
