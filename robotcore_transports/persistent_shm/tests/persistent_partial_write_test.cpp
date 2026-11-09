// A process dies after acquiring a persistent SHM slot but before publishing
// the "filled" semaphore. Only the manager may recover it, after lease death.
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

static Configuration configuration() {
  Configuration c;
  c.transport = TransportKind::persistent_shared_memory;
  c.max_payload_bytes = 128;
  c.queue_depth = 1;
  return c;
}
static bool eventually(const std::function<bool()>& ready) {
  for (int i = 0; i < 300; ++i) {
    if (ready()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return ready();
}
int main() {
  const std::string name = "/robotcore-pshm-half-write-" + std::to_string(::getpid());
  PersistentRegionManager manager(name, 128);
  assert(manager.start());
  assert(manager.begin_session());
  assert(manager.epoch() == 1);
  const int fd = ::shm_open(name.c_str(), O_RDWR, 0600);
  assert(fd >= 0);
  struct stat st{};
  assert(::fstat(fd, &st) == 0);
  assert(st.st_size == static_cast<off_t>(sizeof(TestRegion) + 256));
  void* ptr = ::mmap(nullptr, static_cast<std::size_t>(st.st_size),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  assert(ptr != MAP_FAILED);
  ::close(fd);
  auto* region = static_cast<TestRegion*>(ptr);
  assert(region->magic == 0x52435031u);

  int attached[2]; assert(::pipe(attached) == 0);
  const pid_t writer = ::fork(); assert(writer >= 0);
  if (writer == 0) {
    ::close(attached[0]);
    PersistentSharedMemoryTransport client(name, false, configuration());
    if (!client.start([](Message) {})) _exit(2);
    // Simulate a crash in the middle of transport publish: hold the only
    // free slot and write an uncommitted header without posting "filled".
    auto& outbound = region->slots[0];
    if (::sem_wait(&outbound.free) != 0) _exit(3);
    outbound.epoch = manager.epoch();
    outbound.sequence = 700;
    outbound.size = 42; // deliberately incomplete payload / checksum
    const char ready = 'x';
    if (::write(attached[1], &ready, 1) != 1) _exit(4);
    ::close(attached[1]);
    for (;;) ::pause();
  }
  ::close(attached[1]);
  char ready{};
  assert(::read(attached[0], &ready, 1) == 1 && ready == 'x');
  ::close(attached[0]);
  // A live writer owns the slot; the region must NEVER be reset underneath it.
  assert(manager.begin_session().error == Error::already_started);
  assert(::kill(writer, SIGKILL) == 0);
  int status=0;
  assert(::waitpid(writer, &status, 0) == writer);
  assert(WIFSIGNALED(status) && WTERMSIG(status) == SIGKILL);
  assert(manager.begin_session());
  assert(manager.epoch() == 2);

  // The new epoch must have a clean slot; the partial frame is not delivered.
  std::atomic<int> seen{0};
  PersistentSharedMemoryTransport responder(name, true, configuration());
  PersistentSharedMemoryTransport initiator(name, false, configuration());
  assert(responder.start([&](Message message) {
    assert(message.sequence == 901);
    assert(message.payload == std::vector<std::uint8_t>({1, 2, 3}));
    assert(responder.publish(message));
  }));
  assert(initiator.start([&](Message message) {
    assert(message.sequence == 901);
    assert(message.payload == std::vector<std::uint8_t>({1, 2, 3}));
    ++seen;
  }));
  assert(initiator.publish({901, {1, 2, 3}}));
  assert(eventually([&] { return seen == 1; }));
  assert(initiator.stop());
  assert(responder.stop());
  assert(manager.stop());
  assert(::munmap(ptr, static_cast<std::size_t>(st.st_size)) == 0);
  assert(::shm_open(name.c_str(), O_RDWR, 0600) < 0);
}
