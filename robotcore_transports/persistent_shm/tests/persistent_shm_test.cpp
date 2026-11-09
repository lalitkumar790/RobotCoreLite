#include "robotcore/persistent_shm.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace robotcore;
using namespace std::chrono_literals;

static bool eventually(const std::function<bool()>& check) {
  for (int i=0; i<400; ++i) {
    if (check()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return check();
}
static Configuration settings() {
  Configuration c;
  c.transport = TransportKind::persistent_shared_memory;
  c.max_payload_bytes = 128;
  c.queue_depth = 1;
  return c;
}
static void session_round_trip(PersistentRegionManager& manager,
                               const std::string& name,
                               int session, bool crash_client) {
  const auto expected_epoch = manager.epoch();
  int ready[2];
  assert(::pipe(ready) == 0);
  const pid_t child = ::fork();
  assert(child >= 0);
  if (child == 0) {
    ::close(ready[1]);
    char flag = 0;
    if (::read(ready[0], &flag, 1) != 1) _exit(10);
    ::close(ready[0]);
    PersistentSharedMemoryTransport client(name, false, settings());
    std::atomic<int> seen{0};
    if (!client.start([&](Message msg) {
      if (msg.sequence >= static_cast<std::uint64_t>(session * 1000) &&
          msg.sequence < static_cast<std::uint64_t>(session * 1000 + 30) &&
          msg.payload == std::vector<std::uint8_t>(64, static_cast<std::uint8_t>(session)))
        ++seen;
    })) _exit(11);
    if (client.session_epoch() != expected_epoch) _exit(12);
    if (client.publish({12345, std::vector<std::uint8_t>(129)}).error != Error::too_large)
      _exit(13);
    const int total = crash_client ? 1 : 30;
    for (int i=0; i<total; ++i) {
      if (!client.publish({static_cast<std::uint64_t>(session * 1000 + i),
                          std::vector<std::uint8_t>(64, static_cast<std::uint8_t>(session))}))
        _exit(14);
    }
    if (crash_client) _exit(0); // Intentionally keep a lease and possibly a pending response.
    if (!eventually([&] { return seen == 30; })) _exit(15);
    if (!client.stop()) _exit(16);
    _exit(0);
  }
  ::close(ready[0]);
  PersistentSharedMemoryTransport server(name, true, settings());
  std::atomic<int> requests{0};
  assert(server.start([&](Message msg) {
    if (msg.payload == std::vector<std::uint8_t>(64, static_cast<std::uint8_t>(session))) {
      ++requests;
      assert(server.publish(msg));
    }
  }));
  assert(server.session_epoch() == expected_epoch);
  // A manager must not reset/recycle a session with a live endpoint.
  assert(manager.begin_session().error == Error::already_started);
  assert(manager.stop().error == Error::already_started);
  assert(::write(ready[1], "r", 1) == 1);
  ::close(ready[1]);
  int status=0;
  assert(::waitpid(child, &status, 0) == child);
  assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  if (!crash_client) assert(requests.load() == 30);
  else assert(eventually([&] { return requests.load() >= 1; }));
  assert(server.stop());
  // All endpoint roles are now detached or dead; only manager may reset slots.
  assert(manager.begin_session());
  assert(manager.epoch() == expected_epoch + 1);
}
int main() {
  const auto name = "/robotcore-persistent-test-" + std::to_string(::getpid());
  PersistentRegionManager manager(name, 128);
  assert(manager.start());
  PersistentRegionManager duplicate(name, 128);
  assert(duplicate.start().error == Error::io_error);
  assert(manager.begin_session());
  assert(manager.epoch() == 1);
  // The manager-owned region survives these endpoint sessions without reallocation.
  int fd = ::shm_open(name.c_str(), O_RDWR, 0600);
  assert(fd >= 0);
  struct stat before{};
  assert(::fstat(fd, &before) == 0);
  ::close(fd);
  session_round_trip(manager, name, 1, false);
  session_round_trip(manager, name, 2, false);
  session_round_trip(manager, name, 3, true);
  session_round_trip(manager, name, 4, false);
  fd = ::shm_open(name.c_str(), O_RDWR, 0600);
  assert(fd >= 0);
  struct stat after{};
  assert(::fstat(fd, &after) == 0);
  ::close(fd);
  assert(before.st_dev == after.st_dev && before.st_ino == after.st_ino);
  // An endpoint role may be used only once per epoch.
  PersistentSharedMemoryTransport server(name, true, settings());
  assert(server.start([](Message){}));
  assert(server.stop());
  assert(server.start([](Message){}).error == Error::already_started);
  assert(manager.begin_session());
  Configuration incorrect=settings();
  incorrect.queue_depth=2;
  PersistentSharedMemoryTransport invalid(name, false, incorrect);
  assert(invalid.start([](Message){}).error == Error::invalid_argument);
  incorrect=settings();
  incorrect.max_payload_bytes=64;
  PersistentSharedMemoryTransport mismatch(name,false,incorrect);
  assert(mismatch.start([](Message){}).error == Error::io_error);
  assert(manager.stop());
  assert(::shm_open(name.c_str(), O_RDWR, 0600) == -1);
}
