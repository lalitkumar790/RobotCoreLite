#include "robotcore/shared_memory.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>
using namespace robotcore;
using namespace std::chrono_literals;
static bool eventually(const std::function<bool()>& condition) {
  for (int i = 0; i < 400; ++i) {
    if (condition()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return condition();
}
int main() {
  const std::string name = "/robotcore-shm-test-" + std::to_string(::getpid());
  Configuration config;
  config.transport = TransportKind::shared_memory;
  config.queue_depth = 1;
  config.max_payload_bytes = 128;
  assert(resolve_transport(config).first);
  assert(resolve_transport(config).second == TransportKind::shared_memory);
  SharedMemoryTransport missing(name, false, config);
  assert(missing.start([](Message) {}).error == Error::io_error);
  int ready[2]; assert(::pipe(ready) == 0);
  const pid_t child = ::fork(); assert(child >= 0);
  if (child == 0) {
    ::close(ready[1]);
    char signal{};
    if (::read(ready[0], &signal, 1) != 1) _exit(1);
    ::close(ready[0]);
    SharedMemoryTransport client(name, false, config);
    std::atomic<int> replies{0};
    if (!client.start([&](Message m) {
      if (m.sequence > 0 && m.sequence <= 60 && m.payload == std::vector<std::uint8_t>(64, 0x5a))
        ++replies;
    })) _exit(2);
    if (client.publish({0, std::vector<std::uint8_t>(129)}).error != Error::too_large) _exit(3);
    for (std::uint64_t seq = 1; seq <= 60; ++seq) {
      if (!client.publish({seq, std::vector<std::uint8_t>(64, 0x5a)})) _exit(4);
    }
    if (!eventually([&] { return replies.load() == 60; })) _exit(5);
    if (!client.stop()) _exit(6);
    _exit(0);
  }
  ::close(ready[0]);
  SharedMemoryTransport server(name, true, config);
  std::atomic<int> requests{0};
  assert(server.start([&](Message m) {
    assert(m.payload == std::vector<std::uint8_t>(64, 0x5a));
    ++requests;
    assert(server.publish(m));
  }));
  SharedMemoryTransport duplicate(name, true, config);
  assert(duplicate.start([](Message){}).error == Error::io_error);
  assert(::write(ready[1], "r", 1) == 1);
  ::close(ready[1]);
  int status = 0;
  assert(::waitpid(child, &status, 0) == child);
  assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  assert(requests.load() == 60);
  assert(server.stop());
  assert(::shm_open(name.c_str(), O_RDWR, 0600) == -1);
  assert(server.publish({1,{1}}).error == Error::not_started);
  assert(server.start([](Message){}));
  assert(server.stop());
  config.queue_depth = 2;
  SharedMemoryTransport invalid(name, true, config);
  assert(invalid.start([](Message){}).error == Error::invalid_argument);
}
