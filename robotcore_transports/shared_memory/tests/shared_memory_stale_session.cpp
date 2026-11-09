#include "robotcore/shared_memory.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
using namespace robotcore;
using namespace std::chrono_literals;

static Configuration config() {
  Configuration c;
  c.transport = TransportKind::shared_memory;
  c.queue_depth = 1;
  c.max_payload_bytes = 128;
  return c;
}
static bool until(const std::function<bool()>& test) {
  for (int i = 0; i < 200; ++i) {
    if (test()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return test();
}

static void stale_reply_is_not_inherited() {
  const std::string name = "/robotcore-shm-stale-reply-" + std::to_string(::getpid());
  auto c = config();
  SharedMemoryTransport server(name, true, c);
  SharedMemoryTransport previous(name, false, c);
  SharedMemoryTransport next(name, false, c);
  assert(server.start([](Message) {}));
  assert(previous.start([](Message) {}));
  assert(previous.stop());

  // A response queued after client departure belongs to the old session.
  // A fresh client must never read it.
  assert(server.publish({123, {1, 2, 3}}));
  std::atomic<int> unexpected{0};
  const auto error = next.start([&](Message) { ++unexpected; });
  assert(error.error == Error::io_error);
  assert(unexpected.load() == 0);
  assert(server.stop());

  // A new mapping/session is a valid explicit recovery operation.
  assert(server.start([](Message) {}));
  assert(next.start([&](Message) { ++unexpected; }));
  assert(next.stop());
  assert(server.stop());
  assert(unexpected.load() == 0);
}

static void unfinished_callback_prevents_reattach() {
  const std::string name = "/robotcore-shm-active-cb-" + std::to_string(::getpid());
  auto c = config();
  std::atomic<bool> callback_active{false};
  std::atomic<bool> release_callback{false};
  SharedMemoryTransport server(name, true, c);
  assert(server.start([&](Message) {
    callback_active = true;
    while (!release_callback) std::this_thread::sleep_for(1ms);
    callback_active = false;
  }));
  SharedMemoryTransport previous(name, false, c);
  assert(previous.start([](Message) {}));
  assert(previous.publish({42, {1}}));
  assert(until([&] { return callback_active.load(); }));
  assert(previous.stop());

  SharedMemoryTransport next(name, false, c);
  assert(next.start([](Message) {}).error == Error::io_error);
  release_callback = true;
  assert(until([&] { return !callback_active.load(); }));
  // The old receive callback must finish before a fresh client attaches.
  assert(until([&] {
    const auto status = next.start([](Message) {});
    return static_cast<bool>(status);
  }));
  assert(next.stop());
  assert(server.stop());
}

int main() {
  stale_reply_is_not_inherited();
  unfinished_callback_prevents_reattach();
}
