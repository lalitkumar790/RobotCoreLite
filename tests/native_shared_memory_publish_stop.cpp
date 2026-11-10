// Reproduces stop() racing a publisher blocked on a depth-one SHM slot.
// The server deliberately holds its receiver callback while the client
// fills the one available slot and starts a third publish on another thread.
//
// Tests exercise *published memory lifetime*, not full thread-safety of
// concurrent start()/stop() or of a destroyed transport object.
#if defined(ROBOTCORE_STOP_RACE_PERSISTENT)
#include "robotcore/persistent_shm.hpp"
#else
#include "robotcore/shared_memory.hpp"
#endif
#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

using namespace robotcore;
using namespace std::chrono_literals;

static void require(bool passed, const char* message) {
  if (!passed) throw std::runtime_error(message);
}

static Configuration configuration() {
  Configuration c;
#if defined(ROBOTCORE_STOP_RACE_PERSISTENT)
  c.transport = TransportKind::persistent_shared_memory;
#else
  c.transport = TransportKind::shared_memory;
#endif
  c.max_payload_bytes = 128;
  c.queue_depth = 1;
  c.reliable = true;
  return c;
}

static bool eventually(const std::atomic<bool>& check) {
  for (int i = 0; i < 500; ++i) {
    if (check.load()) return true;
    std::this_thread::sleep_for(2ms);
  }
  return check.load();
}

static void test_one_round(int round) {
  const std::string name = "/robotcore-stop-publisher-" +
                           std::to_string(::getpid()) + "-" +
                           std::to_string(round);
#if defined(ROBOTCORE_STOP_RACE_PERSISTENT)
  PersistentRegionManager manager(name, 128);
  require(bool(manager.start()), "manager start failed");
  require(bool(manager.begin_session()), "manager session failed");
  PersistentSharedMemoryTransport server(name, true, configuration());
  PersistentSharedMemoryTransport client(name, false, configuration());
#else
  SharedMemoryTransport server(name, true, configuration());
  SharedMemoryTransport client(name, false, configuration());
#endif
  std::atomic<bool> callback_entered{false}, release_callback{false};
  require(bool(server.start([&](Message message) {
    if (message.sequence == 1) {
      callback_entered = true;
      // The test thread releases us after stop() has joined the client.
      while (!release_callback.load()) std::this_thread::sleep_for(1ms);
    }
  })), "server start failed");
  require(bool(client.start([](Message) {})), "client start failed");

  require(bool(client.publish({1, {1, 2, 3}})), "first publish failed");
  require(eventually(callback_entered), "server callback did not enter");
  require(bool(client.publish({2, {4, 5, 6}})), "second publish failed");

  std::atomic<bool> publisher_started{false};
  std::atomic<bool> publisher_done{false};
  Status third_result;
  std::thread blocked_publisher([&] {
    publisher_started = true;
    third_result = client.publish({3, {7, 8, 9}});
    publisher_done = true;
  });
  require(eventually(publisher_started), "concurrent publish thread not started");
  std::this_thread::sleep_for(30ms);
  require(!publisher_done.load(), "third publish was not blocked by full slot");

  // Previously stop() could munmap the region while blocked_publisher was
  // still inside sem_timedwait on one of its semaphores (ASan use-after-unmap).
  const Status stop_result = client.stop();
  require(bool(stop_result), "client stop failed");
  blocked_publisher.join();
  require(publisher_done, "concurrent publisher never returned");
  require(!third_result, "in-flight blocked publish succeeded after stop");
  require(!client.publish({4, {1}}), "stopped client accepted new publish");

  release_callback = true;
  require(bool(server.stop()), "server stop failed");
#if defined(ROBOTCORE_STOP_RACE_PERSISTENT)
  require(bool(manager.stop()), "manager stop failed");
#endif
}

int main() {
  // Repeated iterations vary the stop vs semaphore timing under sanitizers.
  for (int i = 0; i < 12; ++i) test_one_round(i);
}
