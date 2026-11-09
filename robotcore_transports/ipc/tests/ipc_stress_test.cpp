#include "robotcore/ipc.hpp"
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
using namespace robotcore;
using namespace std::chrono_literals;
int main() {
  constexpr int producers = 4, per_producer = 150;
  const std::string path = "/tmp/robotcore-ipc-stress-" + std::to_string(::getpid()) + ".sock";
  Configuration c; c.transport = TransportKind::ipc; c.max_payload_bytes = 1024;
  UnixSocketTransport server(path, true, c);
  std::array<std::atomic<int>, producers * per_producer> seen{};
  std::atomic<int> count{0};
  std::atomic<int> invalid{0};
  assert(server.start([&](Message message) {
    if (message.sequence >= seen.size() || message.payload.size() != 128) {
      ++invalid; return;
    }
    const unsigned producer = static_cast<unsigned>(message.sequence / per_producer);
    for (auto byte : message.payload) if (byte != static_cast<std::uint8_t>(producer)) {
      ++invalid; return;
    }
    if (++seen[message.sequence] != 1) ++invalid;
    ++count;
  }));
  UnixSocketTransport client(path, false, c);
  assert(client.start([](Message) {}));
  std::atomic<int> publish_errors{0};
  std::vector<std::thread> workers;
  for (int id = 0; id < producers; ++id) {
    workers.emplace_back([&, id] {
      for (int n = 0; n < per_producer; ++n) {
        const std::uint64_t seq = static_cast<std::uint64_t>(id * per_producer + n);
        const auto outcome = client.publish({seq, std::vector<std::uint8_t>(128, static_cast<std::uint8_t>(id))});
        if (!outcome) ++publish_errors;
      }
    });
  }
  for (auto& worker : workers) worker.join();
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (count.load() < producers * per_producer &&
         std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(5ms);
  assert(publish_errors == 0);
  assert(invalid == 0);
  assert(count == producers * per_producer);
  assert(client.stop());
  // A clean client close must cause the receive worker to terminate.
  const auto disconnect_deadline = std::chrono::steady_clock::now() + 2s;
  while (server.publish({999, {}}).error != Error::not_started &&
         std::chrono::steady_clock::now() < disconnect_deadline)
    std::this_thread::sleep_for(10ms);
  assert(server.publish({999, {}}).error == Error::not_started);
  assert(server.stop());
  assert(::access(path.c_str(), F_OK) == -1);
  std::cout << "600 concurrent-producer messages delivered once each; disconnect handled\n";
}
