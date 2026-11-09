#include "robotcore/ipc.hpp"
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
using namespace robotcore;
using namespace std::chrono_literals;

static void put32(std::uint8_t* p, std::uint32_t value) {
  for (int i=0; i<4; ++i) p[i] = static_cast<std::uint8_t>(value >> (8*i));
}
static int raw_connect(const std::string& path) {
  int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  assert(fd >= 0);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  assert(path.size() < sizeof(addr.sun_path));
  path.copy(addr.sun_path, path.size());
  assert(::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
  return fd;
}
static std::string unique_path(const char* suffix) {
  return "/tmp/robotcore-ipc-fault-" + std::to_string(::getpid()) + "-" +
         std::string(suffix) + ".sock";
}
static void inject(const char* scenario, const std::function<void(int)>& attack) {
  const auto path = unique_path(scenario);
  Configuration c; c.transport = TransportKind::ipc; c.max_payload_bytes = 64;
  std::atomic<int> delivered{0};
  UnixSocketTransport server(path, true, c);
  assert(server.start([&](Message) { ++delivered; }));
  int fd = raw_connect(path);
  attack(fd);
  ::shutdown(fd, SHUT_WR);
  ::close(fd);
  // The worker must reject all malformed input without invoking callbacks.
  std::this_thread::sleep_for(100ms);
  assert(server.stop());
  assert(delivered == 0);
  assert(::access(path.c_str(), F_OK) == -1);
}
int main() {
  Configuration c; c.transport = TransportKind::ipc;
  const auto idle_path = unique_path("idle");
  UnixSocketTransport idle(idle_path, true, c);
  assert(idle.start([](Message) {}));
  const auto t0 = std::chrono::steady_clock::now();
  assert(idle.stop()); // Must not hang inside a blocking accept().
  assert(std::chrono::steady_clock::now() - t0 < 2s);
  assert(::access(idle_path.c_str(), F_OK) == -1);

  inject("truncated", [](int fd) {
    std::array<std::uint8_t, 5> bytes{{0x31, 0x4c, 0x43, 0x52, 0}};
    assert(::send(fd, bytes.data(), bytes.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(bytes.size()));
  });
  inject("badmagic", [](int fd) {
    std::array<std::uint8_t, 20> header{};
    put32(header.data(), 0x12345678);
    assert(::send(fd, header.data(), header.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(header.size()));
  });
  inject("oversized", [](int fd) {
    std::array<std::uint8_t, 20> header{};
    put32(header.data(), 0x52434c31);
    put32(header.data() + 4, 65); // Above configured 64-byte limit.
    assert(::send(fd, header.data(), header.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(header.size()));
  });
  inject("checksum", [](int fd) {
    std::array<std::uint8_t, 21> bytes{};
    put32(bytes.data(), 0x52434c31);
    put32(bytes.data() + 4, 1);
    // Incorrect checksum for byte 0x7f.
    bytes[20] = 0x7f;
    assert(::send(fd, bytes.data(), bytes.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(bytes.size()));
  });
  inject("shortpayload", [](int fd) {
    std::array<std::uint8_t, 21> bytes{};
    put32(bytes.data(), 0x52434c31);
    put32(bytes.data() + 4, 5);
    bytes[20] = 5; // Declared five bytes, sent only one.
    assert(::send(fd, bytes.data(), bytes.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(bytes.size()));
  });
  std::cout << "idle stop and malformed framing scenarios passed\n";
}
