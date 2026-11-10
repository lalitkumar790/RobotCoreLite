#include "robotcore/shared_memory.hpp"
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
using namespace robotcore;
using namespace std::chrono_literals;

static void must(bool yes, const char* why) {
  if (!yes) throw std::runtime_error(why);
}
static Configuration config() {
  Configuration c;
  c.transport = TransportKind::shared_memory;
  c.queue_depth = 1;
  c.max_payload_bytes = 4096;
  c.reliable = true;
  return c;
}
static std::size_t payload_size(std::uint64_t seq) {
  return seq % 9 == 0 ? 0u : (seq % 3 == 0 ? 4096u : 1047u);
}
static std::uint8_t pattern(std::uint64_t seq, std::size_t i) {
  return static_cast<std::uint8_t>((seq * 13u + i * 31u) & 0xffu);
}

static void client_publish(const std::string& name) {
  SharedMemoryTransport client(name, false, config());
  must(bool(client.start([](Message) {})), "client failed to attach");
  SharedMemoryTransport::WriteLoan empty;
  must(bool(client.acquire_write_loan(empty)), "could not reserve disposable slot");
  must(bool(empty), "write loan missing");
  must(empty.capacity() == config().max_payload_bytes, "incorrect capacity");
  empty.data()[0] = 0xff;
  empty.discard();
  empty.discard();  // idempotent
  must(!empty, "discard did not clear loan");

  for (std::uint64_t seq = 1; seq <= 40; ++seq) {
    SharedMemoryTransport::WriteLoan loan;
    must(bool(client.acquire_write_loan(loan)), "could not acquire direct SHM buffer");
    must(bool(loan), "write loan invalid");
    const auto size = payload_size(seq);
    for (std::size_t i = 0; i < size; ++i) loan.data()[i] = pattern(seq, i);
    if (seq == 5) {
      SharedMemoryTransport::WriteLoan moved(std::move(loan));
      must(!loan && bool(moved), "moving a loan lost ownership");
      must(bool(moved.commit(seq, size)), "moved loan failed commit");
      must(!moved, "successful commit did not consume loan");
    } else if (seq == 8) {
      auto failure = loan.commit(seq, loan.capacity() + 1);
      must(failure.error == Error::too_large, "oversize commit accepted");
      must(bool(loan), "failed commit should retain loan for correction");
      must(bool(loan.commit(seq, size)), "failed to recover oversized loan");
    } else {
      must(bool(loan.commit(seq, size)), "loaned commit failed");
    }
    must(!loan, "committed loan still owned slot");
    if (seq == 9) {
      auto second = loan.commit(seq, size);
      must(second.error == Error::not_started, "double commit succeeded");
    }
  }
  must(bool(client.stop()), "client stop failed");
  SharedMemoryTransport::WriteLoan late;
  must(client.acquire_write_loan(late).error == Error::not_started,
       "loan after stop should fail");
}

static void cross_process() {
  const std::string name = "/robotcore-loaned-" + std::to_string(::getpid());
  int ready[2];
  must(::pipe(ready) == 0, "pipe failed");
  pid_t pid = ::fork();
  must(pid >= 0, "fork failed");
  if (pid == 0) {
    ::close(ready[0]);
    SharedMemoryTransport server(name, true, config());
    std::atomic<int> received{0};
    std::atomic<bool> bad{false};
    auto st = server.start_borrowed([&](BorrowedMessage message) {
      const auto expected = static_cast<std::uint64_t>(received.load() + 1);
      if (message.sequence != expected || message.size != payload_size(expected) ||
          !message.data) { bad = true; return; }
      for (std::size_t i = 0; i < message.size; ++i) {
        if (message.data[i] != pattern(expected, i)) { bad = true; return; }
      }
      // Holding a read loan keeps the slot unavailable to the producer.
      if (expected == 1) std::this_thread::sleep_for(35ms);
      ++received;
    });
    if (!st) _exit(10);
    if (::write(ready[1], "r", 1) != 1) _exit(11);
    ::close(ready[1]);
    const auto until = std::chrono::steady_clock::now() + 6s;
    while (!bad && received < 40 && std::chrono::steady_clock::now() < until)
      std::this_thread::sleep_for(2ms);
    const bool passed = !bad && received == 40;
    if (!server.stop()) _exit(12);
    _exit(passed ? 0 : 13);
  }

  ::close(ready[1]);
  char one{};
  must(::read(ready[0], &one, 1) == 1 && one == 'r', "server not ready");
  ::close(ready[0]);
  client_publish(name);
  int status = 0;
  must(::waitpid(pid, &status, 0) == pid, "server did not exit");
  must(WIFEXITED(status) && WEXITSTATUS(status) == 0,
       "server rejected loaned payload or sequence");
}

static void lifetime_guard() {
  const std::string name = "/robotcore-loan-lifetime-" + std::to_string(::getpid());
  SharedMemoryTransport server(name, true, config());
  must(bool(server.start_borrowed([](BorrowedMessage) {})), "server start failed");
  SharedMemoryTransport::WriteLoan held;
  must(bool(server.acquire_write_loan(held)), "could not acquire loan");
  must(bool(held), "missing loan");
  std::atomic<bool> stopped{false};
  std::thread stopper([&] {
    const Status result = server.stop();
    stopped = bool(result);
  });
  std::this_thread::sleep_for(30ms);
  must(!stopped, "stop unmapped a live loan");
  held.discard();
  stopper.join();
  must(stopped, "stop could not drain loaned publisher");
}

int main() {
  try {
    cross_process();
    lifetime_guard();
    std::cout << "borrowed_shm_cross_process=ok\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "borrowed SHM regression: " << e.what() << '\n';
    return 1;
  }
}
