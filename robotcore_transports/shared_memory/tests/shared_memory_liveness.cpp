#include "robotcore/shared_memory.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <csignal>
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

static bool eventually(const std::function<bool()>& f) {
  for (int i = 0; i < 200; ++i) {
    if (f()) return true;
    std::this_thread::sleep_for(10ms);
  }
  return f();
}
static Configuration configuration() {
  Configuration c;
  c.transport = TransportKind::shared_memory;
  c.queue_depth = 1;
  c.max_payload_bytes = 128;
  return c;
}
static void server_loss(bool kill_server) {
  const std::string name = "/robotcore-shm-owner-" + std::to_string(::getpid()) +
                           (kill_server ? "-crash" : "-stop");
  const auto c = configuration();
  int ready_pipe[2], stop_pipe[2];
  assert(::pipe(ready_pipe) == 0);
  assert(::pipe(stop_pipe) == 0);
  const pid_t child = ::fork();
  assert(child >= 0);
  if (child == 0) {
    ::close(ready_pipe[0]);
    ::close(stop_pipe[1]);
    SharedMemoryTransport server(name, true, c);
    if (!server.start([&](Message msg) { (void)server.publish(msg); })) _exit(3);
    const char ready = 'r';
    if (::write(ready_pipe[1], &ready, 1) != 1) _exit(4);
    ::close(ready_pipe[1]);
    char stop = 0;
    if (::read(stop_pipe[0], &stop, 1) != 1) _exit(5);
    ::close(stop_pipe[0]);
    if (!server.stop()) _exit(6);
    _exit(0);
  }
  ::close(ready_pipe[1]);
  ::close(stop_pipe[0]);
  char ready = 0;
  assert(::read(ready_pipe[0], &ready, 1) == 1 && ready == 'r');
  ::close(ready_pipe[0]);
  SharedMemoryTransport client(name, false, c);
  std::atomic<bool> replied{false};
  assert(client.start([&](Message msg) {
    if (msg.sequence == 21 && msg.payload == std::vector<std::uint8_t>{1, 2, 3})
      replied = true;
  }));
  assert(client.publish({21, {1, 2, 3}}));
  assert(eventually([&] { return replied.load(); }));
  if (kill_server) {
    ::close(stop_pipe[1]);
    assert(::kill(child, SIGKILL) == 0);
  } else {
    assert(::write(stop_pipe[1], "s", 1) == 1);
    ::close(stop_pipe[1]);
  }
  int status = 0;
  assert(::waitpid(child, &status, 0) == child);
  if (kill_server) assert(WIFSIGNALED(status) && WTERMSIG(status) == SIGKILL);
  else assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  assert(eventually([&] {
    return client.publish({23, {4}}).error == Error::closed;
  }));
  assert(client.stop());

  if (kill_server) {
    // A crash leaves the shm name. A new client must reject it, and a new
    // server must not overwrite another session's stale synchronization.
    SharedMemoryTransport stale(name, false, c);
    assert(stale.start([](Message) {}).error == Error::closed);
    SharedMemoryTransport refused(name, true, c);
    assert(refused.start([](Message) {}).error == Error::io_error);
    // Test-only cleanup: child was reaped and the owner is known dead.
    assert(::shm_unlink(name.c_str()) == 0);
    assert(refused.start([](Message) {}));
    assert(refused.stop());
  } else {
    assert(::shm_open(name.c_str(), O_RDWR, 0600) < 0);
  }
}

static void abandoned_client_claim() {
  const std::string name = "/robotcore-shm-client-death-" + std::to_string(::getpid());
  const auto c = configuration();
  int pipefd[2]; assert(::pipe(pipefd) == 0);
  const pid_t child = ::fork(); assert(child >= 0);
  if (child == 0) {
    ::close(pipefd[1]);
    char ready;
    if (::read(pipefd[0], &ready, 1) != 1) _exit(4);
    ::close(pipefd[0]);
    SharedMemoryTransport client(name, false, c);
    if (!client.start([](Message) {})) _exit(5);
    _exit(0); // process exit deliberately skips stop() and claim release
  }
  ::close(pipefd[0]);
  SharedMemoryTransport server(name, true, c);
  assert(server.start([](Message) {}));
  assert(::write(pipefd[1], "c", 1) == 1);
  ::close(pipefd[1]);
  int status = 0;
  assert(::waitpid(child, &status, 0) == child);
  assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  SharedMemoryTransport blocked(name, false, c);
  assert(blocked.start([](Message) {}).error == Error::already_started);
  // Fail-closed. Do NOT reset semaphores or reclaim the dead client's claim.
  assert(server.stop());
  assert(server.start([](Message) {})); // fresh, distinct region
  assert(blocked.start([](Message) {}));
  assert(blocked.stop());
  assert(server.stop());
}
int main() {
  server_loss(false);
  server_loss(true);
  abandoned_client_claim();
}
