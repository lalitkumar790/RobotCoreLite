#include "robotcore/ipc.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace robotcore;
using namespace std::chrono_literals;
static bool until(const std::function<bool()>& predicate) {
  for (int n=0; n < 200; ++n) {
    if (predicate()) return true;
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}
int main() {
  const std::string path = "/tmp/robotcore-ipc-test-" + std::to_string(getpid()) + ".sock";
  Configuration config; config.transport = TransportKind::ipc; config.max_payload_bytes = 1024;
  UnixSocketTransport missing(path, false, config);
  assert(missing.start([](Message){}).error == Error::io_error);
  int ready_pipe[2];
  assert(pipe(ready_pipe) == 0);
  const pid_t child = fork();
  assert(child >= 0);
  if (child == 0) {
    close(ready_pipe[1]);
    char ready;
    if (read(ready_pipe[0], &ready, 1) != 1) _exit(5);
    close(ready_pipe[0]);
    UnixSocketTransport client(path, false, config);
    std::atomic<bool> child_got{false};
    if (!client.start([&](Message msg){
      if (msg.sequence == 43 && msg.payload == std::vector<std::uint8_t>{8,9,10}) child_got=true;
    })) _exit(1);
    if (client.publish({42, {0, 1, 255}}).error != Error::ok) _exit(2);
    if (!until([&] {return child_got.load();})) _exit(3);
    if (client.publish({44, std::vector<std::uint8_t>(1025)}).error != Error::too_large) _exit(4);
    client.stop();
    _exit(0);
  }
  close(ready_pipe[0]);
  UnixSocketTransport server(path, true, config);
  std::atomic<bool> parent_got{false};
  assert(server.start([&](Message msg) {
    if (msg.sequence == 42 && msg.payload == std::vector<std::uint8_t>{0, 1, 255}) {
      parent_got = true;
      server.publish({43, {8, 9, 10}});
    }
  }));
  UnixSocketTransport duplicate(path, true, config);
  assert(duplicate.start([](Message){}).error == Error::io_error);
  assert(write(ready_pipe[1], "x", 1) == 1);
  close(ready_pipe[1]);
  assert(until([&]{return parent_got.load();}));
  int status = 0;
  assert(waitpid(child, &status, 0) == child);
  assert(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  assert(server.stop());
  assert(access(path.c_str(), F_OK) == -1);
  assert(server.publish({1, {}}).error == Error::not_started);
  assert(server.start([](Message){}));
  assert(server.stop());
  std::cout << "cross-process IPC delivery, framing, bounds, endpoint ownership, lifecycle PASS\n";
}
