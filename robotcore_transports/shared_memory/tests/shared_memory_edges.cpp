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
static bool until(const std::function<bool()>& f) {
  for (int i=0; i<400; ++i) {
    if (f()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return f();
}
int main() {
  Configuration c; c.transport = TransportKind::shared_memory;
  c.max_payload_bytes = 128; c.queue_depth = 1;
  const std::string name = "/robotcore-shm-edge-" + std::to_string(getpid());
  SharedMemoryTransport server(name,true,c);
  std::atomic<int> delivered{0};
  assert(server.start([&](Message m) { assert(server.publish(m)); }));
  SharedMemoryTransport client(name,false,c);
  assert(client.start([&](Message m) {
    if(m.sequence==1 && m.payload.empty()) ++delivered;
    if(m.sequence==2 && m.payload==std::vector<std::uint8_t>(128,42)) ++delivered;
  }));
  SharedMemoryTransport duplicate(name,false,c);
  assert(duplicate.start([](Message){}).error==Error::already_started);
  Configuration different=c; different.max_payload_bytes=127;
  SharedMemoryTransport mismatch(name,false,different);
  assert(mismatch.start([](Message){}).error==Error::io_error);
  assert(client.publish({1,{}}));
  assert(client.publish({2,std::vector<std::uint8_t>(128,42)}));
  assert(until([&]{return delivered.load()==2;}));
  assert(client.stop());
  // Orderly detach releases ownership, allowing a subsequent attach.
  assert(duplicate.start([](Message){}));
  assert(duplicate.stop());
  assert(server.stop());
  assert(::shm_open(name.c_str(),O_RDWR,0600)==-1);

  // Abrupt process exit preserves the shm name; never unlink automatically.
  int pipefd[2]; assert(::pipe(pipefd)==0);
  const pid_t pid=::fork(); assert(pid>=0);
  if (pid==0) {
    ::close(pipefd[0]);
    SharedMemoryTransport dead(name,true,c);
    if (!dead.start([](Message){})) _exit(3);
    char ready='x';
    if (::write(pipefd[1],&ready,1)!=1) _exit(4);
    _exit(0); // Simulates crash without running destructors.
  }
  ::close(pipefd[1]);
  char ready{};
  assert(::read(pipefd[0],&ready,1)==1 && ready=='x');
  ::close(pipefd[0]);
  int status=0; assert(::waitpid(pid,&status,0)==pid);
  assert(WIFEXITED(status) && WEXITSTATUS(status)==0);
  SharedMemoryTransport refused(name,true,c);
  assert(refused.start([](Message){}).error==Error::io_error);
  // Fixture-only cleanup after waiting for process exit.
  assert(::shm_unlink(name.c_str())==0);
  assert(refused.start([](Message){}));
  assert(refused.stop());
}
