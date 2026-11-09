#include "robotcore/ipc.hpp"
#include <atomic>
#include <cassert>
#include <chrono>
#include <csignal>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
using namespace robotcore;
using namespace std::chrono_literals;
static bool wait_for(const std::function<bool()>& fn, int timeout_ms=2500) {
  for(int elapsed=0;elapsed<timeout_ms;elapsed+=10) {
    if(fn()) return true;
    std::this_thread::sleep_for(10ms);
  }
  return fn();
}
int main() {
  Configuration c; c.transport=TransportKind::ipc; c.max_payload_bytes=1024;
  const std::string path="/tmp/robotcore-ipc-reconnect-"+std::to_string(getpid())+".sock";
  std::atomic<int> received{0};
  UnixSocketTransport server(path,true,c);
  assert(server.start([&](Message m) {
    if(m.payload==std::vector<std::uint8_t>{7,8}) ++received;
  }));
  // The same listening endpoint should survive two sequential client sessions.
  for(int session=0;session<2;++session) {
    UnixSocketTransport client(path,false,c);
    assert(client.start([](Message){}));
    assert(client.publish({static_cast<std::uint64_t>(session+1),{7,8}}));
    assert(wait_for([&]{return received.load()==session+1;}));
    assert(client.stop());
  }
  assert(server.stop());
  assert(::access(path.c_str(),F_OK)!=0);

  // A child process that is killed cannot unlink its bound filesystem socket.
  // The library intentionally refuses to silently overwrite that endpoint.
  int ready[2]; assert(::pipe(ready)==0);
  const pid_t pid=::fork(); assert(pid>=0);
  if(pid==0) {
    ::close(ready[0]);
    UnixSocketTransport dead(path,true,c);
    if(!dead.start([](Message){})) _exit(3);
    char mark='y';
    if(::write(ready[1], &mark,1)!=1) _exit(4);
    ::close(ready[1]);
    for(;;) ::pause();
  }
  ::close(ready[1]);
  char mark=0; assert(::read(ready[0],&mark,1)==1 && mark=='y');
  ::close(ready[0]);
  assert(::kill(pid,SIGKILL)==0);
  int status=0; assert(::waitpid(pid,&status,0)==pid);
  assert(WIFSIGNALED(status) && WTERMSIG(status)==SIGKILL);
  struct stat st{};
  assert(::lstat(path.c_str(),&st)==0 && S_ISSOCK(st.st_mode));
  UnixSocketTransport blocked(path,true,c);
  assert(blocked.start([](Message){}).error==Error::io_error);
  // This explicit unlink is exclusively test-fixture cleanup. A production
  // application needs ownership checks and an operator-controlled recovery policy.
  assert(::unlink(path.c_str())==0);
  UnixSocketTransport restarted(path,true,c);
  std::atomic<bool> got{false};
  assert(restarted.start([&](Message m){if(m.sequence==99 && m.payload==std::vector<std::uint8_t>{1})got=true;}));
  UnixSocketTransport reconnect(path,false,c);
  assert(reconnect.start([](Message){}));
  assert(reconnect.publish({99,{1}}));
  assert(wait_for([&]{return got.load();}));
  assert(reconnect.stop());
  assert(restarted.stop());
  assert(::access(path.c_str(),F_OK)!=0);
}
