#include "robotcore/ipc.hpp"
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace robotcore {
namespace {
constexpr std::uint32_t kMagic = 0x52434c31; // RCL1
constexpr std::size_t kHeaderBytes = 20;
void put32(std::uint8_t* b, std::uint32_t v) {
  for (int i=0;i<4;++i) b[i] = static_cast<std::uint8_t>(v >> (8*i));
}
void put64(std::uint8_t* b, std::uint64_t v) {
  for (int i=0;i<8;++i) b[i] = static_cast<std::uint8_t>(v >> (8*i));
}
std::uint32_t get32(const std::uint8_t* b) {
  std::uint32_t v=0; for(int i=0;i<4;++i) v |= std::uint32_t(b[i]) << (8*i); return v;
}
std::uint64_t get64(const std::uint8_t* b) {
  std::uint64_t v=0; for(int i=0;i<8;++i) v |= std::uint64_t(b[i]) << (8*i); return v;
}
std::uint32_t checksum(const std::vector<std::uint8_t>& payload) {
  std::uint32_t hash = 2166136261u;
  for (auto byte : payload) { hash ^= byte; hash *= 16777619u; }
  return hash;
}
void set_timeout(int fd) {
  timeval tv{}; tv.tv_sec = 2;
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
}
bool write_all(int fd, const std::uint8_t* p, std::size_t n) {
  while (n) {
    const auto written = ::send(fd, p, n, MSG_NOSIGNAL);
    if (written < 0 && errno == EINTR) continue;
    if (written <= 0) return false;
    p += written; n -= static_cast<std::size_t>(written);
  }
  return true;
}
bool read_all(int fd, std::uint8_t* p, std::size_t n) {
  while (n) {
    auto got = ::recv(fd, p, n, 0);
    if (got < 0 && errno == EINTR) continue;
    if (got <= 0) return false;
    p += got; n -= static_cast<std::size_t>(got);
  }
  return true;
}
} // namespace

UnixSocketTransport::UnixSocketTransport(std::string path, bool server, Configuration config)
  : path_(std::move(path)), server_(server), config_(config) {}
UnixSocketTransport::~UnixSocketTransport() { stop(); }

Status UnixSocketTransport::start(ReceiveCallback callback) {
  if (running_ || worker_.joinable())
    return Status::failure(Error::already_started, "call stop before restarting");
  auto valid = validate(config_);
  if (!valid) return valid;
  if (config_.transport != TransportKind::ipc)
    return Status::failure(Error::invalid_argument, "config transport must be ipc");
  if (!callback || path_.empty() || path_.size() >= sizeof(sockaddr_un::sun_path))
    return Status::failure(Error::invalid_argument, "callback/path required and path must fit sockaddr_un");
  const int sock = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (sock < 0) return Status::failure(Error::io_error, std::strerror(errno));
  sockaddr_un addr{}; addr.sun_family = AF_UNIX;
  std::memcpy(addr.sun_path, path_.c_str(), path_.size() + 1);
  if (server_) {
    // Never unlink somebody else's existing endpoint. A stale socket must be resolved by caller.
    if (::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      const std::string error = std::strerror(errno); ::close(sock);
      return Status::failure(Error::io_error, error);
    }
    owns_path_ = true;
    ::chmod(path_.c_str(), S_IRUSR | S_IWUSR);
    if (::listen(sock, 1) < 0) {
      const std::string error = std::strerror(errno);
      ::close(sock); ::unlink(path_.c_str()); owns_path_ = false;
      return Status::failure(Error::io_error, error);
    }
    listener_ = sock;
  } else {
    if (::connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      const std::string error = std::strerror(errno); ::close(sock);
      return Status::failure(Error::io_error, error);
    }
    set_timeout(sock);
    peer_ = sock;
  }
  callback_ = std::move(callback);
  running_ = true;
  try {
    worker_ = std::thread([this] { run(); });
  } catch (...) {
    running_ = false;
    const auto l = listener_.exchange(-1), p = peer_.exchange(-1);
    if (l >= 0) ::close(l);
    if (p >= 0) ::close(p);
    if (owns_path_) { ::unlink(path_.c_str()); owns_path_ = false; }
    callback_ = {};
    return Status::failure(Error::io_error, "cannot create receive thread");
  }
  return Status::success();
}

void UnixSocketTransport::run() {
  if (!server_) {
    const int fd = peer_;
    if (fd >= 0) receive_loop(fd);
    running_ = false; // Client explicitly stop()s before attempting reconnect.
    return;
  }
  // One peer at a time. The listening socket persists across disconnects.
  while (running_) {
    pollfd wait_for_client{};
    wait_for_client.fd = listener_;
    wait_for_client.events = POLLIN;
    const int ready = ::poll(&wait_for_client, 1, 100);
    if (ready < 0 && errno == EINTR) continue;
    if (ready < 0) break;
    if (ready == 0) continue;
    if (!running_ || (wait_for_client.revents & (POLLERR | POLLHUP | POLLNVAL))) break;
    if (!(wait_for_client.revents & POLLIN)) continue;
    const int fd = ::accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
    if (fd < 0 && (errno == EAGAIN || errno == EINTR)) continue;
    if (fd < 0) break;
    const int flags = ::fcntl(fd, F_GETFL);
    if (flags >= 0) ::fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    set_timeout(fd);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) { ::close(fd); break; }
      peer_ = fd;
    }
    receive_loop(fd);
    // Coordinate descriptor lifetime with publishers and stop(). A disconnected
    // peer must never leave a stale descriptor pointing to a later client.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (peer_ == fd) {
        peer_ = -1;
        ::close(fd);
      }
    }
  }
  running_ = false;
}
void UnixSocketTransport::receive_loop(int fd) {
  while (running_) {
    std::array<std::uint8_t, kHeaderBytes> header{};
    if (!read_all(fd, header.data(), header.size())) break;
    if (get32(header.data()) != kMagic) break;
    const auto length = get32(header.data() + 4);
    if (length > config_.max_payload_bytes) break;
    Message msg; msg.sequence = get64(header.data() + 8);
    msg.payload.resize(length);
    if (!read_all(fd, msg.payload.data(), msg.payload.size())) break;
    if (get32(header.data() + 16) != checksum(msg.payload)) break;
    try { callback_(std::move(msg)); } catch (...) { break; }
  }
}
Status UnixSocketTransport::publish(const Message& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || peer_ < 0)
    return Status::failure(Error::not_started, "transport not connected");
  auto valid = validate_message(message, config_);
  if (!valid) return valid;
  std::array<std::uint8_t, kHeaderBytes> header{};
  put32(header.data(), kMagic);
  put32(header.data() + 4, static_cast<std::uint32_t>(message.payload.size()));
  put64(header.data() + 8, message.sequence);
  put32(header.data() + 16, checksum(message.payload));
  if (!write_all(peer_, header.data(), header.size()) ||
      !write_all(peer_, message.payload.data(), message.payload.size())) {
    running_ = false;
    ::shutdown(peer_, SHUT_RDWR);
    return Status::failure(Error::io_error, "socket write failed; call stop before restarting");
  }
  return Status::success();
}
Status UnixSocketTransport::stop() {
  if (worker_.joinable() && worker_.get_id() == std::this_thread::get_id())
    return Status::failure(Error::invalid_argument, "stop must not be called from callback");
  running_ = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int l = listener_;
    const int p = peer_;
    if (l >= 0) ::shutdown(l, SHUT_RDWR);
    if (p >= 0) ::shutdown(p, SHUT_RDWR);
  }
  if (worker_.joinable()) worker_.join();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int l = listener_.exchange(-1), p = peer_.exchange(-1);
    if (l >= 0) ::close(l);
    if (p >= 0) ::close(p);
  }
  if (owns_path_) { ::unlink(path_.c_str()); owns_path_ = false; }
  callback_ = {};
  return Status::success();
}
} // namespace robotcore
