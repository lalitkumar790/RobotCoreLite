// Cross-process sustained-load characterization of one-slot session SHM.
// NOT a ROS 2 / DDS benchmark. Monotonic clocks are shared only on one Linux host.
#include "robotcore/shared_memory.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <time.h>
using namespace std::chrono;
namespace {
std::atomic<bool> running{true};
void stop_signal(int) { running = false; }
std::uint64_t now_ns() {
  timespec ts{};
  if (::clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
    throw std::runtime_error("clock_gettime failed");
  return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(ts.tv_nsec);
}
std::size_t positive(const char* arg) {
  std::string s(arg);
  if (s.empty() || s[0] == '-') throw std::invalid_argument("invalid positive integer");
  std::size_t used = 0;
  auto n = std::stoull(s, &used);
  if (used != s.size() || n == 0 || n > 10000000)
    throw std::invalid_argument("invalid positive integer");
  return static_cast<std::size_t>(n);
}
struct Observation {
  std::uint64_t seq;
  std::uint64_t source_ns;
  std::uint64_t receiver_ns;
  bool valid;
};
void fill(std::uint8_t* data, std::size_t bytes, std::uint64_t seq, std::uint64_t stamp) {
  // memcpy avoids misaligned uint64_t access.
  std::memcpy(data, &seq, sizeof(seq));
  std::memcpy(data + sizeof(seq), &stamp, sizeof(stamp));
  for (std::size_t i = 16; i < bytes; ++i)
    data[i] = static_cast<std::uint8_t>((seq * 31U + i * 17U) & 255U);
}
bool inspect(const std::uint8_t* data, std::size_t bytes,
             std::uint64_t& seq, std::uint64_t& sent) {
  if (bytes < 16) return false;
  std::memcpy(&seq, data, sizeof(seq));
  std::memcpy(&sent, data + sizeof(seq), sizeof(sent));
  for (std::size_t i = 16; i < bytes; ++i)
    if (data[i] != static_cast<std::uint8_t>((seq * 31U + i * 17U) & 255U))
      return false;
  return true;
}
}
int main(int argc, char** argv) {
  try {
    // role mode shm_name payload_bytes frames callback_delay_us output.csv
    if (argc != 8)
      throw std::invalid_argument("usage: shm_load_bench sender|receiver copied|loaned /name bytes frames delay_us output.csv");
    const std::string role(argv[1]), mode(argv[2]), name(argv[3]);
    if ((role != "sender" && role != "receiver") ||
        (mode != "copied" && mode != "loaned") || name.size() < 2 || name[0] != '/')
      throw std::invalid_argument("invalid role/mode/shm name");
    const auto bytes = positive(argv[4]);
    const auto frames = positive(argv[5]);
    const std::string delay_string(argv[6]);
    if (delay_string.empty() || delay_string[0] == '-')
      throw std::invalid_argument("delay must be nonnegative");
    std::size_t delay_used = 0;
    const auto delay_us = std::stoull(delay_string, &delay_used);
    if (delay_used != delay_string.size() || delay_us > 1000000)
      throw std::invalid_argument("delay must be <= 1 second");
    if (bytes < 16 || bytes > robotcore::kMaxPayloadBytes)
      throw std::invalid_argument("payload outside [16, 16 MiB]");
    std::signal(SIGINT, stop_signal);
    std::signal(SIGTERM, stop_signal);
    robotcore::Configuration cfg;
    cfg.transport = robotcore::TransportKind::shared_memory;
    cfg.queue_depth = 1;
    cfg.max_payload_bytes = bytes;
    robotcore::SharedMemoryTransport transport(name, role == "receiver", cfg);
    const bool receive = role == "receiver";
    std::atomic<std::size_t> received{0};
    std::atomic<bool> invalid{false};
    std::vector<Observation> observed;
    if (receive) observed.reserve(frames);
    const auto callback = [&](std::uint64_t provided_seq, const std::uint8_t* payload,
                              std::size_t length) {
      const auto entered = now_ns();
      std::uint64_t embedded_seq = 0, sent = 0;
      bool ok = length == bytes && inspect(payload, length, embedded_seq, sent);
      const auto next = received.load(std::memory_order_relaxed) + 1;
      ok = ok && provided_seq == next && embedded_seq == next &&
           sent <= entered && sent > 0;
      observed.push_back({provided_seq, sent, entered, ok});
      if (!ok) invalid = true;
      if (delay_us) std::this_thread::sleep_for(microseconds(delay_us));
      received.store(next, std::memory_order_release);
    };
    robotcore::Status status;
    if (receive && mode == "loaned") {
      status = transport.start_borrowed([&](robotcore::BorrowedMessage msg) {
        callback(msg.sequence, msg.data, msg.size);
      });
    } else {
      status = transport.start([&](robotcore::Message msg) {
        if (receive) callback(msg.sequence, msg.payload.data(), msg.payload.size());
      });
    }
    if (!status) throw std::runtime_error("transport start: " + status.detail);
    std::ofstream out(argv[7], std::ios::trunc);
    if (!out) throw std::runtime_error("cannot write output CSV");
    if (receive) {
      std::cout << "ready" << std::endl;
      const auto deadline = steady_clock::now() + seconds(90);
      while (running && received.load(std::memory_order_acquire) < frames &&
             !invalid && steady_clock::now() < deadline)
        std::this_thread::sleep_for(milliseconds(1));
      const bool complete = received.load(std::memory_order_acquire) == frames;
      // Wait for the worker callback to return before reading its vector.
      status = transport.stop();
      out << "mode,seq,source_monotonic_ns,callback_monotonic_ns,delivery_us,valid\n";
      for (const auto& row : observed) {
        out << mode << ',' << row.seq << ',' << row.source_ns << ','
            << row.receiver_ns << ',';
        if (row.receiver_ns >= row.source_ns)
          out << (row.receiver_ns-row.source_ns)/1000.0;
        out << ',' << (row.valid ? "true" : "false") << '\n';
      }
      out.flush();
      if (!status || !complete || invalid || observed.size() != frames || !out) {
        std::cerr << "receiver incomplete or invalid: " << observed.size()
                  << '/' << frames << '\n';
        return 3;
      }
      return 0;
    }
    out << "mode,seq,payload_bytes,source_monotonic_ns,publish_duration_us,status\n";
    // Reuse owned payload storage, avoiding an unrelated per-frame allocation.\n    robotcore::Message owned_msg;\n    owned_msg.payload.resize(bytes);
    for (std::size_t seq = 1; seq <= frames && running; ++seq) {
      // Source time is taken before the transport API: delivery_us is an
      // application-intent-to-callback latency INCLUDING producer backpressure.
      const auto source = now_ns();
      if (mode == "loaned") {
        robotcore::SharedMemoryTransport::WriteLoan loan;
        status = transport.acquire_write_loan(loan);
        if (status) {
          fill(loan.data(), bytes, seq, source);
          status = loan.commit(seq, bytes);
        }
      } else {
        owned_msg.sequence = seq;
        fill(owned_msg.payload.data(), bytes, seq, source);
        status = transport.publish(owned_msg);
      }
      const auto after = now_ns();
      out << mode << ',' << seq << ',' << bytes << ',' << source << ','
          << (after-source)/1000.0 << ',' << (status ? "ok" : "error") << '\n';
      if (!status) {
        std::cerr << "send failed seq=" << seq << ": " << status.detail << '\n';
        break;
      }
    }
    out.flush();
    const bool complete = status && running && out.good();
    auto stopped = transport.stop();
    return complete && stopped ? 0 : 3;
  } catch (const std::exception& e) {
    std::cerr << "shm_load_bench: " << e.what() << '\n';
    return 3;
  }
}
