// Local-only SHM copied-versus-loaned RTT experiment. NOT a DDS comparison.
// Two processes; one outstanding frame; all RTTs use steady_clock on initiator.
#include "robotcore/shared_memory.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono;
namespace {
std::atomic<bool> active{true};
void terminate(int) { active = false; }
std::uint8_t pattern(std::uint64_t seq, std::size_t index) {
  return static_cast<std::uint8_t>((seq * 31U + index * 17U) & 255U);
}
void check(const std::uint8_t* p, std::size_t len, std::uint64_t seq) {
  for (std::size_t j = 0; j < len; ++j)
    if (p[j] != pattern(seq, j)) throw std::runtime_error("payload corruption");
}
std::size_t number(const char* text) {
  const std::string s(text);
  if (s.empty() || s[0] == '-') throw std::invalid_argument("positive integer required");
  std::size_t consumed = 0;
  auto n = std::stoull(s, &consumed);
  if (consumed != s.size() || n == 0) throw std::invalid_argument("positive integer required");
  return static_cast<std::size_t>(n);
}
}
int main(int argc, char** argv) {
  try {
    // role, mode, POSIX shm name, bytes, warmup, samples, CSV path (initiator)
    if (argc != 7 && argc != 8) {
      std::cerr << "usage: shm_path_bench responder|initiator copied|loaned /name bytes warmup samples [csv]\n";
      return 1;
    }
    const std::string role(argv[1]), mode(argv[2]), name(argv[3]);
    const bool responder = role == "responder";
    if ((!responder && role != "initiator") || (mode != "copied" && mode != "loaned") ||
        name.empty() || name[0] != '/' || (!responder && argc != 8))
      throw std::invalid_argument("invalid role/mode/name/CSV");
    const std::size_t bytes = number(argv[4]), warmup = number(argv[5]), samples = number(argv[6]);
    if (bytes > robotcore::kMaxPayloadBytes || warmup > 1000000 || samples > 10000000)
      throw std::invalid_argument("benchmark bounds exceeded");
    std::signal(SIGTERM, terminate);
    std::signal(SIGINT, terminate);
    robotcore::Configuration cfg;
    cfg.transport = robotcore::TransportKind::shared_memory;
    cfg.queue_depth = 1;
    cfg.max_payload_bytes = bytes;
    robotcore::SharedMemoryTransport transport(name, responder, cfg);
    std::mutex mutex;
    std::condition_variable wake;
    std::uint64_t received = 0;
    bool corrupt = false;
    auto reply = [&](std::uint64_t seq, const std::uint8_t* data, std::size_t length) {
      // Validation belongs to the measured application callback in BOTH modes.
      bool valid = length == bytes;
      if (valid) {
        for (std::size_t j = 0; j < length; ++j) {
          if (data[j] != pattern(seq, j)) { valid = false; break; }
        }
      }
      if (!valid) { std::lock_guard<std::mutex> guard(mutex); corrupt = true; wake.notify_one(); return; }
      if (responder) {
        if (mode == "loaned") {
          robotcore::SharedMemoryTransport::WriteLoan loan;
          auto status = transport.acquire_write_loan(loan);
          if (!status) { active = false; return; }
          // Echo requires an intentional read/write copy: loaning eliminates
          // the extra temporary vector and the transport's slot copy.
          for (std::size_t j = 0; j < bytes; ++j) loan.data()[j] = data[j];
          status = loan.commit(seq, bytes);
          if (!status) active = false;
        } else {
          robotcore::Message echo;
          echo.sequence = seq;
          echo.payload.assign(data, data + length);
          if (!transport.publish(echo)) active = false;
        }
      } else {
        std::lock_guard<std::mutex> guard(mutex);
        received = seq;
        wake.notify_one();
      }
    };
    const auto started = mode == "loaned"
      ? transport.start_borrowed([&](robotcore::BorrowedMessage msg) {
          reply(msg.sequence, msg.data, msg.size);
        })
      : transport.start([&](robotcore::Message msg) {
          reply(msg.sequence, msg.payload.data(), msg.payload.size());
        });
    if (!started) throw std::runtime_error("start: " + started.detail);
    if (responder) {
      std::cout << "ready\n" << std::flush;
      while (active) std::this_thread::sleep_for(milliseconds(20));
      if (!transport.stop()) return 3;
      return 0;
    }
    std::ofstream output(argv[7], std::ios::trunc);
    if (!output) throw std::runtime_error("cannot open output CSV");
    output << "mode,seq,payload_bytes,status,rtt_us\n";
    const auto send = [&](std::uint64_t seq) {
      if (mode == "loaned") {
        robotcore::SharedMemoryTransport::WriteLoan loan;
        auto status = transport.acquire_write_loan(loan);
        if (!status) return status;
        for (std::size_t j = 0; j < bytes; ++j) loan.data()[j] = pattern(seq,j);
        return loan.commit(seq, bytes);
      }
      robotcore::Message msg;
      msg.sequence = seq;
      msg.payload.resize(bytes);
      for (std::size_t j = 0; j < bytes; ++j) msg.payload[j] = pattern(seq,j);
      return transport.publish(msg);
    };
    // Probe for endpoint readiness; never mix discovery RTT with samples.
    bool ready = false;
    const auto discovery_end = steady_clock::now() + seconds(15);
    while (active && steady_clock::now() < discovery_end) {
      auto status = send(0);
      if (!status) throw std::runtime_error("discovery publish: " + status.detail);
      std::unique_lock<std::mutex> lock(mutex);
      if (wake.wait_for(lock, milliseconds(300), [&]{ return received == 0 || corrupt; })) {
        if (corrupt) throw std::runtime_error("discovery corruption");
        ready = true; break;
      }
    }
    if (!ready) throw std::runtime_error("discovery timeout");
    int result = 0;
    for (std::size_t seq = 1; seq <= warmup + samples && active; ++seq) {
      const auto start = steady_clock::now();
      auto status = send(seq);
      if (!status) throw std::runtime_error("publish: " + status.detail);
      std::unique_lock<std::mutex> lock(mutex);
      const bool done = wake.wait_for(lock, milliseconds(1000), [&] {
        return received == seq || corrupt || !active;
      });
      const auto end = steady_clock::now();
      if (corrupt) throw std::runtime_error("payload corruption");
      if (seq <= warmup) {
        if (!done || received != seq) throw std::runtime_error("warmup timeout");
        continue;
      }
      const bool ok = done && received == seq;
      output << mode << ',' << seq << ',' << bytes << ',' << (ok ? "ok" : "timeout") << ',';
      if (ok) output << duration<double,std::micro>(end-start).count();
      output << '\n';
      if (!ok) result = 2;
    }
    output.flush();
    if (!output) throw std::runtime_error("failed to write CSV");
    if (!transport.stop()) return 3;
    return active ? result : 130;
  } catch (const std::exception& e) {
    std::cerr << "shm_path_bench: " << e.what() << '\n';
    return 3;
  }
}
