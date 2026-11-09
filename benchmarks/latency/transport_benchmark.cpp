// Shared stop-and-wait benchmark for ROS 2 DDS and Linux IPC.
// Build the SAME source as transport_bench_dds and transport_bench_ipc.
// This measures request/reply RTT, never direct one-way latency.
#include "robotcore/core.hpp"
#if defined(ROBOTCORE_BENCH_DDS)
#include "robotcore_ros/dds_transport.hpp"
#include <rclcpp/rclcpp.hpp>
#elif defined(ROBOTCORE_BENCH_IPC)
#include "robotcore/ipc.hpp"
#else
#error "Select ROBOTCORE_BENCH_DDS or ROBOTCORE_BENCH_IPC"
#endif
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
std::atomic<bool> running{true};
void stop_signal(int) { running.store(false); }

struct Options {
  std::string role{"responder"};
  std::string csv;
  std::string run_id{"manual"};
  std::string socket_path;
  std::uint64_t samples{1000};
  std::uint64_t warmup{100};
  std::uint64_t timeout_ms{1000};
  std::uint64_t discovery_timeout_ms{15000};
  std::size_t payload_bytes{64};
  std::size_t qos_depth{10};
  bool reliable{true};
};

std::uint64_t positive_integer(const std::string& raw, const std::string& name,
                               std::uint64_t min, std::uint64_t max) {
  if (raw.empty() || raw[0] == '-') throw std::invalid_argument("invalid " + name);
  std::size_t used = 0;
  const auto value = std::stoull(raw, &used, 10);
  if (used != raw.size() || value < min || value > max)
    throw std::invalid_argument(name + " out of range");
  return value;
}

Options parse(int argc, char** argv) {
  Options opts;
  for (int i = 1; i < argc; i += 2) {
    if (i + 1 >= argc) throw std::invalid_argument("every option requires a value");
    const std::string key = argv[i], val = argv[i + 1];
    if (key == "--role") opts.role = val;
    else if (key == "--csv") opts.csv = val;
    else if (key == "--run-id") opts.run_id = val;
    else if (key == "--socket") opts.socket_path = val;
    else if (key == "--samples") opts.samples = positive_integer(val, key, 1, 10000000);
    else if (key == "--warmup") opts.warmup = positive_integer(val, key, 1, 1000000);
    else if (key == "--timeout-ms") opts.timeout_ms = positive_integer(val, key, 1, 60000);
    else if (key == "--discovery-timeout-ms") opts.discovery_timeout_ms = positive_integer(val, key, 1, 60000);
    else if (key == "--payload-bytes") opts.payload_bytes =
      static_cast<std::size_t>(positive_integer(val, key, 1, robotcore::kMaxPayloadBytes));
    else if (key == "--qos-depth") opts.qos_depth =
      static_cast<std::size_t>(positive_integer(val, key, 1, 10000));
    else if (key == "--reliability") {
      if (val != "reliable" && val != "best_effort")
        throw std::invalid_argument("reliability must be reliable or best_effort");
      opts.reliable = val == "reliable";
    } else throw std::invalid_argument("unknown argument: " + key);
  }
  if (opts.role != "initiator" && opts.role != "responder")
    throw std::invalid_argument("role must be initiator or responder");
#if defined(ROBOTCORE_BENCH_IPC)
  if (opts.socket_path.empty()) throw std::invalid_argument("--socket required for IPC");
  if (!opts.reliable) throw std::invalid_argument("IPC does not implement best_effort");
#endif
  if (opts.role == "initiator" && opts.csv.empty())
    throw std::invalid_argument("--csv required for initiator");
  if (opts.run_id.find('\n') != std::string::npos ||
      opts.run_id.find('\r') != std::string::npos)
    throw std::invalid_argument("run-id must be a single line");
  return opts;
}

std::string csv_escape(const std::string& value) {
  std::string escaped(1, '"');
  for (const char c : value) {
    if (c == '"') escaped += "\"\"";
    else escaped += c;
  }
  return escaped + '"';
}

class RttSampler {
public:
  RttSampler(robotcore::Transport& transport, const Options& opts)
    : transport_(transport), opts_(opts), expected_payload_(opts.payload_bytes, 0xa5) {}

  void on_reply(robotcore::Message reply) {
    const auto arrived = Clock::now();
    if (reply.payload != expected_payload_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (reply.sequence != sequence_ || acknowledged_) return;
    acknowledged_ = true;
    arrived_ = arrived;
    condition_.notify_one();
  }

  int run() {
    // Probes use sequence zero and are never included in warmup or CSV.
    // In particular, DDS discovery can take longer than a measurement timeout.
    if (!await_discovery()) {
      std::cerr << "No working echo endpoint before discovery deadline\n";
      return 3;
    }
    std::ofstream out(opts_.csv, std::ios::trunc);
    if (!out) throw std::runtime_error("cannot create CSV: " + opts_.csv);
#if defined(ROBOTCORE_BENCH_DDS)
    constexpr const char* transport_name = "dds";
#else
    constexpr const char* transport_name = "ipc";
#endif
    out << "run_id,transport,seq,payload_bytes,status,rtt_us,rtt_half_proxy_us\n";
    std::uint64_t successes = 0, timeouts = 0;
    const std::uint64_t total = opts_.warmup + opts_.samples;
    for (std::uint64_t seq = 1; seq <= total && running; ++seq) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        sequence_ = seq;
        acknowledged_ = false;
      }
      const auto started = Clock::now();
      const auto sent = transport_.publish({seq, expected_payload_});
      if (!sent) {
        std::cerr << "publish error on sequence " << seq << ": " << sent.detail << "\n";
        return 3;
      }
      std::unique_lock<std::mutex> lock(mutex_);
      const bool received = condition_.wait_for(
        lock, std::chrono::milliseconds(opts_.timeout_ms),
        [this] { return acknowledged_ || !running.load(); });
      if (!running) return 130;
      const double elapsed_us = received && acknowledged_
        ? std::chrono::duration<double, std::micro>(arrived_ - started).count() : 0.0;
      lock.unlock();
      if (seq <= opts_.warmup) {
        if (!received) {
          std::cerr << "warmup timeout on sequence " << seq
                    << "; no comparable measurements produced\n";
          return 3;
        }
        continue;
      }
      out << csv_escape(opts_.run_id) << ',' << transport_name << ',' << seq << ','
          << opts_.payload_bytes << ',' << (received ? "ok" : "timeout") << ',';
      if (received) {
        out << std::fixed << std::setprecision(3) << elapsed_us << ','
            << elapsed_us / 2.0;
        ++successes;
      } else {
        out << ',';
        ++timeouts;
      }
      out << '\n';
    }
    out.flush();
    if (!out) throw std::runtime_error("CSV write failed");
    std::cout << "transport=" << transport_name << " received=" << successes
              << " timed_out=" << timeouts << " recorded=" << successes + timeouts << "\n";
    return timeouts ? 2 : 0;
  }
private:
  bool await_discovery() {
    const auto deadline = Clock::now() + std::chrono::milliseconds(opts_.discovery_timeout_ms);
    while (running && Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        sequence_ = 0;
        acknowledged_ = false;
      }
      const auto status = transport_.publish({0, expected_payload_});
      if (!status) {
        std::cerr << "Discovery publish failed: " << status.detail << "\n";
        return false;
      }
      std::unique_lock<std::mutex> lock(mutex_);
      if (condition_.wait_for(lock, std::chrono::milliseconds(400),
            [this] { return acknowledged_ || !running.load(); }) && acknowledged_) {
        return true;
      }
    }
    return false;
  }

  robotcore::Transport& transport_;
  const Options& opts_;
  std::vector<std::uint8_t> expected_payload_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::uint64_t sequence_{0};
  bool acknowledged_{false};
  Clock::time_point arrived_{};
};

int run(int argc, char** argv) {
  const auto opts = parse(argc, argv);
  std::signal(SIGTERM, stop_signal);
  std::signal(SIGINT, stop_signal);
  robotcore::Configuration config;
#if defined(ROBOTCORE_BENCH_DDS)
  config.transport = robotcore::TransportKind::dds;
#else
  config.transport = robotcore::TransportKind::ipc;
#endif
  config.max_payload_bytes = opts.payload_bytes;
  config.queue_depth = opts.qos_depth;
  config.reliable = opts.reliable;

#if defined(ROBOTCORE_BENCH_DDS)
  rclcpp::init(argc, argv);
  struct RosShutdown {
    ~RosShutdown() { if (rclcpp::ok()) rclcpp::shutdown(); }
  } ros_shutdown;
  auto node = std::make_shared<rclcpp::Node>("robotcore_compare_" + opts.role);
  const bool responder = opts.role == "responder";
  robotcore_ros::DdsTransport transport(node,
    responder ? "robotcore/compare/reply" : "robotcore/compare/request",
    responder ? "robotcore/compare/request" : "robotcore/compare/reply", config);
#else
  const bool responder = opts.role == "responder";
  robotcore::UnixSocketTransport transport(opts.socket_path, responder, config);
#endif

  if (responder) {
    const auto started = transport.start([&transport](robotcore::Message msg) {
      const auto result = transport.publish(msg);
      if (!result) std::cerr << "responder publish failed: " << result.detail << "\n";
    });
    if (!started) throw std::runtime_error("responder start: " + started.detail);
#if defined(ROBOTCORE_BENCH_DDS)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (running && rclcpp::ok()) executor.spin_some(std::chrono::milliseconds(10));
    executor.remove_node(node);
#else
    while (running) std::this_thread::sleep_for(std::chrono::milliseconds(20));
#endif
    transport.stop();
    return 0;
  }

  RttSampler sampler(transport, opts);
  const auto started = transport.start([&sampler](robotcore::Message msg) {
    sampler.on_reply(std::move(msg));
  });
  if (!started) throw std::runtime_error("initiator start: " + started.detail);
#if defined(ROBOTCORE_BENCH_DDS)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor, &node] {
    (void)node;
    while (running && rclcpp::ok())
      executor.spin_some(std::chrono::milliseconds(10));
  });
  int result = 1;
  try {
    result = sampler.run();
  } catch (...) {
    running = false;
    executor_thread.join();
    executor.remove_node(node);
    transport.stop();
    throw;
  }
  running = false;
  executor_thread.join();
  executor.remove_node(node);
#else
  const int result = sampler.run();
#endif
  transport.stop();
  return result;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    return run(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "benchmark error: " << e.what() << "\n";
    return 1;
  }
}
