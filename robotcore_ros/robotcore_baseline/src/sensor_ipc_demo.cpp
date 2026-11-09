// Full native LaserScan -> versioned CDR -> selectable RobotCore-Lite transport.
// Three OS processes (plus a separate region manager for persistent SHM).
// Functional typed integration, not a latency or throughput benchmark.
#if defined(ROBOTCORE_SENSOR_SHM)
#include "robotcore/shared_memory.hpp"
#elif defined(ROBOTCORE_SENSOR_PSHM)
#include "robotcore/persistent_shm.hpp"
#else
#include "robotcore/ipc.hpp"
#endif
#include "robotcore_ros/scan_codec.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Scan = sensor_msgs::msg::LaserScan;
using namespace std::chrono_literals;
constexpr std::size_t kReadings = 360;
constexpr const char* kScanTopic = "/robotcore/native_scan";
#if defined(ROBOTCORE_SENSOR_SHM)
constexpr const char* kBackendName = "shared_memory";
#elif defined(ROBOTCORE_SENSOR_PSHM)
constexpr const char* kBackendName = "persistent_shared_memory";
#else
constexpr const char* kBackendName = "ipc";
#endif

volatile std::sig_atomic_t manager_stop_signal = 0;
void request_manager_stop(int) { manager_stop_signal = 1; }

float generated_range(std::size_t angle, std::uint64_t seq) {
  return 1.0f + static_cast<float>((angle + seq) % 100u) * 0.01f;
}
// Validate every field in this deterministic demo, not merely the ranges.
// The CDR codec itself is generic and preserves arbitrary valid LaserScans.
bool valid_scan(const Scan& scan) {
  if (scan.header.frame_id != "laser_frame" ||
      scan.header.stamp.sec <= 0 ||
      scan.header.stamp.nanosec >= 1000000000u ||
      scan.angle_min != -3.14159265f ||
      scan.angle_max != 3.14159265f ||
      scan.angle_increment != 6.28318530f / static_cast<float>(kReadings) ||
      scan.range_min != 0.2f || scan.range_max != 30.0f ||
      scan.scan_time != 0.02f ||
      scan.time_increment != 0.02f / static_cast<float>(kReadings) ||
      scan.ranges.size() != kReadings || scan.intensities.size() != 1 ||
      !std::isfinite(scan.intensities[0]) ||
      scan.intensities[0] < 1.0f || scan.intensities[0] > 1000000.0f)
    return false;
  const auto seq = static_cast<std::uint64_t>(scan.intensities[0]);
  if (static_cast<float>(seq) != scan.intensities[0]) return false;
  for (std::size_t i = 0; i < kReadings; ++i)
    if (scan.ranges[i] != generated_range(i, seq)) return false;
  return true;
}
struct Options {
  std::string role;
  std::string endpoint;
  std::uint64_t samples{20};
};
Options parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; i += 2) {
    if (i + 1 >= argc) throw std::invalid_argument("option missing value");
    const std::string key = argv[i], value = argv[i + 1];
    if (key == "--role") o.role = value;
    else if (key == "--socket" || key == "--shm-name") o.endpoint = value;
    else if (key == "--samples") {
      if (value.empty() || value[0] == '-')
        throw std::invalid_argument("samples must be positive");
      std::size_t used = 0;
      o.samples = std::stoull(value, &used);
      if (used != value.size() || o.samples < 1 || o.samples > 1000000)
        throw std::invalid_argument("samples out of range");
    } else throw std::invalid_argument("unknown option: " + key);
  }
#if defined(ROBOTCORE_SENSOR_PSHM)
  if (o.role != "source" && o.role != "bridge" &&
      o.role != "sink" && o.role != "manager")
    throw std::invalid_argument("role must be source, bridge, sink or manager");
#else
  if (o.role != "source" && o.role != "bridge" && o.role != "sink")
    throw std::invalid_argument("role must be source, bridge or sink");
#endif
  if (o.role != "source" && o.endpoint.empty())
    throw std::invalid_argument("endpoint path or SHM name is required");
  return o;
}

class ScanSource final : public rclcpp::Node {
public:
  explicit ScanSource(std::uint64_t samples)
    : rclcpp::Node("robotcore_scan_source"), limit_(samples) {
    publisher_ = create_publisher<Scan>(
        kScanTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    timer_ = create_wall_timer(20ms, [this] {
      if (sequence_ > limit_) { rclcpp::shutdown(); return; }
      Scan scan;
      scan.header.stamp = this->now();
      scan.header.frame_id = "laser_frame";
      scan.angle_min = -3.14159265f;
      scan.angle_max = 3.14159265f;
      scan.angle_increment = 6.28318530f / static_cast<float>(kReadings);
      scan.range_min = 0.2f;
      scan.range_max = 30.0f;
      scan.scan_time = 0.02f;
      scan.time_increment = 0.02f / static_cast<float>(kReadings);
      scan.intensities = {static_cast<float>(sequence_)};
      scan.ranges.reserve(kReadings);
      for (std::size_t i = 0; i < kReadings; ++i)
        scan.ranges.push_back(generated_range(i, sequence_));
      publisher_->publish(scan);
      ++sequence_;
    });
  }
private:
  std::uint64_t sequence_{1}, limit_;
  rclcpp::Publisher<Scan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

robotcore::Configuration sensor_transport_config() {
  robotcore::Configuration c;
#if defined(ROBOTCORE_SENSOR_SHM)
  c.transport = robotcore::TransportKind::shared_memory;
#elif defined(ROBOTCORE_SENSOR_PSHM)
  c.transport = robotcore::TransportKind::persistent_shared_memory;
#else
  c.transport = robotcore::TransportKind::ipc;
#endif
  c.max_payload_bytes = robotcore_ros::scan_codec::kMaxFrameBytes;
  c.queue_depth = 1;
  c.reliable = true;
  return c;
}

std::unique_ptr<robotcore::Transport> make_transport(const Options& o, bool server) {
#if defined(ROBOTCORE_SENSOR_SHM)
  return std::make_unique<robotcore::SharedMemoryTransport>(
      o.endpoint, server, sensor_transport_config());
#elif defined(ROBOTCORE_SENSOR_PSHM)
  return std::make_unique<robotcore::PersistentSharedMemoryTransport>(
      o.endpoint, server, sensor_transport_config());
#else
  return std::make_unique<robotcore::UnixSocketTransport>(
      o.endpoint, server, sensor_transport_config());
#endif
}

#if defined(ROBOTCORE_SENSOR_PSHM)
int run_manager(const Options& o) {
  robotcore::PersistentRegionManager manager(
      o.endpoint, robotcore_ros::scan_codec::kMaxFrameBytes);
  auto status = manager.start();
  if (!status) throw std::runtime_error("region manager start: " + status.detail);
  status = manager.begin_session();
  if (!status) throw std::runtime_error("region manager epoch: " + status.detail);
  manager_stop_signal = 0;
  std::signal(SIGTERM, request_manager_stop);
  std::signal(SIGINT, request_manager_stop);
  std::cout << "manager_ready_epoch=" << manager.epoch() << std::endl;
  while (!manager_stop_signal)
    std::this_thread::sleep_for(20ms);
  status = manager.stop();
  if (!status) throw std::runtime_error("region manager stop: " + status.detail);
  return 0;
}
#endif

int run_sink(const Options& o) {
  auto sink = make_transport(o, true);
  std::atomic<std::uint64_t> count{0};
  std::atomic<bool> invalid{false};
  std::uint64_t last_seq = 0; // callback worker only
  const auto started = sink->start([&](robotcore::Message message) {
    try {
      const Scan scan = robotcore_ros::scan_codec::decode(message.payload);
      if (!valid_scan(scan) ||
          message.sequence != static_cast<std::uint64_t>(scan.intensities[0]) ||
          message.sequence <= last_seq) {
        invalid = true;
        return;
      }
      last_seq = message.sequence;
      ++count;
    } catch (const std::exception&) {
      invalid = true;
    }
  });
  if (!started) throw std::runtime_error("sink start: " + started.detail);
  const auto deadline = std::chrono::steady_clock::now() + 20s;
  while (count.load() < o.samples && !invalid &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(10ms);
  const auto result = sink->stop();
  if (!result) throw std::runtime_error("sink stop: " + result.detail);
  std::cout << "validated_native_scan_frames=" << count.load()
            << " invalid=" << invalid.load() << '\n';
  return !invalid && count >= o.samples ? 0 : 2;
}
int run_bridge(const Options& o, int argc, char** argv) {
  auto sender = make_transport(o, false);
  const auto started = sender->start([](robotcore::Message) {});
  if (!started) throw std::runtime_error("bridge IPC start: " + started.detail);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robotcore_scan_bridge");
  std::atomic<std::uint64_t> forwarded{0}, failed{0};
  auto subscriber = node->create_subscription<Scan>(
      kScanTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [&](Scan::ConstSharedPtr scan) {
        if (!valid_scan(*scan)) { ++failed; return; }
        try {
          const auto seq = static_cast<std::uint64_t>(scan->intensities[0]);
          const auto frame = robotcore_ros::scan_codec::encode(*scan);
          if (!sender->publish({seq, frame})) { ++failed; return; }
          ++forwarded;
        } catch (const std::exception&) {
          ++failed;
        }
      });
  (void)subscriber; // keep subscriber alive while spinning
  rclcpp::spin(node);
  const auto stopped = sender->stop();
  rclcpp::shutdown();
  if (!stopped) throw std::runtime_error("bridge IPC stop failed");
  std::cout << "bridge_forwarded=" << forwarded << " errors=" << failed << '\n';
  return failed ? 2 : 0;
}
int run_source(const Options& o, int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanSource>(o.samples);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
} // namespace

int main(int argc, char** argv) {
  try {
    const auto o = parse(argc, argv);
#if defined(ROBOTCORE_SENSOR_PSHM)
    if (o.role == "manager") return run_manager(o);
#endif
    if (o.role == "sink") return run_sink(o);
    if (o.role == "bridge") return run_bridge(o, argc, argv);
    return run_source(o, argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "sensor-to-" << kBackendName << " demo: " << e.what() << '\n';
    return 1;
  }
}
