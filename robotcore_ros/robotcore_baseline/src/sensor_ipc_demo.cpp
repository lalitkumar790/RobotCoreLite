// ROS 2 typed LaserScan -> RobotCore-Lite IPC -> checked receiver.
// Three OS processes. Functional integration only, not a latency benchmark.
#include "robotcore/ipc.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
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
constexpr std::size_t kPayloadBytes = kReadings * sizeof(float);
constexpr const char* kScanTopic = "/robotcore/native_scan";

float generated_range(std::size_t angle, std::uint64_t seq) {
  return 1.0f + static_cast<float>((angle + seq) % 100u) * 0.01f;
}
void write_u32(std::uint8_t* ptr, std::uint32_t n) {
  for (int k = 0; k < 4; ++k)
    ptr[k] = static_cast<std::uint8_t>(n >> (8 * k));
}
std::uint32_t read_u32(const std::uint8_t* ptr) {
  std::uint32_t result = 0;
  for (int k = 0; k < 4; ++k)
    result |= static_cast<std::uint32_t>(ptr[k]) << (8 * k);
  return result;
}
std::vector<std::uint8_t> encode_ranges(const Scan& scan) {
  std::vector<std::uint8_t> bytes(kPayloadBytes);
  for (std::size_t i = 0; i < kReadings; ++i) {
    std::uint32_t bits = 0;
    static_assert(sizeof(float) == sizeof(bits), "float32 expected");
    std::memcpy(&bits, &scan.ranges[i], sizeof(bits));
    write_u32(bytes.data() + i * 4, bits);
  }
  return bytes;
}
bool valid_scan(const Scan& scan) {
  if (scan.header.frame_id != "laser_frame" ||
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
bool valid_encoded(const robotcore::Message& message) {
  if (message.sequence == 0 || message.sequence > 1000000 ||
      message.payload.size() != kPayloadBytes) return false;
  for (std::size_t i = 0; i < kReadings; ++i) {
    const auto bits = read_u32(message.payload.data() + i * 4);
    float range = 0;
    std::memcpy(&range, &bits, sizeof(range));
    if (range != generated_range(i, message.sequence)) return false;
  }
  return true;
}
struct Options {
  std::string role;
  std::string socket;
  std::uint64_t samples{20};
};
Options parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; i += 2) {
    if (i + 1 >= argc) throw std::invalid_argument("option missing value");
    const std::string key = argv[i], value = argv[i + 1];
    if (key == "--role") o.role = value;
    else if (key == "--socket") o.socket = value;
    else if (key == "--samples") {
      if (value.empty() || value[0] == '-')
        throw std::invalid_argument("samples must be positive");
      std::size_t used = 0;
      o.samples = std::stoull(value, &used);
      if (used != value.size() || o.samples < 1 || o.samples > 1000000)
        throw std::invalid_argument("samples out of range");
    } else throw std::invalid_argument("unknown option: " + key);
  }
  if (o.role != "source" && o.role != "bridge" && o.role != "sink")
    throw std::invalid_argument("role must be source, bridge or sink");
  if (o.role != "source" && o.socket.empty())
    throw std::invalid_argument("--socket required for IPC roles");
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

robotcore::Configuration ipc_config() {
  robotcore::Configuration c;
  c.transport = robotcore::TransportKind::ipc;
  c.max_payload_bytes = kPayloadBytes;
  c.queue_depth = 1;
  return c;
}
int run_sink(const Options& o) {
  robotcore::UnixSocketTransport sink(o.socket, true, ipc_config());
  std::atomic<std::uint64_t> count{0};
  std::atomic<bool> invalid{false};
  std::uint64_t last_seq = 0; // callback worker only
  const auto started = sink.start([&](robotcore::Message message) {
    if (!valid_encoded(message) || message.sequence <= last_seq) {
      invalid = true;
      return;
    }
    last_seq = message.sequence;
    ++count;
  });
  if (!started) throw std::runtime_error("sink start: " + started.detail);
  const auto deadline = std::chrono::steady_clock::now() + 20s;
  while (count.load() < o.samples && !invalid &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(10ms);
  const auto result = sink.stop();
  if (!result) throw std::runtime_error("sink stop: " + result.detail);
  std::cout << "validated_native_scan_frames=" << count.load()
            << " invalid=" << invalid.load() << '\n';
  return !invalid && count >= o.samples ? 0 : 2;
}
int run_bridge(const Options& o, int argc, char** argv) {
  robotcore::UnixSocketTransport sender(o.socket, false, ipc_config());
  const auto started = sender.start([](robotcore::Message) {});
  if (!started) throw std::runtime_error("bridge IPC start: " + started.detail);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robotcore_scan_bridge");
  std::atomic<std::uint64_t> forwarded{0}, failed{0};
  auto subscriber = node->create_subscription<Scan>(
      kScanTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [&](Scan::ConstSharedPtr scan) {
        if (!valid_scan(*scan)) { ++failed; return; }
        const auto seq = static_cast<std::uint64_t>(scan->intensities[0]);
        if (!sender.publish({seq, encode_ranges(*scan)})) { ++failed; return; }
        ++forwarded;
      });
  (void)subscriber; // keep subscriber alive while spinning
  rclcpp::spin(node);
  const auto stopped = sender.stop();
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
    if (o.role == "sink") return run_sink(o);
    if (o.role == "bridge") return run_bridge(o, argc, argv);
    return run_source(o, argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "sensor-to-IPC demo: " << e.what() << '\n';
    return 1;
  }
}
