// Full native LaserScan -> versioned CDR -> selectable RobotCore-Lite transport.
// Three OS processes (plus a separate region manager for persistent SHM).
// Functional typed integration, not a latency or throughput benchmark.
#if defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
#include <std_msgs/msg/u_int8_multi_array.hpp>
#elif defined(ROBOTCORE_SENSOR_DDS)
// This mode uses native ROS 2 typed publication and subscription directly.
#elif defined(ROBOTCORE_SENSOR_SHM)
#include "robotcore/shared_memory.hpp"
#elif defined(ROBOTCORE_SENSOR_PSHM)
#include "robotcore/persistent_shm.hpp"
#else
#include "robotcore/ipc.hpp"
#endif
#include "robotcore_ros/scan_codec.hpp"
#include "robotcore_ros/scan_sequence.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <atomic>
#include <csignal>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <utility>

namespace {
using Scan = sensor_msgs::msg::LaserScan;
using namespace std::chrono_literals;
constexpr std::size_t kReadings = 360;
constexpr const char* kScanTopic = "/robotcore/native_scan";

struct Event {
  std::uint64_t sequence;
  std::int64_t monotonic_ns;
};
std::int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
// Buffer event records in memory and write once after worker/executor stops,
// keeping disk I/O out of the publication and receive callbacks.
void write_events(const std::string& csv, const std::vector<Event>& events) {
  if (csv.empty()) return;
  std::ofstream output(csv, std::ios::trunc);
  if (!output) throw std::runtime_error("cannot open event CSV: " + csv);
  output << "sequence,monotonic_ns\n";
  for (const auto& e : events)
    output << e.sequence << ',' << e.monotonic_ns << '\n';
  output.flush();
  if (!output) throw std::runtime_error("failed to write event CSV: " + csv);
}
#if defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
constexpr const char* kForwardedTopic = "/robotcore/forwarded_scan_cdr";
#endif
#if defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
constexpr const char* kBackendName = "dds_cdr_bridge";
#elif defined(ROBOTCORE_SENSOR_DDS)
constexpr const char* kBackendName = "dds_native";
#elif defined(ROBOTCORE_SENSOR_SHM)
constexpr const char* kBackendName = "shared_memory";
#elif defined(ROBOTCORE_SENSOR_PSHM)
constexpr const char* kBackendName = "persistent_shared_memory";
#else
constexpr const char* kBackendName = "ipc";
#endif

#if defined(ROBOTCORE_SENSOR_PSHM)
volatile std::sig_atomic_t manager_stop_signal = 0;
void request_manager_stop(int) { manager_stop_signal = 1; }
#endif

float generated_range(std::size_t angle, std::uint64_t seq) {
  return 1.0f + static_cast<float>((angle + seq) % 100u) * 0.01f;
}
// Validate every field in this deterministic demo, not merely the ranges.
// The CDR codec itself is generic and preserves arbitrary valid LaserScans.
bool valid_scan(const Scan& scan, std::uint64_t period_ms) {
  if (scan.header.frame_id != "laser_frame" ||
      scan.header.stamp.sec <= 0 ||
      scan.header.stamp.nanosec >= 1000000000u ||
      scan.angle_min != -3.14159265f ||
      scan.angle_max != 3.14159265f ||
      scan.angle_increment != 6.28318530f / static_cast<float>(kReadings) ||
      scan.range_min != 0.2f || scan.range_max != 30.0f ||
      scan.scan_time != static_cast<float>(period_ms) / 1000.0f ||
      scan.time_increment != static_cast<float>(period_ms) / 1000.0f / static_cast<float>(kReadings) ||
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
  std::uint64_t period_ms{20};
  std::string events_csv;
};
Options parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; i += 2) {
    if (i + 1 >= argc) throw std::invalid_argument("option missing value");
    const std::string key = argv[i], value = argv[i + 1];
    if (key == "--role") o.role = value;
    else if (key == "--socket" || key == "--shm-name") o.endpoint = value;
    else if (key == "--events-csv") o.events_csv = value;
    else if (key == "--period-ms") {
      if (value.empty() || value[0] == '-')
        throw std::invalid_argument("period-ms must be a positive integer");
      std::size_t used = 0;
      o.period_ms = std::stoull(value, &used);
      if (used != value.size() || o.period_ms < 1 || o.period_ms > 1000)
        throw std::invalid_argument("period-ms out of range (1..1000)");
    } else if (key == "--samples") {
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
#if defined(ROBOTCORE_SENSOR_DDS)
  if (o.role != "source" && o.role != "sink")
    throw std::invalid_argument("DDS reference role must be source or sink");
#else
  if (o.role != "source" && o.role != "bridge" && o.role != "sink")
    throw std::invalid_argument("role must be source, bridge or sink");
#endif
#endif
#if !defined(ROBOTCORE_SENSOR_DDS) && !defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
  if (o.role != "source" && o.endpoint.empty())
    throw std::invalid_argument("endpoint path or SHM name is required");
#endif
  return o;
}

class ScanSource final : public rclcpp::Node {
public:
  explicit ScanSource(std::uint64_t samples, std::uint64_t period_ms,
                      std::string events_csv)
    : rclcpp::Node("robotcore_scan_source"), limit_(samples), period_ms_(period_ms),
      events_csv_(std::move(events_csv)),
      ready_deadline_(std::chrono::steady_clock::now() + 15s) {
    if (!events_csv_.empty()) events_.reserve(limit_);
    publisher_ = create_publisher<Scan>(
        kScanTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms_), [this] {
      if (sequence_ > limit_) { rclcpp::shutdown(); return; }
      // Do not count early publications before the ROS subscription matches.
      if (publisher_->get_subscription_count() == 0) {
        if (std::chrono::steady_clock::now() >= ready_deadline_) {
          RCLCPP_ERROR(this->get_logger(), "No LaserScan subscriber matched within 15 s");
          rclcpp::shutdown();
        }
        return;
      }
      Scan scan;
      scan.header.stamp = this->now();
      scan.header.frame_id = "laser_frame";
      scan.angle_min = -3.14159265f;
      scan.angle_max = 3.14159265f;
      scan.angle_increment = 6.28318530f / static_cast<float>(kReadings);
      scan.range_min = 0.2f;
      scan.range_max = 30.0f;
      scan.scan_time = static_cast<float>(period_ms_) / 1000.0f;
      scan.time_increment = scan.scan_time / static_cast<float>(kReadings);
      scan.intensities = {static_cast<float>(sequence_)};
      scan.ranges.reserve(kReadings);
      for (std::size_t i = 0; i < kReadings; ++i)
        scan.ranges.push_back(generated_range(i, sequence_));
      const auto stamp = monotonic_ns();  // immediately before ROS publish
      publisher_->publish(scan);
      if (!events_csv_.empty()) events_.push_back({sequence_, stamp});
      ++sequence_;
    });
  }
  std::uint64_t published_count() const noexcept { return sequence_ - 1; }
  void flush_events() const { write_events(events_csv_, events_); }
private:
  std::uint64_t sequence_{1}, limit_, period_ms_;
  std::string events_csv_;
  std::vector<Event> events_;
  std::chrono::steady_clock::time_point ready_deadline_;
  rclcpp::Publisher<Scan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#if !defined(ROBOTCORE_SENSOR_DDS) && !defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
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

#endif  // Native transports

int run_sink(const Options& o) {
#if defined(ROBOTCORE_SENSOR_DDS) || defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("robotcore_dds_scan_sink");
#else
  auto sink = make_transport(o, true);
#endif
  std::atomic<std::uint64_t> count{0};
  std::atomic<bool> invalid{false};
  std::vector<Event> observed;
  if (!o.events_csv.empty()) observed.reserve(o.samples);
  robotcore_ros::StrictScanSequence tracker(o.samples);  // callback thread only
  auto check_scan = [&](const Scan& scan, std::uint64_t seq,
                        std::int64_t arrival_ns) {
    if (!valid_scan(scan, o.period_ms) ||
        seq != static_cast<std::uint64_t>(scan.intensities[0]) ||
        !tracker.accept(seq)) {
      invalid = true;
      return;
    }
    if (!o.events_csv.empty()) observed.push_back({seq, arrival_ns});
    ++count;
  };
#if defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
  auto subscriber = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
      kForwardedTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [&](std_msgs::msg::UInt8MultiArray::ConstSharedPtr packet) {
        const auto received_ns = monotonic_ns();
        try {
          const Scan scan = robotcore_ros::scan_codec::decode(packet->data);
          const auto seq = static_cast<std::uint64_t>(
              scan.intensities.empty() ? 0.0f : scan.intensities[0]);
          check_scan(scan, seq, received_ns);
        } catch (const std::exception&) {
          invalid = true;
        }
      });
  (void)subscriber;
#elif defined(ROBOTCORE_SENSOR_DDS)
  auto subscriber = node->create_subscription<Scan>(
      kScanTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [&](Scan::ConstSharedPtr scan) {
        const auto received_ns = monotonic_ns();
        check_scan(*scan, static_cast<std::uint64_t>(
            scan->intensities.empty() ? 0.0f : scan->intensities[0]), received_ns);
      });
  (void)subscriber;
#else
  const auto started = sink->start([&](robotcore::Message message) {
    const auto received_ns = monotonic_ns();
    try {
      const Scan scan = robotcore_ros::scan_codec::decode(message.payload);
      check_scan(scan, message.sequence, received_ns);
    } catch (const std::exception&) {
      invalid = true;
    }
  });
  if (!started) throw std::runtime_error("sink start: " + started.detail);
#endif
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(o.samples * o.period_ms + 15000);
  while (count.load() < o.samples && !invalid &&
         std::chrono::steady_clock::now() < deadline) {
#if defined(ROBOTCORE_SENSOR_DDS) || defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(1ms);
#else
    std::this_thread::sleep_for(2ms);
#endif
  }
#if defined(ROBOTCORE_SENSOR_DDS) || defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
  rclcpp::shutdown();
#else
  const auto result = sink->stop();
  if (!result) throw std::runtime_error("sink stop: " + result.detail);
#endif
  write_events(o.events_csv, observed);
  std::cout << "validated_native_scan_frames=" << count.load()
            << " expected=" << o.samples
            << " invalid=" << invalid.load() << '\n';
  return !invalid && count == o.samples ? 0 : 2;
}
#if defined(ROBOTCORE_SENSOR_DDS_BRIDGE)
int run_bridge(const Options& o, int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robotcore_scan_cdr_dds_bridge");
  auto publisher = node->create_publisher<std_msgs::msg::UInt8MultiArray>(
      kForwardedTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  std::atomic<std::uint64_t> forwarded{0}, failed{0};
  std::atomic<bool> ready{false};
  auto subscriber = node->create_subscription<Scan>(
      kScanTopic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [&](Scan::ConstSharedPtr scan) {
        if (!valid_scan(*scan, o.period_ms) || !ready.load()) {
          ++failed;
          return;
        }
        try {
          std_msgs::msg::UInt8MultiArray packet;
          packet.data = robotcore_ros::scan_codec::encode(*scan);
          publisher->publish(packet);
          ++forwarded;
        } catch (const std::exception&) {
          ++failed;
        }
      });
  (void)subscriber;
  // Prevent the source from publishing before the second DDS hop is matched.
  // The runner observes this marker before launching the source.
  auto readiness = node->create_wall_timer(50ms, [&] {
    if (!ready && publisher->get_subscription_count() != 0) {
      ready = true;
      std::cout << "bridge_ready=1" << std::endl;
    }
  });
  (void)readiness;
  rclcpp::spin(node);
  rclcpp::shutdown();
  std::cout << "bridge_forwarded=" << forwarded << " errors=" << failed << '\n';
  return failed ? 2 : 0;
}
#elif !defined(ROBOTCORE_SENSOR_DDS)
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
        if (!valid_scan(*scan, o.period_ms)) { ++failed; return; }
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
#endif
int run_source(const Options& o, int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanSource>(o.samples, o.period_ms, o.events_csv);
  rclcpp::spin(node);
  const auto sent = node->published_count();
  rclcpp::shutdown();
  node->flush_events();
  std::cout << "published_native_scan_frames=" << sent << " expected=" << o.samples << '\n';
  return sent == o.samples ? 0 : 2;
}
} // namespace

int main(int argc, char** argv) {
  try {
    const auto o = parse(argc, argv);
#if defined(ROBOTCORE_SENSOR_PSHM)
    if (o.role == "manager") return run_manager(o);
#endif
    if (o.role == "sink") return run_sink(o);
#if !defined(ROBOTCORE_SENSOR_DDS)
    if (o.role == "bridge") return run_bridge(o, argc, argv);
#endif
    return run_source(o, argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "sensor-to-" << kBackendName << " demo: " << e.what() << '\n';
    return 1;
  }
}
