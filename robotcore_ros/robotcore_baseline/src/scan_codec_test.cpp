// Integration-level unit coverage for the full LaserScan CDR IPC envelope.
// Tests execute with the same Jazzy build as the three-process bridge.
#include "robotcore_ros/scan_codec.hpp"
#include "robotcore_ros/scan_sequence.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Scan = sensor_msgs::msg::LaserScan;
namespace codec = robotcore_ros::scan_codec;
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
bool equal(const Scan& a, const Scan& b) {
  return a.header.stamp.sec == b.header.stamp.sec &&
         a.header.stamp.nanosec == b.header.stamp.nanosec &&
         a.header.frame_id == b.header.frame_id &&
         a.angle_min == b.angle_min &&
         a.angle_max == b.angle_max &&
         a.angle_increment == b.angle_increment &&
         a.time_increment == b.time_increment &&
         a.scan_time == b.scan_time &&
         a.range_min == b.range_min &&
         a.range_max == b.range_max &&
         a.ranges == b.ranges &&
         a.intensities == b.intensities;
}
Scan sample() {
  Scan s;
  s.header.stamp.sec = 1730000001;
  s.header.stamp.nanosec = 987654321;
  s.header.frame_id = "robot/front/lidar";
  s.angle_min = -1.5707963f;
  s.angle_max = 1.5707963f;
  s.angle_increment = 0.01f;
  s.time_increment = 0.0001f;
  s.scan_time = 0.037f;
  s.range_min = 0.12f;
  s.range_max = 35.0f;
  s.ranges = {0.12f, 1.25f, 10.75f, 35.0f};
  s.intensities = {0.0f, 4.25f, 99.0f, 3.5f};
  return s;
}
bool rejected(const std::vector<std::uint8_t>& data) {
  try { (void)codec::decode(data); } catch (const std::exception&) { return true; }
  return false;
}
void test_codec() {
  const auto original = sample();
  const auto frame = codec::encode(original);
  require(frame.size() > codec::kEnvelopeBytes, "empty serialized message");
  require(codec::read_u32(frame.data()) == codec::kMagic, "wire magic mismatch");
  require(codec::read_u32(frame.data() + 4) == codec::kVersion, "wire version mismatch");
  require(codec::read_u32(frame.data() + 8) == frame.size() - codec::kEnvelopeBytes,
          "wire size mismatch");
  require(equal(original, codec::decode(frame)), "complete LaserScan roundtrip lost fields");

  auto mutated = frame;
  mutated[0] ^= 0x80;
  require(rejected(mutated), "corrupt magic accepted");

  mutated = frame;
  codec::write_u32(mutated.data() + 4, codec::kVersion + 1);
  require(rejected(mutated), "future envelope version accepted");

  mutated = frame;
  codec::write_u32(mutated.data() + 8, 0);
  require(rejected(mutated), "zero CDR size accepted");

  mutated = frame;
  codec::write_u32(mutated.data() + 8,
                   static_cast<std::uint32_t>(frame.size() - codec::kEnvelopeBytes + 1));
  require(rejected(mutated), "mismatched CDR size accepted");

  mutated = frame;
  mutated.pop_back();
  require(rejected(mutated), "truncated envelope accepted");

  mutated = frame;
  mutated.push_back(0);
  require(rejected(mutated), "trailing bytes accepted");

  require(rejected({}), "empty frame accepted");
  require(rejected({0x52, 0x43, 0x4c}), "short header accepted");
  require(rejected(std::vector<std::uint8_t>(codec::kMaxFrameBytes + 1, 0)),
          "oversized frame accepted");

  Scan full_scan = original;
  full_scan.header.stamp.nanosec = 1;
  full_scan.ranges.assign(360, 4.0f);
  full_scan.intensities.assign(360, 123.5f);
  require(equal(full_scan, codec::decode(codec::encode(full_scan))),
          "360-beam LaserScan lost intensities/ranges");
}
void test_strict_sequence() {
  robotcore_ros::StrictScanSequence tracker(3);
  require(!tracker.complete(), "empty sequence incorrectly marked complete");
  require(!tracker.accept(0), "zero sequence should be rejected");
  require(!tracker.accept(2), "missing first frame should be rejected");
  require(tracker.accept(1), "first frame not accepted");
  require(!tracker.accept(1), "duplicate frame should be rejected");
  require(!tracker.accept(3), "skipped frame should be rejected");
  require(tracker.accept(2), "second frame not accepted");
  require(tracker.accept(3), "third frame not accepted");
  require(tracker.complete(), "complete sequence not recognized");
  require(!tracker.accept(4), "out-of-range frame should be rejected");
  require(tracker.accepted() == 3, "rejected frames changed sequence count");
  bool rejected_zero_expected = false;
  try { robotcore_ros::StrictScanSequence invalid(0); (void)invalid; }
  catch (const std::invalid_argument&) { rejected_zero_expected = true; }
  require(rejected_zero_expected, "zero expected count accepted");
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    test_codec();
    test_strict_sequence();
    std::cout << "full_laserscan_codec_roundtrip=ok" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "scan codec test: " << e.what() << std::endl;
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
