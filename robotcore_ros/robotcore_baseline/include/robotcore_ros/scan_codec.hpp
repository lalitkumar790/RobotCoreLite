#pragma once

// Bounded, versioned envelope around the ROS 2 CDR serialization of the
// *complete* sensor_msgs/msg/LaserScan. IPC adds its own checksum/framing.
// This is a typed ROS bridge payload, not a ROS-agnostic wire ABI.
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace robotcore_ros {
namespace scan_codec {

using Scan = sensor_msgs::msg::LaserScan;
constexpr std::uint32_t kMagic = 0x534c4352u;  // "RCLS" (little-endian)
constexpr std::uint32_t kVersion = 1u;
constexpr std::size_t kEnvelopeBytes = 12u;
constexpr std::size_t kMaxFrameBytes = 65536u;

inline void write_u32(std::uint8_t* ptr, std::uint32_t value) {
  for (int i = 0; i < 4; ++i)
    ptr[i] = static_cast<std::uint8_t>(value >> (8 * i));
}
inline std::uint32_t read_u32(const std::uint8_t* ptr) {
  std::uint32_t value = 0;
  for (int i = 0; i < 4; ++i)
    value |= static_cast<std::uint32_t>(ptr[i]) << (8 * i);
  return value;
}

inline std::vector<std::uint8_t> encode(const Scan& scan) {
  rclcpp::Serialization<Scan> codec;
  rclcpp::SerializedMessage serialized;
  codec.serialize_message(&scan, &serialized);
  const auto& bytes = serialized.get_rcl_serialized_message();
  if (bytes.buffer_length == 0 ||
      bytes.buffer_length > kMaxFrameBytes - kEnvelopeBytes)
    throw std::invalid_argument("LaserScan CDR frame exceeds bounded IPC envelope");

  std::vector<std::uint8_t> frame(kEnvelopeBytes + bytes.buffer_length);
  write_u32(frame.data(), kMagic);
  write_u32(frame.data() + 4, kVersion);
  write_u32(frame.data() + 8,
            static_cast<std::uint32_t>(bytes.buffer_length));
  std::memcpy(frame.data() + kEnvelopeBytes, bytes.buffer, bytes.buffer_length);
  return frame;
}

inline Scan decode(const std::vector<std::uint8_t>& frame) {
  if (frame.size() < kEnvelopeBytes || frame.size() > kMaxFrameBytes)
    throw std::invalid_argument("LaserScan frame has invalid total length");
  if (read_u32(frame.data()) != kMagic)
    throw std::invalid_argument("LaserScan envelope magic mismatch");
  if (read_u32(frame.data() + 4) != kVersion)
    throw std::invalid_argument("unsupported LaserScan envelope version");

  const auto length = static_cast<std::size_t>(read_u32(frame.data() + 8));
  if (length == 0 || length != frame.size() - kEnvelopeBytes)
    throw std::invalid_argument("LaserScan envelope CDR length mismatch");

  rclcpp::SerializedMessage serialized(length);
  auto& bytes = serialized.get_rcl_serialized_message();
  std::memcpy(bytes.buffer, frame.data() + kEnvelopeBytes, length);
  bytes.buffer_length = length;
  Scan scan;
  rclcpp::Serialization<Scan> codec;
  codec.deserialize_message(&serialized, &scan);
  return scan;
}

}  // namespace scan_codec
}  // namespace robotcore_ros
