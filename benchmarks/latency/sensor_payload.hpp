#pragma once

// Deterministic, *synthetic* robotics-shaped byte payloads for one shared
// benchmark workload. These are not sensor_msgs, sensor readings or a claim
// that a complete ROS 2 sensor-to-actuator pipeline has been tested.
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace robotcore_bench {
enum class PayloadProfile { uniform, laser360, gray_image_vga, depth16_vga };

inline PayloadProfile parse_profile(const std::string& value) {
  if (value == "uniform") return PayloadProfile::uniform;
  if (value == "laser360") return PayloadProfile::laser360;
  if (value == "gray_image_vga") return PayloadProfile::gray_image_vga;
  if (value == "depth16_vga") return PayloadProfile::depth16_vga;
  throw std::invalid_argument("unknown --payload-profile: " + value);
}
inline const char* profile_name(PayloadProfile p) {
  switch (p) {
    case PayloadProfile::uniform: return "uniform";
    case PayloadProfile::laser360: return "laser360";
    case PayloadProfile::gray_image_vga: return "gray_image_vga";
    case PayloadProfile::depth16_vga: return "depth16_vga";
  }
  throw std::invalid_argument("unsupported payload profile");
}
inline std::size_t profile_bytes(PayloadProfile p, std::size_t uniform_size) {
  switch (p) {
    case PayloadProfile::uniform: return uniform_size;
    case PayloadProfile::laser360: return 360u * 4u;
    case PayloadProfile::gray_image_vga: return 640u * 480u;
    case PayloadProfile::depth16_vga: return 640u * 480u * 2u;
  }
  throw std::invalid_argument("unsupported payload profile");
}
inline void put16(std::uint8_t* p, std::uint16_t n) {
  p[0] = static_cast<std::uint8_t>(n);
  p[1] = static_cast<std::uint8_t>(n >> 8);
}
inline void put32(std::uint8_t* p, std::uint32_t n) {
  for (int i = 0; i < 4; ++i)
    p[i] = static_cast<std::uint8_t>(n >> (8 * i));
}

inline std::vector<std::uint8_t> make_payload(PayloadProfile profile,
                                               std::size_t size,
                                               std::uint64_t sequence) {
  if (size != profile_bytes(profile, size))
    throw std::invalid_argument("payload size does not match sensor shape");
  std::vector<std::uint8_t> bytes(size);
  switch (profile) {
    case PayloadProfile::uniform:
      for (auto& value : bytes) value = 0xa5;
      break;
    case PayloadProfile::laser360:
      // 360 float32 ranges in meters (synthetic rotating range pattern).
      for (std::size_t i = 0; i < 360; ++i) {
        const std::uint64_t bucket = (i * 37u + (sequence % 10000u) * 11u) % 5000u;
        const float meters = 0.4f + static_cast<float>(bucket) / 1000.0f;
        std::uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(meters), "float32 required");
        std::memcpy(&bits, &meters, sizeof(bits));
        put32(bytes.data() + i * 4u, bits);
      }
      break;
    case PayloadProfile::gray_image_vga:
      // 640x480 single-channel 8-bit synthetic intensity image.
      for (std::size_t y = 0; y < 480; ++y)
        for (std::size_t x = 0; x < 640; ++x)
          bytes[y * 640u + x] =
              static_cast<std::uint8_t>((13u * x + 7u * y + sequence % 251u) & 0xffu);
      break;
    case PayloadProfile::depth16_vga:
      // 640x480 uint16 little-endian synthetic depth, millimeters.
      for (std::size_t y = 0; y < 480; ++y)
        for (std::size_t x = 0; x < 640; ++x) {
          const std::uint64_t mm = 500u + (3u * x + 5u * y +
                                             (sequence % 10000u) * 17u) % 3500u;
          put16(bytes.data() + (y * 640u + x) * 2u,
                static_cast<std::uint16_t>(mm));
        }
      break;
  }
  return bytes;
}
} // namespace robotcore_bench
