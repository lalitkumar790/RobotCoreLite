#include "sensor_payload.hpp"
#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
using namespace robotcore_bench;

static bool rejects_size(PayloadProfile p, std::size_t size) {
  try { (void)make_payload(p, size, 1); } catch (const std::invalid_argument&) { return true; }
  return false;
}
int main() {
  assert(parse_profile("uniform") == PayloadProfile::uniform);
  assert(parse_profile("laser360") == PayloadProfile::laser360);
  assert(parse_profile("gray_image_vga") == PayloadProfile::gray_image_vga);
  assert(parse_profile("depth16_vga") == PayloadProfile::depth16_vga);
  const auto uniform = make_payload(PayloadProfile::uniform, 32, 10);
  assert(uniform.size() == 32);
  for (std::uint8_t x : uniform) assert(x == 0xa5);

  const auto scan1 = make_payload(PayloadProfile::laser360, 1440, 1);
  const auto scan2 = make_payload(PayloadProfile::laser360, 1440, 2);
  assert(scan1.size() == 1440 && scan1 != scan2);
  assert(scan1 == make_payload(PayloadProfile::laser360, 1440, 1));
  assert(rejects_size(PayloadProfile::laser360, 1439));

  const auto gray1 = make_payload(PayloadProfile::gray_image_vga, 307200, 1);
  const auto gray2 = make_payload(PayloadProfile::gray_image_vga, 307200, 2);
  assert(gray1.size() == 640u * 480u && gray1 != gray2);
  assert(gray1[0] == static_cast<std::uint8_t>(1));
  assert(gray1[1] == static_cast<std::uint8_t>(14));
  assert(gray1[640] == static_cast<std::uint8_t>(8));
  assert(gray1 == make_payload(PayloadProfile::gray_image_vga, 307200, 1));
  assert(rejects_size(PayloadProfile::gray_image_vga, 1024));

  const auto depth1 = make_payload(PayloadProfile::depth16_vga, 614400, 1);
  const auto depth2 = make_payload(PayloadProfile::depth16_vga, 614400, 2);
  assert(depth1.size() == 640u * 480u * 2u && depth1 != depth2);
  // 500 + 17 mm in little endian = 0x0205.
  assert(depth1[0] == 0x05 && depth1[1] == 0x02);
  assert(depth1 == make_payload(PayloadProfile::depth16_vga, 614400, 1));
  assert(rejects_size(PayloadProfile::depth16_vga, 128));
  assert(std::string(profile_name(PayloadProfile::laser360)) == "laser360");
  bool invalid_rejected = false;
  try { (void)parse_profile("invented_sensor"); }
  catch (const std::invalid_argument&) { invalid_rejected = true; }
  assert(invalid_rejected);
}
