#pragma once
#include <cstdint>
#include <stdexcept>

namespace robotcore_ros {

// Used by the synthetic LaserScan integration tests only. The first accepted
// frame must be #1 and every subsequent frame must be contiguous. Reject
// missing, duplicate, reordered, zero and out-of-range sequences.
class StrictScanSequence {
public:
  explicit StrictScanSequence(std::uint64_t expected) : expected_(expected) {
    if (expected_ == 0) throw std::invalid_argument("expected count must be positive");
  }
  bool accept(std::uint64_t sequence) noexcept {
    if (sequence == 0 || sequence > expected_ || sequence != last_ + 1)
      return false;
    last_ = sequence;
    return true;
  }
  std::uint64_t accepted() const noexcept { return last_; }
  bool complete() const noexcept { return last_ == expected_; }
private:
  std::uint64_t expected_;
  std::uint64_t last_{0};
};
}  // namespace robotcore_ros
