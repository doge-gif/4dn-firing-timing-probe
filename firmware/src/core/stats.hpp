#pragma once
#include <cstddef>
#include <cstdint>

namespace core {

class Welford {
public:
  void add(float x) {
    ++n_;
    float d = x - mean_;
    mean_ += d / float(n_);
    m2_ += d * (x - mean_);
  }
  std::uint32_t count() const { return n_; }
  float mean() const { return (n_ == 0) ? 0.0f : mean_; }
  float stddev_sample() const { return (n_ < 2) ? 0.0f : sqrtf_(m2_ / float(n_ - 1)); }

private:
  static float sqrtf_(float v); // wraps std::sqrt in the .cpp to keep header light
  std::uint32_t n_ = 0;
  float mean_ = 0.0f;
  float m2_ = 0.0f;
};

struct MedianResult {
  float median = 0.0f;
  float min = 0.0f;
  float max = 0.0f;
};

// Computes median/min/max. Reorders `buf[0..n)` in place (caller owns the buffer).
MedianResult median_minmax(float* buf, std::size_t n);

} // namespace core
