#include "core/stats.hpp"

#include <algorithm>
#include <cmath>

namespace core {

float Welford::sqrtf_(float v) { return std::sqrt(v); }

MedianResult median_minmax(float* buf, std::size_t n) {
  MedianResult r{};
  if (n == 0)
    return r;
  r.min = buf[0];
  r.max = buf[0];
  for (std::size_t i = 1; i < n; ++i) {
    r.min = std::min(r.min, buf[i]);
    r.max = std::max(r.max, buf[i]);
  }
  std::sort(buf, buf + n); // fixed-capacity buffer; no heap
  r.median = (n & 1) ? buf[n / 2] : 0.5f * (buf[n / 2 - 1] + buf[n / 2]);
  return r;
}

} // namespace core
