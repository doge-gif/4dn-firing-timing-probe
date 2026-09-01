#pragma once
#include "core/types.hpp"

#include <array>
#include <cstdint>

namespace core {

struct EdgeRef {             // an emitted pickup edge in the shared timebase
  std::uint64_t t_ticks = 0; // 64-bit timestamp (ticks)
  float angle_deg = 0.0f;    // known wheel-frame angle
};

// Interpolate the crank angle of an event at time t_ticks between two bracketing edges.
float interp_angle(std::uint64_t t_ticks, const EdgeRef& before, const EdgeRef& after);

// °BTDC = (tdc_ref - spark_angle) mod wheel, result in [0, wheel).
float to_btdc(float spark_angle, float tdc_ref_deg, float wheel_deg);

struct DwellResult {
  bool has_dwell = false;
  std::uint64_t dwell_ticks = 0;
};

// One instance handles all channels; state is per channel index (< kMaxCylinders).
class DwellPairer {
public:
  DwellResult on_edge(std::uint8_t ch, bool rising, std::uint64_t t_ticks) {
    DwellResult r{};
    if (ch >= kMaxCylinders)
      return r;
    if (rising) {
      if (pending_[ch]) {
        r.has_dwell = true;
        r.dwell_ticks = t_ticks - fall_t_[ch];
        pending_[ch] = false;
      }
    } else {
      fall_t_[ch] = t_ticks;
      pending_[ch] = true; // record/replace charge start
    }
    return r;
  }

private:
  std::array<std::uint64_t, kMaxCylinders> fall_t_{};
  std::array<bool, kMaxCylinders> pending_{};
};

} // namespace core
