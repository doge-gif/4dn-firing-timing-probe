#pragma once
#include "constants.hpp"

#include <array>
#include <cstdint>

namespace core {

// Sourced from cfg:: (constants.hpp) so the single-source-of-truth values cannot
// drift; existing core::kMax* names stay valid for downstream code.
inline constexpr std::size_t kMaxTeeth = cfg::kMaxTeeth;         // wheel teeth per rev
inline constexpr std::size_t kMaxCylinders = cfg::kMaxCylinders; // ignition outputs
static_assert(kMaxTeeth <= 64, "tooth table must stay small");
static_assert(kMaxCylinders <= 8, "cylinder table must stay small");

enum class RunMode : std::uint8_t { Stepped, RampUp, RampDown, Hold };

struct ToothSpan {           // wheel-frame degrees
  float leading_deg = 0.0f;  // start angle
  float trailing_deg = 0.0f; // end angle (leading_deg < trailing_deg)
};

struct Config {
  // geometry
  std::array<ToothSpan, kMaxTeeth> teeth{};
  std::uint8_t tooth_count = 0;
  float wheel_deg = 360.0f;
  std::array<float, kMaxCylinders> tdc_ref_deg{}; // absolute wheel-frame angle per output
  std::uint8_t cyl_count = 0;
  // run mode selection
  RunMode run_mode = RunMode::Stepped; // C++ member init only; INI key is required
  // sweep (also bounds the ramp modes)
  std::uint32_t sweep_start_rpm = 0;
  std::uint32_t sweep_end_rpm = 0;
  std::uint32_t sweep_step_rpm = 0;
  std::uint32_t sweep_settle_revs = 0; // revs to settle at each step before sampling
  std::uint32_t sweep_samples = 0;     // spark samples reduced per step
  float ramp_up_rpm_per_s = 0.0f;      // rpm/s
  float ramp_down_rpm_per_s = 0.0f;    // rpm/s
  std::uint32_t hold_rpm = 0;          // constant RPM for RunMode::Hold (optional INI key)
};

} // namespace core
