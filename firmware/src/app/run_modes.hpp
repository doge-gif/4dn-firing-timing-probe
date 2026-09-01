#pragma once
#include "core/config.hpp"

#include <cstdint>

namespace app {

// Pure schedule for the stepped sweep run mode: steps RPM from sweep_start_rpm
// to sweep_end_rpm (inclusive) by sweep_step_rpm. Effects (pattern reload,
// capture reads) are injected elsewhere; this only tracks the schedule state.
class SteppedSchedule {
public:
  explicit SteppedSchedule(const core::Config& c);

  std::uint32_t current_rpm() const;
  void advance_step();
  bool done() const;

  std::uint32_t revs_to_settle() const;
  std::uint32_t samples_wanted() const;

private:
  std::uint32_t sweep_start_rpm_ = 0;
  std::uint32_t sweep_end_rpm_ = 0;
  std::uint32_t sweep_step_rpm_ = 0;
  std::uint32_t sweep_settle_revs_ = 0;
  std::uint32_t sweep_samples_ = 0;

  std::uint32_t current_rpm_ = 0;
  bool done_ = false;
};

// Pure schedule for a ramp run mode: continuously integrates RPM by a fixed
// rate, clamped at the appropriate bound (sweep_end_rpm for ramp-up,
// sweep_start_rpm for ramp-down).
class RampSchedule {
public:
  RampSchedule(const core::Config& c, bool up);

  void tick(float dt_s);
  float current_rpm() const;
  bool done() const;

private:
  float rpm_ = 0.0f;
  float rate_ = 0.0f;
  float bound_ = 0.0f;
  bool up_ = false;
};

} // namespace app
