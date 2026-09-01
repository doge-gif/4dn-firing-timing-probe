#include "app/run_modes.hpp"

namespace app {

SteppedSchedule::SteppedSchedule(const core::Config& c)
    : sweep_start_rpm_(c.sweep_start_rpm), sweep_end_rpm_(c.sweep_end_rpm),
      sweep_step_rpm_(c.sweep_step_rpm), sweep_settle_revs_(c.sweep_settle_revs),
      sweep_samples_(c.sweep_samples), current_rpm_(c.sweep_start_rpm) {}

std::uint32_t SteppedSchedule::current_rpm() const { return current_rpm_; }

void SteppedSchedule::advance_step() {
  current_rpm_ += sweep_step_rpm_;
  if (current_rpm_ > sweep_end_rpm_)
    done_ = true;
}

bool SteppedSchedule::done() const { return done_; }

std::uint32_t SteppedSchedule::revs_to_settle() const { return sweep_settle_revs_; }

std::uint32_t SteppedSchedule::samples_wanted() const { return sweep_samples_; }

RampSchedule::RampSchedule(const core::Config& c, bool up) : up_(up) {
  if (up_) {
    rpm_ = static_cast<float>(c.sweep_start_rpm);
    rate_ = c.ramp_up_rpm_per_s;
    bound_ = static_cast<float>(c.sweep_end_rpm);
  } else {
    rpm_ = static_cast<float>(c.sweep_end_rpm);
    rate_ = -c.ramp_down_rpm_per_s;
    bound_ = static_cast<float>(c.sweep_start_rpm);
  }
}

void RampSchedule::tick(float dt_s) {
  rpm_ += rate_ * dt_s;
  if (up_) {
    if (rpm_ >= bound_)
      rpm_ = bound_;
  } else {
    if (rpm_ <= bound_)
      rpm_ = bound_;
  }
}

float RampSchedule::current_rpm() const { return rpm_; }

bool RampSchedule::done() const { return up_ ? (rpm_ >= bound_) : (rpm_ <= bound_); }

} // namespace app
