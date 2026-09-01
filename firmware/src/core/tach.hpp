#pragma once
// tach.hpp -- pure reduction math for the tachometer reference-diagnostic
// channel (system input-path latency + sync-lock RPM). SDK-free and
// host-testable: only <cstdint> + constants.hpp. These are REFERENCE values
// only -- never subtracted from BTDC.
//
// The DUT ignitor outputs a tach square wave: one rising edge per crank
// revolution, aligned to tooth-0's rising edge (backed by running-ignitor scope
// captures -- see the scope_shots/ directory of the denso-TNDF17-reverse-engineer
// repo, cited at cfg::kTachPulsesPerRev in constants.hpp). On the bench there is no
// tach signal (it exists only with the real ignitor), so this math is
// host-unit-tested and additionally exercised on hardware when a real ignitor is
// connected.
//
// Units are capture-COUNTS (ticks), the same 62.5 MHz unit as CaptureEvent.ticks
// (kPioTicksPerSec / kCaptureCyclesPerCount).

#include "constants.hpp"
#include "core/stats.hpp"

#include <cstdint>

namespace core {

// Composite input-path latency: (tach_edge - our_emitted_tooth0_edge), in us.
// Clean-edge / bench-only REFERENCE value; never subtracted from BTDC. Guards to
// 0.0f if the tach edge precedes tooth-0 (unmatched / no signal).
float tach_latency_us(std::uint64_t tach_edge_ticks, std::uint64_t tooth0_edge_ticks);

// Sync-lock RPM derived from one tach cycle period (counts between consecutive
// tach reference edges). Confirms the ignitor locked onto our synthesized
// pattern. Guards to 0.0f for a zero-length cycle (no signal).
float tach_rpm_from_cycle_ticks(std::uint64_t cycle_ticks);

// Per-step accumulator for the tach reference diagnostics (sync-lock RPM + input
// latency). Owns ONLY the tach-specific state; the tooth-0 timestamp is passed
// in because it is shared with the pickup/ramp rev-period logic.
//
// reset() MUST be called at the start of every RPM step. Without it, the cycle
// and latency Welfords accumulate across the WHOLE sweep, so rpm() reports a
// running (~harmonic) mean of every prior step instead of the current one, and
// latency_us_mean() is likewise sweep-cumulative. (This is the exact defect the
// regression tests in test_tach.cpp guard against.)
struct TachDiag {
  // Clear per-step state. Drops the (soon-to-straddle) last-edge reference too,
  // so the first interval measured after a step boundary is not counted.
  void reset() {
    cycle_ticks_ = Welford{};
    latency_us_ = Welford{};
    have_last_ = false;
  }

  // Reduce one tach RISING reference edge at capture-count `tach_ticks`.
  // `have_tooth0`/`tooth0_ticks` give the most recent emitted tooth-0 rising edge
  // for the latency sample; pass have_tooth0=false to skip latency (e.g. before
  // the first tooth-0 of a run).
  void on_rising(std::uint64_t tach_ticks, bool have_tooth0, std::uint64_t tooth0_ticks) {
    if (have_last_) {
      cycle_ticks_.add(float(tach_ticks - last_edge_));
    }
    last_edge_ = tach_ticks;
    have_last_ = true;
    if (have_tooth0 && tach_ticks >= tooth0_ticks) {
      latency_us_.add(tach_latency_us(tach_ticks, tooth0_ticks));
    }
  }

  float rpm() const { return tach_rpm_from_cycle_ticks(std::uint64_t(cycle_ticks_.mean())); }
  float latency_us_mean() const { return latency_us_.mean(); }

private:
  Welford cycle_ticks_{};       // interval between consecutive tach rising edges
  Welford latency_us_{};        // composite input-path latency samples (us)
  std::uint64_t last_edge_ = 0; // previous tach rising edge (capture counts)
  bool have_last_ = false;
};

} // namespace core
