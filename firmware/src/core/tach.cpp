#include "core/tach.hpp"

namespace core {

float tach_latency_us(std::uint64_t tach_edge_ticks, std::uint64_t tooth0_edge_ticks) {
  if (tach_edge_ticks < tooth0_edge_ticks) {
    return 0.0f; // tach before tooth-0 -> unmatched / no signal, guard
  }
  return float(tach_edge_ticks - tooth0_edge_ticks) / cfg::kCaptureCountsPerUs;
}

float tach_rpm_from_cycle_ticks(std::uint64_t cycle_ticks) {
  if (cycle_ticks == 0) {
    return 0.0f; // no cycle measured (bench / no ignitor), guard
  }
  // Capture counts per second (== 62.5e6): the capture counter rate.
  const float counts_per_sec = cfg::kPioTicksPerSec / float(cfg::kCaptureCyclesPerCount);
  return 60.0f * counts_per_sec / float(cycle_ticks) / float(cfg::kTachPulsesPerRev);
}

} // namespace core
