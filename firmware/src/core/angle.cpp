#include "core/angle.hpp"

namespace core {

float interp_angle(std::uint64_t t_ticks, const EdgeRef& before, const EdgeRef& after) {
  if (after.t_ticks == before.t_ticks)
    return before.angle_deg;
  const double frac = double(t_ticks - before.t_ticks) / double(after.t_ticks - before.t_ticks);
  return float(before.angle_deg + frac * (after.angle_deg - before.angle_deg));
}

// to_btdc's three params are all wheel-frame degrees. The proper fix is a strong Deg
// units type threaded through the angle math (deferred as a larger change); for now
// the callers pass named values (interp result, cfg.tdc_ref_deg[cyl], cfg.wheel_deg).
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
float to_btdc(float spark_angle, float tdc_ref_deg, float wheel_deg) {
  float d = tdc_ref_deg - spark_angle;
  while (d < 0.0f)
    d += wheel_deg;
  while (d >= wheel_deg)
    d -= wheel_deg;
  return d;
}

} // namespace core
