#include "core/config.hpp"

#include "core/ini.hpp"
#include "core/parse.hpp"

namespace core {
namespace {
// CONFIG.INI holds only the operator-tunable RUN params. The DUT/wheel geometry
// (teeth, wheel span, per-cylinder TDC + sense pins) is hardware-fixed and baked
// into constants.hpp -- NOT in CONFIG.INI -- so those keys are absent here.
constexpr std::string_view kKnownKeys[] = {
    "RUN_MODE",          "SWEEP_START_RPM",     "SWEEP_END_RPM",
    "SWEEP_STEP_RPM",    "SWEEP_SETTLE_REVS",   "SWEEP_SAMPLES",
    "RAMP_UP_RPM_PER_S", "RAMP_DOWN_RPM_PER_S", "HOLD_RPM"};
// Keys that MUST be present. HOLD_RPM is intentionally EXCLUDED (optional, defaults
// to cfg::kDefaultHoldRpm) so pre-HOLD on-flash CONFIG.INI files still load.
constexpr std::string_view kRequiredKeys[] = {
    "RUN_MODE",          "SWEEP_START_RPM", "SWEEP_END_RPM",     "SWEEP_STEP_RPM",
    "SWEEP_SETTLE_REVS", "SWEEP_SAMPLES",   "RAMP_UP_RPM_PER_S", "RAMP_DOWN_RPM_PER_S"};
bool iequal(std::string_view a, std::string_view b) {
  if (a.size() != b.size())
    return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    char ca = a[i], cb = b[i];
    if (ca >= 'A' && ca <= 'Z')
      ca += 32;
    if (cb >= 'A' && cb <= 'Z')
      cb += 32;
    if (ca != cb)
      return false;
  }
  return true;
}
bool known(std::string_view k) {
  for (auto kk : kKnownKeys)
    if (iequal(k, kk))
      return true;
  return false;
}
// RUN_MODE keyword -> RunMode. Returns false for any value other than the
// three recognized keywords (case-insensitive).
bool parse_run_mode(std::string_view v, RunMode& out) {
  if (iequal(v, "STEPPED")) {
    out = RunMode::Stepped;
    return true;
  }
  if (iequal(v, "RAMP_UP")) {
    out = RunMode::RampUp;
    return true;
  }
  if (iequal(v, "RAMP_DOWN")) {
    out = RunMode::RampDown;
    return true;
  }
  if (iequal(v, "HOLD")) {
    out = RunMode::Hold;
    return true;
  }
  return false;
}
LoadResult err(ConfigError e) {
  LoadResult r{};
  r.error = e;
  return r;
}
} // namespace

std::uint32_t rpm_max_hw(float min_interval_deg, float wheel_deg) {
  // The reference formula writes the denominator as literal 360; we use wheel_deg to generalize
  // to non-360 wheels (identical for the SRV250 default WHEEL_DEG=360).
  // RPM_MAX_HW = (min_interval_deg/wheel_deg) * 60 * f_pio / N_min
  float v = (min_interval_deg / wheel_deg) * 60.0f * kPioClockHz / kMinTicks;
  return (v < 0.0f) ? 0u : std::uint32_t(v);
}

LoadResult load_config(std::string_view ini_text) {
  IniResult ini = parse_ini(ini_text);
  if (!ini.ok)
    return err(ConfigError::IniFormat);

  // reject unknown keys
  for (std::size_t i = 0; i < ini.count; ++i)
    if (!known(ini.pairs[i].key))
      return err(ConfigError::UnknownKey);
  // require all known keys present
  for (auto k : kRequiredKeys)
    if (!ini.has(k))
      return err(ConfigError::MissingKey);

  Config c{};

  // Geometry is hardware-fixed (baked in constants.hpp), not read from CONFIG.INI.
  // Copy it into the runtime Config so downstream code (core1, rpm_max_hw) keeps
  // using c.teeth/c.wheel_deg/c.tdc_ref_deg unchanged.
  c.wheel_deg = cfg::kWheelDeg;
  c.tooth_count = std::uint8_t(cfg::kToothCount);
  for (std::size_t i = 0; i < cfg::kToothCount; ++i)
    c.teeth[i] = ToothSpan{cfg::kToothLeadingDeg[i], cfg::kToothTrailingDeg[i]};
  c.cyl_count = std::uint8_t(cfg::kCylCount);
  for (std::size_t i = 0; i < cfg::kCylCount; ++i)
    c.tdc_ref_deg[i] = cfg::kTdcRefDeg[i];

  // RUN_MODE
  if (!parse_run_mode(ini.value_of("RUN_MODE"), c.run_mode))
    return err(ConfigError::RunModeInvalid);

  // sweep scalars
  if (!parse_u32(ini.value_of("SWEEP_START_RPM"), c.sweep_start_rpm))
    return err(ConfigError::BadNumber);
  if (!parse_u32(ini.value_of("SWEEP_END_RPM"), c.sweep_end_rpm))
    return err(ConfigError::BadNumber);
  if (!parse_u32(ini.value_of("SWEEP_STEP_RPM"), c.sweep_step_rpm))
    return err(ConfigError::BadNumber);
  if (!parse_u32(ini.value_of("SWEEP_SETTLE_REVS"), c.sweep_settle_revs))
    return err(ConfigError::BadNumber);
  if (!parse_u32(ini.value_of("SWEEP_SAMPLES"), c.sweep_samples))
    return err(ConfigError::BadNumber);
  if (!parse_f32(ini.value_of("RAMP_UP_RPM_PER_S"), c.ramp_up_rpm_per_s))
    return err(ConfigError::BadNumber);
  if (!parse_f32(ini.value_of("RAMP_DOWN_RPM_PER_S"), c.ramp_down_rpm_per_s))
    return err(ConfigError::BadNumber);

  // HOLD_RPM is optional: absent -> default. A present-but-malformed value is an error.
  if (ini.has("HOLD_RPM")) {
    if (!parse_u32(ini.value_of("HOLD_RPM"), c.hold_rpm))
      return err(ConfigError::BadNumber);
  } else {
    c.hold_rpm = cfg::kDefaultHoldRpm;
  }

  // range checks
  if (c.sweep_step_rpm == 0)
    return err(ConfigError::StepRange);
  if (c.sweep_start_rpm > c.sweep_end_rpm)
    return err(ConfigError::RpmRange);
  if (c.sweep_settle_revs == 0 || c.sweep_samples == 0)
    return err(ConfigError::MiscRange);
  if (c.ramp_up_rpm_per_s <= 0.0f || c.ramp_down_rpm_per_s <= 0.0f)
    return err(ConfigError::MiscRange);
  if ((c.run_mode == RunMode::RampUp || c.run_mode == RunMode::RampDown) &&
      !(c.sweep_start_rpm < c.sweep_end_rpm))
    return err(ConfigError::RpmRange); // a ramp needs a span

  // narrowest emitted interval = min(tooth widths, gaps) across the wheel
  float min_iv = c.wheel_deg;
  for (std::size_t i = 0; i < c.tooth_count; ++i) {
    float width = c.teeth[i].trailing_deg - c.teeth[i].leading_deg;
    if (width < min_iv)
      min_iv = width;
    float next_lead = (i + 1 < c.tooth_count) ? c.teeth[i + 1].leading_deg
                                              : (c.teeth[0].leading_deg + c.wheel_deg);
    float gap = next_lead - c.teeth[i].trailing_deg;
    if (gap < min_iv)
      min_iv = gap;
  }
  if (c.run_mode == RunMode::Hold &&
      (c.hold_rpm == 0 || c.hold_rpm > rpm_max_hw(min_iv, c.wheel_deg)))
    return err(ConfigError::MiscRange); // HOLD_RPM outside 1..hardware ceiling
  if (c.sweep_end_rpm > rpm_max_hw(min_iv, c.wheel_deg))
    return err(ConfigError::RpmCeiling);

  LoadResult r{};
  r.ok = true;
  r.config = c;
  return r;
}

} // namespace core
