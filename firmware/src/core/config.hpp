#pragma once
#include "core/types.hpp"

#include <string_view>

namespace core {

enum class ConfigError : std::uint8_t {
  None,
  IniFormat,
  UnknownKey,
  MissingKey,
  BadNumber,
  ToothSpan,
  ToothOrder,
  ToothRange,
  TooManyTeeth,
  CylCount,
  TdcRange,
  RpmRange,
  RpmCeiling,
  StepRange,
  MiscRange,
  RunModeInvalid
};

struct LoadResult {
  bool ok = false;
  ConfigError error = ConfigError::None;
  Config config{};
};

// Hardware ceiling inputs. Overridable for tests if needed.
inline constexpr float kPioClockHz = cfg::kPioTicksPerSec;
inline constexpr float kMinTicks = cfg::kRpmCeilMinTicks;

// Parse + validate INI text into a Config. Text is borrowed during the call.
LoadResult load_config(std::string_view ini_text);

// Exposed for direct testing: ceiling in rpm for a given narrowest interval (deg).
std::uint32_t rpm_max_hw(float min_interval_deg, float wheel_deg);

} // namespace core
