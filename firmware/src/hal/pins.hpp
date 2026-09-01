#pragma once
// Pin map. SDK-free: values source from cfg:: (constants.hpp) and use plain
// `unsigned`, so this header no longer needs hardware/gpio.h.
#include "constants.hpp"

namespace hal {
inline constexpr unsigned kPickup = cfg::kPickup;
inline constexpr unsigned kTach = cfg::kTach;
inline constexpr unsigned kLed = cfg::kLed;
// Per-cylinder ignition-sense GPIO (cyl i -> kIgnPins[i]); baked DUT geometry.
inline constexpr std::size_t kCylCount = cfg::kCylCount;
inline constexpr const unsigned* kIgnPins = cfg::kIgnPins;
} // namespace hal
