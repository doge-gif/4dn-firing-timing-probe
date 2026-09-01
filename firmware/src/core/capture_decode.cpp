#include "core/capture_decode.hpp"

namespace core {

std::uint64_t decode_capture(std::uint32_t raw, Timebase64& tb, bool& level) {
  // Polarity is carried PER EDGE in bit 31 (sampled from the pin in PIO), not
  // inferred by alternation: true = line HIGH after this edge (rising), false =
  // LOW (falling). Reading it directly means a dropped edge cannot invert every
  // edge after it (the old alternation failure mode).
  level = ((raw >> kCaptureLevelBit) & 1u) != 0u;

  // Extend the 31-bit counter payload to 64-bit. `tb` is 31-bit, so update()
  // masks off the level bit and bumps the high word on a 2^31 wrap; now() returns
  // the full 64-bit timestamp for THIS edge.
  tb.update(raw);

  return tb.now();
}

} // namespace core
