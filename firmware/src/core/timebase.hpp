#pragma once
#include <cstdint>

namespace core {

// Modular subtraction: exact for any single wrap of a 32-bit counter.
constexpr std::uint32_t elapsed32(std::uint32_t now, std::uint32_t prev) {
  return now - prev; // unsigned wraparound is well-defined
}

// Software 64-bit extension of a free-running N-bit counter, N in
// [1,32]. Default N=32 (a full 32-bit source). The capture path uses N=31 because
// the pushed word cedes bit 31 to the per-edge pin level (see capture.pio /
// core::kCaptureTsBits); a 31-bit counter at 62.5 MHz wraps every ~34.36 s.
// Precondition: update() must be called at least once within each wrap period.
// The capture ring is drained every core1 loop iteration (~microseconds), far
// tighter than the ~34 s wrap period (31-bit @ 62.5 MHz), so wrap detection is
// provably exact.
// NOT thread-safe: now() reads high_ and prev_ without a barrier — if update() runs
// on another core, the caller must synchronize.
class Timebase64 {
public:
  Timebase64() = default;
  // `bits` = width of the source counter (its wrap modulus is 2^bits). The high
  // word is shifted by `bits` in now(), and update() masks the raw to `bits` so a
  // level bit (or any high padding) in the pushed word never perturbs wrap logic.
  explicit Timebase64(unsigned bits)
      : mask_(bits >= 32 ? 0xFFFF'FFFFu : ((1u << bits) - 1u)), bits_(bits) {}

  void update(std::uint32_t raw) {
    raw &= mask_;
    if (has_prev_ && raw < prev_)
      ++high_; // low field went backwards => wrapped
    prev_ = raw;
    has_prev_ = true;
  }
  std::uint64_t now() const { return (std::uint64_t(high_) << bits_) | prev_; }

private:
  std::uint32_t mask_ = 0xFFFF'FFFFu;
  unsigned bits_ = 32;
  std::uint32_t prev_ = 0;
  std::uint32_t high_ = 0;
  bool has_prev_ = false;
};

} // namespace core
