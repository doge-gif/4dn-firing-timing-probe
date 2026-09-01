#include "core/capture_decode.hpp"
#include "core/timebase.hpp"
#include "doctest/doctest.h"

#include <cstdint>

// Capture word format (see capture.pio): bit 31 = the pin level SAMPLED AT THE
// EDGE (1 = line now HIGH => this edge was rising; 0 = falling); bits [30:0] = a
// 31-bit free-running up-counter snapshot (62.5 MHz, wraps ~34 s). Polarity is
// therefore CARRIED PER EDGE, not inferred by alternation -- a dropped edge can
// no longer flip the polarity of every edge after it. The channel Timebase64 is
// constructed 31-bit (one bit ceded to the level).
static constexpr std::uint32_t kLvl = 1u << core::kCaptureLevelBit; // 0x8000'0000

// (a) Monotonic no-wrap: the 31-bit timestamp extends to the same high word; the
// returned 64-bit timestamp == the low-31 payload, independent of the level bit.
TEST_CASE("capture_decode: 31-bit timestamp, no wrap") {
  core::Timebase64 tb{core::kCaptureTsBits};
  bool level = false;

  CHECK(core::decode_capture(0x0000'0100u, tb, level) == 0x0000'0100ull);
  CHECK(level == false); // bit31 clear -> falling
  CHECK(core::decode_capture(kLvl | 0x0000'0200u, tb, level) == 0x0000'0200ull);
  CHECK(level == true); // bit31 set -> rising (and note ts still increased)
  CHECK(core::decode_capture(kLvl | 0x1234'5678u, tb, level) == 0x1234'5678ull);
  CHECK(core::decode_capture(0x7FFF'0000u, tb, level) == 0x7FFF'0000ull);
}

// (b) Wrap across 2^31: when the 31-bit payload steps backwards, the high word
// increments so the 64-bit timestamp keeps counting up. now() shifts by 31.
TEST_CASE("capture_decode: wrap across 2^31 increments high word") {
  core::Timebase64 tb{core::kCaptureTsBits};
  bool level = false;

  CHECK(core::decode_capture(0x7FFF'FF00u, tb, level) == 0x0000'0000'7FFF'FF00ull);
  // Next payload is smaller -> one 31-bit wrap: high=1, now=(1<<31)|0x100.
  CHECK(core::decode_capture(kLvl | 0x0000'0100u, tb, level) == 0x0000'0000'8000'0100ull);
  // Larger within the second span -> no further wrap. (ts 0x4000'0000, level hi)
  CHECK(core::decode_capture(kLvl | 0x4000'0000u, tb, level) == 0x0000'0000'C000'0000ull);
  // Wrap again -> high=2.
  CHECK(core::decode_capture(0x0000'0050u, tb, level) == 0x0000'0001'0000'0050ull);
}

// The elapsed delta across a wrap is exact (modular subtraction over 31 bits).
TEST_CASE("capture_decode: 64-bit delta across a 2^31 wrap is exact") {
  core::Timebase64 tb{core::kCaptureTsBits};
  bool level = false;
  std::uint64_t a = core::decode_capture(0x7FFF'FF00u, tb, level);
  std::uint64_t b = core::decode_capture(kLvl | 0x0000'0100u, tb, level); // 0x200 later
  CHECK(b - a == 0x200ull);
}

// (c) THE ROBUSTNESS PROPERTY: polarity comes from the captured bit, NOT from
// alternation. Two edges captured at the SAME level in a row (e.g. an edge was
// dropped by a full FIFO between them) must BOTH report that level -- alternation
// would have flipped the second, silently corrupting every edge afterwards.
TEST_CASE("capture_decode: level is per-edge, survives a dropped edge") {
  core::Timebase64 tb{core::kCaptureTsBits};
  bool level = false;

  core::decode_capture(kLvl | 0x0000'0064u, tb, level);
  CHECK(level == true); // rising
  core::decode_capture(kLvl | 0x0000'00C8u, tb, level);
  CHECK(level == true); // STILL rising (the intervening falling edge was dropped)
  core::decode_capture(0x0000'012Cu, tb, level);
  CHECK(level == false); // falling
  core::decode_capture(0x0000'0190u, tb, level);
  CHECK(level == false); // STILL falling (a rising edge was dropped)
  core::decode_capture(kLvl | 0x0000'01F4u, tb, level);
  CHECK(level == true); // rising
}
