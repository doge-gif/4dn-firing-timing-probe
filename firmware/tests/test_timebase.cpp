#include "core/timebase.hpp"
#include "doctest/doctest.h"

#include <cstdint>

TEST_CASE("timebase: elapsed32 across a wrap") {
  // prev near top, now just past wrap.
  std::uint32_t prev = 0xFFFF'FF00u;
  std::uint32_t now = 0x0000'0100u; // 0x200 ticks later, modulo 2^32
  CHECK(core::elapsed32(now, prev) == 0x200u);
}

TEST_CASE("timebase: elapsed32 no-wrap") { CHECK(core::elapsed32(1000u, 400u) == 600u); }

TEST_CASE("timebase64: extends across multiple wraps when polled") {
  core::Timebase64 tb;
  tb.update(0x0000'0000u);
  CHECK(tb.now() == 0x0000'0000ull);
  tb.update(0xF000'0000u);
  CHECK(tb.now() == 0xF000'0000ull);
  tb.update(0x1000'0000u); // wrapped once (0xF..->0x1..)
  CHECK(tb.now() == 0x1'1000'0000ull);
  tb.update(0x2000'0000u); // no wrap
  CHECK(tb.now() == 0x1'2000'0000ull);
  tb.update(0x1000'0000u); // wrapped again
  CHECK(tb.now() == 0x2'1000'0000ull);
}
