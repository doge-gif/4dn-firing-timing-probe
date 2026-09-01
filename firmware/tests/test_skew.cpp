#include "core/skew.hpp"
#include "doctest/doctest.h"

TEST_CASE("skew: first edge is unchanged") {
  CHECK(core::skew_corrected_ticks(1000u, 0u) == 1000u);
}

TEST_CASE("skew: correction is edge-count proportional and rounded") {
  // k = 2.75. round(2.75*1)=3, round(2.75*2)=6 (5.5->6), round(2.75*4)=11.
  CHECK(core::skew_corrected_ticks(0u, 1u) == 3u);
  CHECK(core::skew_corrected_ticks(0u, 2u) == 6u);
  CHECK(core::skew_corrected_ticks(0u, 4u) == 11u);
}

TEST_CASE("skew: monotonic non-decreasing in edge index") {
  std::uint64_t prev = 0;
  for (std::uint64_t i = 0; i < 5000; ++i) {
    std::uint64_t v = core::skew_corrected_ticks(100000u, i);
    CHECK(v >= prev);
    prev = v;
  }
}
