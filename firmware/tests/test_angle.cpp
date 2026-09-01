#include "core/angle.hpp"
#include "doctest/doctest.h"

TEST_CASE("angle: linear interpolation between two bracketing edges") {
  core::EdgeRef before{1000ull, 0.0f};
  core::EdgeRef after{2000ull, 60.0f};
  // spark halfway in time -> halfway in angle
  CHECK(core::interp_angle(1500ull, before, after) == doctest::Approx(30.0f));
  // quarter
  CHECK(core::interp_angle(1250ull, before, after) == doctest::Approx(15.0f));
}

TEST_CASE("angle: interpolation exact even under non-constant rate (uses real timestamps)") {
  // Bracket edges are the ACTUAL emitted timestamps, so ratio is time-based.
  core::EdgeRef before{0ull, 120.0f};
  core::EdgeRef after{900ull, 180.0f};
  CHECK(core::interp_angle(300ull, before, after) == doctest::Approx(140.0f));
}

TEST_CASE("angle: btdc wraps into [0,wheel)") {
  // tdc at 5deg, spark at 350deg -> (5-350) mod 360 = 15 deg BTDC
  CHECK(core::to_btdc(350.0f, 5.0f, 360.0f) == doctest::Approx(15.0f));
  // tdc 5, spark 0 -> 5 BTDC
  CHECK(core::to_btdc(0.0f, 5.0f, 360.0f) == doctest::Approx(5.0f));
}

TEST_CASE("dwell: pairs each rising with the most recent falling, per channel") {
  core::DwellPairer p;
  // channel 0: fall@100, rise@250 -> dwell 150
  CHECK_FALSE(p.on_edge(0, false, 100ull).has_dwell); // charge start, no dwell yet
  core::DwellResult r = p.on_edge(0, true, 250ull);   // spark
  REQUIRE(r.has_dwell);
  CHECK(r.dwell_ticks == 150ull);
}

TEST_CASE("dwell: falling may precede the tooth / cross channels independently") {
  core::DwellPairer p;
  p.on_edge(0, false, 1000ull);                             // ch0 charge
  p.on_edge(1, false, 1050ull);                             // ch1 charge (independent)
  CHECK(p.on_edge(1, true, 1200ull).dwell_ticks == 150ull); // ch1 spark
  CHECK(p.on_edge(0, true, 1400ull).dwell_ticks == 400ull); // ch0 spark
}

TEST_CASE("dwell: rising with no pending falling yields no dwell") {
  core::DwellPairer p;
  CHECK_FALSE(p.on_edge(0, true, 500ull).has_dwell);
}

TEST_CASE("dwell: second falling before a rising replaces the pending one") {
  core::DwellPairer p;
  p.on_edge(0, false, 100ull);
  p.on_edge(0, false, 200ull); // replaces
  CHECK(p.on_edge(0, true, 500ull).dwell_ticks == 300ull);
}
