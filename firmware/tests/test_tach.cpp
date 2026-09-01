#include "core/tach.hpp"
#include "doctest/doctest.h"

// Tach reference-diagnostic reduction math (host-tested; the capture wiring is
// exercised on hardware only with a real ignitor connected). ticks are
// capture-counts @ 62.5 counts/us.

TEST_CASE("tach: system latency in microseconds from tick delta") {
  // tooth0 emitted at 1000, tach rising edge 50000 counts later.
  // 50000 / 62.5 = 800.0 us.
  CHECK(core::tach_latency_us(1000 + 50000, 1000) == doctest::Approx(800.0f));
}

TEST_CASE("tach: zero-delta latency is zero") {
  CHECK(core::tach_latency_us(1234, 1234) == doctest::Approx(0.0f));
}

TEST_CASE("tach: negative (tach before tooth0) latency guards to zero") {
  CHECK(core::tach_latency_us(999, 1000) == doctest::Approx(0.0f));
}

TEST_CASE("tach: rpm from cycle ticks") {
  // 62.5e6 capture counts = 1 s of capture clock -> 1 rev/s -> 60 rpm.
  CHECK(core::tach_rpm_from_cycle_ticks(62'500'000) == doctest::Approx(60.0f));
  // 1.25e6 counts -> 62.5e6/1.25e6 = 50 rev/s -> 3000 rpm.
  CHECK(core::tach_rpm_from_cycle_ticks(1'250'000) == doctest::Approx(3000.0f));
}

TEST_CASE("tach: zero cycle guards to zero rpm") {
  CHECK(core::tach_rpm_from_cycle_ticks(0) == doctest::Approx(0.0f));
}

// TachDiag accumulator. cycle_ticks for R rpm = 60 * 62.5e6 / R:
//   1000 rpm -> 3'750'000,  3000 rpm -> 1'250'000.
namespace {
constexpr std::uint64_t kP1000 = 3'750'000;
constexpr std::uint64_t kP3000 = 1'250'000;
// Feed `revs` rising edges spaced `period` apart starting at *tk (no latency).
void feed(core::TachDiag& t, std::uint64_t& tk, std::uint64_t period, int revs) {
  for (int i = 0; i < revs; ++i) {
    tk += period;
    t.on_rising(tk, false, 0);
  }
}
} // namespace

TEST_CASE("tach: TachDiag reports the fed rpm") {
  core::TachDiag t;
  std::uint64_t tk = 0;
  t.on_rising(tk, false, 0); // first edge: baseline, no interval
  feed(t, tk, kP1000, 8);
  CHECK(t.rpm() == doctest::Approx(1000.0f));
}

// REGRESSION: begin_step() must reset the accumulator each step. Without the
// reset, tach_rpm accumulated across the whole sweep and reported the running
// (~harmonic) mean of every prior step -- e.g. 1000 rpm read as ~336 in the
// field data. This test pins the current-step behaviour and the buggy one.
TEST_CASE("tach: rpm tracks the current step, not the cumulative sweep") {
  core::TachDiag t;
  std::uint64_t tk = 0;

  // Step 1: 8 revs at 1000 rpm.
  t.reset();
  t.on_rising(tk, false, 0);
  feed(t, tk, kP1000, 8);
  CHECK(t.rpm() == doctest::Approx(1000.0f));

  // Bug shape: continue into step 2 (3000 rpm) WITHOUT reset. Equal interval
  // counts -> mean period (P1+P2)/2 -> exactly 1500 rpm, nowhere near 3000.
  core::TachDiag no_reset = t; // snapshot the sweep-cumulative state
  std::uint64_t tk2 = tk;
  feed(no_reset, tk2, kP3000, 8);
  CHECK(no_reset.rpm() == doctest::Approx(1500.0f));

  // Fixed: reset at the step boundary -> tracks the current (3000 rpm) step.
  t.reset();
  t.on_rising(tk, false, 0);
  feed(t, tk, kP3000, 8);
  CHECK(t.rpm() == doctest::Approx(3000.0f));
}

TEST_CASE("tach: reset drops the interval straddling a step boundary") {
  core::TachDiag t;
  std::uint64_t tk = 0;
  t.on_rising(tk, false, 0);
  feed(t, tk, kP3000, 2);
  CHECK(t.rpm() == doctest::Approx(3000.0f));

  // After reset, the first edge lands far later (post-settle). It must NOT be
  // counted as one enormous interval from the pre-reset edge.
  t.reset();
  std::uint64_t far = tk + 100'000'000;
  t.on_rising(far, false, 0); // no interval: have_last_ was cleared
  feed(t, far, kP3000, 3);
  CHECK(t.rpm() == doctest::Approx(3000.0f));
}

TEST_CASE("tach: reset clears the latency accumulator") {
  core::TachDiag t;
  // tooth-0 at 1000, tach edge 50000 counts later -> 50000 / 62.5 = 800 us.
  t.on_rising(1000 + 50000, /*have_tooth0=*/true, /*tooth0_ticks=*/1000);
  CHECK(t.latency_us_mean() == doctest::Approx(800.0f));
  t.reset();
  CHECK(t.latency_us_mean() == doctest::Approx(0.0f));
}
