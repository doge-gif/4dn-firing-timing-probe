#include "app/run_modes.hpp"
#include "core/config.hpp"
#include "doctest/doctest.h"

namespace {
core::Config run_cfg() {
  auto lr = core::load_config(
      "RUN_MODE=STEPPED\n"
      "SWEEP_START_RPM=1000\nSWEEP_END_RPM=1200\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=4\n"
      "SWEEP_SAMPLES=8\nRAMP_UP_RPM_PER_S=500\nRAMP_DOWN_RPM_PER_S=500\n");
  REQUIRE(lr.ok);
  return lr.config;
}
} // namespace

TEST_CASE("stepped: visits SWEEP_START_RPM..SWEEP_END_RPM inclusive by SWEEP_STEP_RPM") {
  app::SteppedSchedule s(run_cfg());
  CHECK(s.current_rpm() == 1000);
  s.advance_step();
  CHECK(s.current_rpm() == 1100);
  s.advance_step();
  CHECK(s.current_rpm() == 1200);
  CHECK_FALSE(s.done());
  s.advance_step();
  CHECK(s.done()); // past SWEEP_END_RPM
}

TEST_CASE("stepped: settle then sample counts") {
  app::SteppedSchedule s(run_cfg());
  // needs SWEEP_SETTLE_REVS settle revs before it accepts samples
  CHECK(s.revs_to_settle() == 4);
  CHECK(s.samples_wanted() == 8);
}

TEST_CASE("ramp-up: rpm advances by rate*dt and finishes at SWEEP_END_RPM") {
  app::RampSchedule r(run_cfg(), /*up=*/true);
  float rpm0 = r.current_rpm();
  r.tick(0.1f); // 0.1 s * 500 rpm/s = +50
  CHECK(r.current_rpm() == doctest::Approx(rpm0 + 50.0f));
  // run enough to exceed SWEEP_END_RPM
  for (int i = 0; i < 100 && !r.done(); ++i)
    r.tick(0.1f);
  CHECK(r.done());
  CHECK(r.current_rpm() <= doctest::Approx(1200.0f).epsilon(0.01));
}
