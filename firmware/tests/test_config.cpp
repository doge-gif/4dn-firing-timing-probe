#include "constants.hpp"
#include "core/config.hpp"
#include "doctest/doctest.h"

// NOTE: DUT/wheel geometry (teeth, WHEEL_DEG, TDC, sense pins) is hardware-fixed
// and baked into constants.hpp -- it is NOT read from CONFIG.INI anymore. So
// CONFIG.INI carries only RUN params, and the geometry fields of Config are
// populated from cfg:: constants (verified in the first test below).

TEST_CASE("config: parses the baked default; geometry comes from constants") {
  core::LoadResult lr = core::load_config(cfg::kDefaultConfigIni);
  REQUIRE(lr.ok);
  const core::Config& c = lr.config;
  // Geometry is sourced from constants.hpp, not the INI:
  CHECK(c.tooth_count == 4);
  CHECK(c.teeth[0].leading_deg == doctest::Approx(0.0f));
  CHECK(c.teeth[3].trailing_deg == doctest::Approx(190.0f));
  CHECK(c.cyl_count == 2);
  CHECK(c.tdc_ref_deg[0] == doctest::Approx(cfg::kTdcRefDeg[0]));
  CHECK(c.tdc_ref_deg[1] == doctest::Approx(cfg::kTdcRefDeg[1]));
  // Run params come from the INI:
  CHECK(c.sweep_end_rpm == 10000u);
  CHECK(c.run_mode == core::RunMode::Stepped);
}

TEST_CASE("config: missing a required run key rejected") {
  // Valid keys but RUN_MODE (a required key) omitted.
  core::LoadResult lr = core::load_config(
      "SWEEP_START_RPM=100\nSWEEP_END_RPM=1000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=1\n"
      "SWEEP_SAMPLES=1\nRAMP_UP_RPM_PER_S=1\nRAMP_DOWN_RPM_PER_S=1\n");
  CHECK_FALSE(lr.ok);
  CHECK(lr.error == core::ConfigError::MissingKey);
}

TEST_CASE("config: a geometry key in CONFIG.INI is now an unknown key") {
  // Geometry moved to firmware, so WHEEL_DEG (etc.) is no longer accepted.
  core::LoadResult lr = core::load_config(
      "RUN_MODE=STEPPED\nWHEEL_DEG=360\n"
      "SWEEP_START_RPM=100\nSWEEP_END_RPM=1000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=1\n"
      "SWEEP_SAMPLES=1\nRAMP_UP_RPM_PER_S=1\nRAMP_DOWN_RPM_PER_S=1\n");
  CHECK_FALSE(lr.ok);
  CHECK(lr.error == core::ConfigError::UnknownKey);
}

TEST_CASE("config: RUN_MODE=RAMP_UP parses to RunMode::RampUp") {
  core::LoadResult lr = core::load_config(
      "RUN_MODE=RAMP_UP\n"
      "SWEEP_START_RPM=100\nSWEEP_END_RPM=1000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=1\n"
      "SWEEP_SAMPLES=1\nRAMP_UP_RPM_PER_S=1\nRAMP_DOWN_RPM_PER_S=1\n");
  REQUIRE(lr.ok);
  CHECK(lr.config.run_mode == core::RunMode::RampUp);
}

TEST_CASE("config: RUN_MODE=BOGUS rejected") {
  core::LoadResult lr = core::load_config(
      "RUN_MODE=BOGUS\n"
      "SWEEP_START_RPM=100\nSWEEP_END_RPM=1000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=1\n"
      "SWEEP_SAMPLES=1\nRAMP_UP_RPM_PER_S=1\nRAMP_DOWN_RPM_PER_S=1\n");
  CHECK_FALSE(lr.ok);
  CHECK(lr.error == core::ConfigError::RunModeInvalid);
}

TEST_CASE("config: ramp mode requires sweep_start_rpm < sweep_end_rpm") {
  core::LoadResult lr = core::load_config(
      "RUN_MODE=RAMP_UP\n"
      "SWEEP_START_RPM=1000\nSWEEP_END_RPM=1000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=1\n"
      "SWEEP_SAMPLES=1\nRAMP_UP_RPM_PER_S=1\nRAMP_DOWN_RPM_PER_S=1\n");
  CHECK_FALSE(lr.ok);
  CHECK(lr.error == core::ConfigError::RpmRange);
}

TEST_CASE("config: unknown key rejected") {
  core::LoadResult lr = core::load_config(
      "RUN_MODE=STEPPED\n"
      "SWEEP_START_RPM=100\nSWEEP_END_RPM=1000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=1\n"
      "SWEEP_SAMPLES=1\nRAMP_UP_RPM_PER_S=1\nRAMP_DOWN_RPM_PER_S=1\nBOGUS=1\n");
  CHECK_FALSE(lr.ok);
  CHECK(lr.error == core::ConfigError::UnknownKey);
}

TEST_CASE("config: SWEEP_START_RPM>SWEEP_END_RPM rejected") {
  core::LoadResult lr = core::load_config(
      "RUN_MODE=STEPPED\n"
      "SWEEP_START_RPM=2000\nSWEEP_END_RPM=1000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=1\n"
      "SWEEP_SAMPLES=1\nRAMP_UP_RPM_PER_S=1\nRAMP_DOWN_RPM_PER_S=1\n");
  CHECK_FALSE(lr.ok);
  CHECK(lr.error == core::ConfigError::RpmRange);
}

TEST_CASE("config: SWEEP_END_RPM above hardware ceiling rejected") {
  // Ceiling is derived from the BAKED geometry: narrowest interval is the 10deg
  // tooth width -> (10/360)*60*125MHz/1000 = ~208333 rpm; 2000000 exceeds it.
  core::LoadResult lr = core::load_config(
      "RUN_MODE=STEPPED\n"
      "SWEEP_START_RPM=100\nSWEEP_END_RPM=2000000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=1\n"
      "SWEEP_SAMPLES=1\nRAMP_UP_RPM_PER_S=1\nRAMP_DOWN_RPM_PER_S=1\n");
  CHECK_FALSE(lr.ok);
  CHECK(lr.error == core::ConfigError::RpmCeiling);
}

TEST_CASE("config: HOLD_RPM is OPTIONAL — a config without it still loads") {
  // No HOLD_RPM line -> hold_rpm defaults to kDefaultHoldRpm.
  core::LoadResult lr = core::load_config(
      "RUN_MODE=STEPPED\n"
      "SWEEP_START_RPM=100\nSWEEP_END_RPM=10000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=8\n"
      "SWEEP_SAMPLES=16\nRAMP_UP_RPM_PER_S=500\nRAMP_DOWN_RPM_PER_S=500\n");
  REQUIRE(lr.ok);
  CHECK(lr.config.hold_rpm == cfg::kDefaultHoldRpm);
}

TEST_CASE("config: RUN_MODE=HOLD with HOLD_RPM parses") {
  core::LoadResult lr = core::load_config(
      "RUN_MODE=HOLD\nHOLD_RPM=5000\n"
      "SWEEP_START_RPM=100\nSWEEP_END_RPM=10000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=8\n"
      "SWEEP_SAMPLES=16\nRAMP_UP_RPM_PER_S=500\nRAMP_DOWN_RPM_PER_S=500\n");
  REQUIRE(lr.ok);
  CHECK(lr.config.run_mode == core::RunMode::Hold);
  CHECK(lr.config.hold_rpm == 5000u);
}

TEST_CASE("config: RUN_MODE=HOLD with HOLD_RPM=0 rejected (mode-gated)") {
  core::LoadResult lr = core::load_config(
      "RUN_MODE=HOLD\nHOLD_RPM=0\n"
      "SWEEP_START_RPM=100\nSWEEP_END_RPM=10000\nSWEEP_STEP_RPM=100\nSWEEP_SETTLE_REVS=8\n"
      "SWEEP_SAMPLES=16\nRAMP_UP_RPM_PER_S=500\nRAMP_DOWN_RPM_PER_S=500\n");
  CHECK_FALSE(lr.ok);
}
