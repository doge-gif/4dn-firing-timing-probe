#include "core/fileio.hpp"
#include "doctest/doctest.h"

#include <cstring>
#include <string>

namespace {
struct FakeBackend : core::Backend {
  std::string cfg_text;
  std::string written;
  bool read_config(char* buf, std::size_t cap, std::size_t& len) override {
    if (cfg_text.size() >= cap)
      return false;
    std::memcpy(buf, cfg_text.data(), cfg_text.size());
    len = cfg_text.size();
    return true;
  }
  bool append(std::string_view s) override {
    written.append(s);
    return true;
  }
};
constexpr char kCfg[] = "RUN_MODE=STEPPED\n"
                        "SWEEP_START_RPM=100\nSWEEP_END_RPM=10000\nSWEEP_STEP_RPM=100\n"
                        "SWEEP_SETTLE_REVS=8\nSWEEP_SAMPLES=64\n"
                        "RAMP_UP_RPM_PER_S=500\nRAMP_DOWN_RPM_PER_S=500\n";
} // namespace

TEST_CASE("fileio: load_config_via_backend returns a valid config") {
  FakeBackend b;
  b.cfg_text = kCfg;
  core::LoadResult lr = core::load_config_via_backend(b);
  REQUIRE(lr.ok);
  CHECK(lr.config.sweep_end_rpm == 10000u);
}

TEST_CASE("fileio: writing a stepped row appends CSV text") {
  FakeBackend b;
  core::SteppedRow r{};
  r.rpm_cmd = 6000;
  r.cyl = 0;
  r.n = 1;
  core::write_stepped_row(b, r);
  CHECK(b.written.find("6000,0,1,") != std::string::npos);
}
