#include "core/csv.hpp"
#include "doctest/doctest.h"

#include <cstring>
#include <string_view>

TEST_CASE("csv: stepped header and row") {
  char buf[192];
  std::size_t n = core::csv_stepped_header(buf, sizeof buf);
  CHECK(std::string_view(buf, n) == "rpm_cmd,cyl,n,mean_btdc_deg,median_btdc_deg,stddev_btdc_deg,"
                                    "min_btdc_deg,max_btdc_deg,mean_dwell_us,stddev_dwell_us,"
                                    "mean_sys_latency_us,tach_rpm\n");

  core::SteppedRow r{};
  r.rpm_cmd = 6000;
  r.cyl = 0;
  r.n = 64;
  r.mean_btdc_deg = 28.5f;
  r.median_btdc_deg = 28.4f;
  r.stddev_btdc_deg = 0.30f;
  r.min_btdc_deg = 27.9f;
  r.max_btdc_deg = 29.1f;
  r.mean_dwell_us = 2500.0f;
  r.stddev_dwell_us = 12.0f;
  r.mean_sys_latency_us = 800.0f;
  r.tach_rpm = 5990.0f;
  n = core::csv_stepped_row(buf, sizeof buf, r);
  CHECK(std::string_view(buf, n) ==
        "6000,0,64,28.50,28.40,0.30,27.90,29.10,2500.00,12.00,800.00,5990.00\n");
}

TEST_CASE("csv: ramp header and row") {
  char buf[96];
  std::size_t h = core::csv_ramp_header(buf, sizeof buf);
  CHECK(std::string_view(buf, h) == "rpm_cmd,cyl,rev_period_us,btdc_deg,dwell_us\n");
  core::RampRow r{};
  r.rpm_cmd = 3050;
  r.cyl = 1;
  r.rev_period_us = 19672;
  r.btdc_deg = 18.2f;
  r.dwell_us = 2600.0f;
  std::size_t n = core::csv_ramp_row(buf, sizeof buf, r);
  CHECK(std::string_view(buf, n) == "3050,1,19672,18.20,2600.00\n");
}

TEST_CASE("csv: truncation is reported, not overflowed") {
  char buf[4];
  core::RampRow r{};
  r.rpm_cmd = 3050;
  std::size_t n = core::csv_ramp_row(buf, sizeof buf, r);
  CHECK(n == 0); // did not fit -> 0, buffer not overrun
}
