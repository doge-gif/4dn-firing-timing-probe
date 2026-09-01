#pragma once
#include <cstddef>
#include <cstdint>

namespace core {

struct SteppedRow {
  std::uint32_t rpm_cmd;
  std::uint8_t cyl;
  std::uint32_t n;
  float mean_btdc_deg, median_btdc_deg, stddev_btdc_deg, min_btdc_deg, max_btdc_deg;
  float mean_dwell_us, stddev_dwell_us;
  // Tach reference diagnostics (system input-path latency + sync-lock RPM).
  // REFERENCE only -- never used to correct BTDC. Default 0 on the bench (no tach
  // signal without the real ignitor; exercised only with one connected).
  float mean_sys_latency_us = 0.0f;
  float tach_rpm = 0.0f;
};
struct RampRow {
  std::uint32_t rpm_cmd;
  std::uint8_t cyl;
  std::uint32_t rev_period_us;
  float btdc_deg;
  float dwell_us;
};

// All return bytes written (0 if it would not fit; buffer never overrun).
std::size_t csv_stepped_header(char* buf, std::size_t cap);
std::size_t csv_stepped_row(char* buf, std::size_t cap, const SteppedRow& r);
std::size_t csv_ramp_header(char* buf, std::size_t cap);
std::size_t csv_ramp_row(char* buf, std::size_t cap, const RampRow& r);

} // namespace core
