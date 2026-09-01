#include "core/csv.hpp"

#include <cstdio>

namespace core {

std::size_t csv_stepped_header(char* buf, std::size_t cap) {
  int n = std::snprintf(buf, cap,
                        "rpm_cmd,cyl,n,mean_btdc_deg,median_btdc_deg,stddev_btdc_deg,"
                        "min_btdc_deg,max_btdc_deg,mean_dwell_us,stddev_dwell_us,"
                        "mean_sys_latency_us,tach_rpm\n");
  return (n < 0 || std::size_t(n) >= cap) ? 0 : std::size_t(n);
}
std::size_t csv_stepped_row(char* buf, std::size_t cap, const SteppedRow& r) {
  // Cast every uint32_t to `unsigned` for %u: on arm-none-eabi newlib uint32_t is
  // `unsigned long`, so a bare %u would be format-UB / -Wformat there. `unsigned`
  // matches %u on both host and target and reproduces the same decimal text.
  int n = std::snprintf(buf, cap, "%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        unsigned(r.rpm_cmd), unsigned(r.cyl), unsigned(r.n), r.mean_btdc_deg,
                        r.median_btdc_deg, r.stddev_btdc_deg, r.min_btdc_deg, r.max_btdc_deg,
                        r.mean_dwell_us, r.stddev_dwell_us, r.mean_sys_latency_us, r.tach_rpm);
  return (n < 0 || std::size_t(n) >= cap) ? 0 : std::size_t(n);
}
std::size_t csv_ramp_header(char* buf, std::size_t cap) {
  int n = std::snprintf(buf, cap, "rpm_cmd,cyl,rev_period_us,btdc_deg,dwell_us\n");
  return (n < 0 || std::size_t(n) >= cap) ? 0 : std::size_t(n);
}
std::size_t csv_ramp_row(char* buf, std::size_t cap, const RampRow& r) {
  int n = std::snprintf(buf, cap, "%u,%u,%u,%.2f,%.2f\n", unsigned(r.rpm_cmd), unsigned(r.cyl),
                        unsigned(r.rev_period_us), r.btdc_deg, r.dwell_us);
  return (n < 0 || std::size_t(n) >= cap) ? 0 : std::size_t(n);
}

} // namespace core
