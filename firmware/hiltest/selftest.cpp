// HIL self-test harness (target-only, built with -DPROBER_HILTEST=ON).
// Observation channel is USB-CDC (printf); results are read from /dev/ttyACM*.
// Each gate prints machine-checkable lines, then returns to BOOTSEL via
// reset_usb_boot() so the next .uf2 can be flashed without a physical button.
#include "app/run_modes.hpp"
#include "core/angle.hpp"
#include "core/config.hpp"
#include "core/csv.hpp"
#include "core/stats.hpp"
#include "core/types.hpp"
#include "hal/capture.hpp"
#include "hal/pins.hpp"
#include "hal/soft_pattern.hpp"
#include "hardware/pio.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"

#include <cmath>
#include <cstdint>
#include <cstdio>

// Gate D: end-to-end dry run (no ignitor). Run a config-driven stepped RPM sweep
// using the run-mode scheduler; at each step drive the SRV250 pickup pattern and
// self-capture it (wireless), synthesize an "ignitor" that fires a fixed TIME
// after each tooth-0 rising edge, and push the captured pickup edges + synthetic
// spark through the full measurement pipeline (bracket -> interp_angle -> to_btdc
// -> Welford + median/min/max -> CSV row). Emits the MAP CSV over CDC.
//
// Because the synthetic spark is a fixed TIME delay, the measured advance is
// RPM-dependent: BTDC = TDC - (delay_us / rev_period_us) * 360. With TDC=30deg
// and delay=800us the expected map is 2000->20.4, 4000->10.8, 6000->1.2 deg.
namespace {

constexpr float kTdcRef = 30.0f;        // BTDC datum (deg)
constexpr float kSparkDelayUs = 800.0f; // synthetic ignitor: fire this long after tooth0
constexpr float kCaptureCountsPerUs = cfg::kCaptureCountsPerUs; // 62.5 MHz capture counter
// Intra-revolution angle of each captured edge (SRV250: 4 teeth x 2 edges).
constexpr float kIntraAngle[8] = {0, 10, 60, 70, 120, 130, 180, 190};

core::Config sweep_config() {
  core::Config c{};
  c.wheel_deg = 360.0f;
  c.tooth_count = 4;
  c.teeth[0] = core::ToothSpan{0.0f, 10.0f};
  c.teeth[1] = core::ToothSpan{60.0f, 70.0f};
  c.teeth[2] = core::ToothSpan{120.0f, 130.0f};
  c.teeth[3] = core::ToothSpan{180.0f, 190.0f};
  c.cyl_count = 1;
  c.sweep_start_rpm = 2000;
  c.sweep_end_rpm = 6000;
  c.sweep_step_rpm = 2000;
  c.sweep_settle_revs = 1;
  c.sweep_samples = 4;
  return c;
}

// Measure mean/median/min/max BTDC for one RPM step from freshly captured edges.
// Returns the number of samples reduced.
std::uint32_t measure_step(hal::Capture& capt, std::uint32_t hold_revs, std::uint32_t samples,
                           float delay_counts, core::SteppedRow& row) {
  static hal::CaptureEvent evs[192];
  std::size_t n = capt.drain(evs, 192);
  core::Welford w;
  static float buf[64];
  std::uint32_t got = 0;
  const std::size_t revs = n / 8;
  for (std::size_t r = hold_revs; r < revs && got < samples; ++r) {
    const std::size_t base = r * 8;
    if (base + 2 >= n)
      break;
    std::uint64_t t0 = evs[base].ticks;                       // tooth0 rising
    std::uint64_t spark_t = t0 + std::uint64_t(delay_counts); // synthetic spark
    // Find the bracketing captured edges within this revolution.
    std::size_t i = base;
    while (i + 1 < n && evs[i + 1].ticks <= spark_t)
      ++i;
    if (i + 1 >= n)
      break;
    core::EdgeRef before{evs[i].ticks, kIntraAngle[(i - base) & 7]};
    core::EdgeRef after{evs[i + 1].ticks, kIntraAngle[(i + 1 - base) & 7]};
    float ang = core::interp_angle(spark_t, before, after);
    float btdc = core::to_btdc(ang, kTdcRef, 360.0f);
    w.add(btdc);
    if (got < 64)
      buf[got] = btdc;
    ++got;
  }
  core::MedianResult mm = core::median_minmax(buf, got);
  row.cyl = 0;
  row.n = got;
  row.mean_btdc_deg = w.mean();
  row.median_btdc_deg = mm.median;
  row.stddev_btdc_deg = w.stddev_sample();
  row.min_btdc_deg = mm.min;
  row.max_btdc_deg = mm.max;
  row.mean_dwell_us = 0.0f; // no dwell in this synthetic dry run
  row.stddev_dwell_us = 0.0f;
  return got;
}

} // namespace

int main() {
  stdio_init_all(); // USB-CDC
  const uint LED = PICO_DEFAULT_LED_PIN;
  gpio_init(LED);
  gpio_set_dir(LED, GPIO_OUT);
  gpio_put(LED, 1);

  core::Config cfg = sweep_config();

  static hal::SoftPattern pg;
  static hal::Capture capt;
  pg.init(hal::kPickup);
  capt.init_pickup_selfcapture(0, pio1, hal::kPickup, 0);
  capt.start();

  const float delay_counts = kSparkDelayUs * kCaptureCountsPerUs;

  // Run the sweep, collecting one reduced CSV row per step.
  static char csv[192]; // stepped header is ~140 chars now (tach diagnostic columns)
  static core::SteppedRow rows[8];
  static float exp_btdc[8];
  static std::uint32_t rpms[8];
  int nrows = 0;

  app::SteppedSchedule sched(cfg);
  while (!sched.done() && nrows < 8) {
    std::uint32_t rpm = sched.current_rpm();
    // Restart emission cleanly at this RPM: stop, discard stale captured edges,
    // then start SoftPattern so the first captured edge is tooth-0 (measure_step
    // groups captured edges into clean 8-per-rev blocks starting at index 0).
    pg.idle_low();
    hal::CaptureEvent scratch[192];
    while (capt.drain(scratch, 192) > 0) {
    } // discard stale edges
    pg.start(cfg, rpm ? rpm : 1, time_us_64());
    // Let it settle + accumulate samples (hold + samples + margin revolutions).
    float rev_ms = 60000.0f / float(rpm);
    std::uint32_t wait_ms =
        std::uint32_t((float(cfg.sweep_settle_revs + cfg.sweep_samples + 2)) * rev_ms) + 5;
    sleep_ms(wait_ms);

    core::SteppedRow row{};
    row.rpm_cmd = rpm;
    std::uint32_t got =
        measure_step(capt, cfg.sweep_settle_revs, cfg.sweep_samples, delay_counts, row);
    rows[nrows] = row;
    rpms[nrows] = rpm;
    exp_btdc[nrows] = kTdcRef - kSparkDelayUs * float(rpm) * 6.0e-6f; // TDC - delay_us*rpm*360/60e6
    (void)got;
    ++nrows;
    sched.advance_step();
  }
  pg.idle_low();

  // Evaluate PASS: each row has the wanted sample count and mean within 0.5 deg
  // of the analytic expectation.
  bool pass = (nrows == 3);
  for (int i = 0; i < nrows; ++i) {
    if (rows[i].n != cfg.sweep_samples)
      pass = false;
    if (std::fabs(rows[i].mean_btdc_deg - exp_btdc[i]) > 0.5f)
      pass = false;
  }

  for (int rep = 0; rep < 20; ++rep) {
    gpio_put(LED, rep & 1);
    printf("==== GATE D (end-to-end dry-run sweep) rep=%d ====\n", rep);
    core::csv_stepped_header(csv, sizeof csv);
    printf("%s", csv);
    for (int i = 0; i < nrows; ++i) {
      core::csv_stepped_row(csv, sizeof csv, rows[i]);
      printf("%s", csv);
      printf("GATED rpm=%lu mean_btdc=%.3f exp=%.3f n=%lu\n", (unsigned long)rpms[i],
             (double)rows[i].mean_btdc_deg, (double)exp_btdc[i], (unsigned long)rows[i].n);
    }
    printf("GATED RESULT %s (rows=%d)\n", pass ? "PASS" : "FAIL", nrows);
    sleep_ms(1000);
  }
  printf("HIL-DONE reset_to_bootsel\n");
  sleep_ms(50);
  reset_usb_boot(0, 0);
  while (true) {
  }
}
