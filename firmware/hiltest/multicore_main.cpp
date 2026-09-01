// HIL Gate F: validate the dual-core integration machinery on silicon, wireless.
// core 1 runs the (Gate-D) synthetic-spark measurement pipeline using the CPU
// SoftPattern emitter + self-capture, and hands reduced rows to core 0 via the REAL
// app::Spsc ring. core 0 drains the ring, flushes a MAP CSV to flash (FatFs),
// and exposes it over USB-MSC. Validates: multicore_launch_core1, cross-core
// SPSC handoff, flush-to-flash, and MSC retrieval -- everything except the real
// ignition-capture-from-a-live-signal (that needs a real ignitor).
//
// RP2040 multicore-flash hazard: while core 0 erases/programs flash, core 1 must
// NOT be executing from XIP flash or it faults. Here core 1 finishes its sweep
// and parks in a RAM-resident spin (__not_in_flash_func) before core 0 writes,
// so the flush is safe without multicore_lockout. NOTE: the real app
// needs the same discipline (RAM-park or multicore_lockout) around its end-of-run
// flush -- its core1 otherwise waits for the next run in a flash-resident loop.
#include "app/run_modes.hpp"
#include "app/spsc.hpp"
#include "core/angle.hpp"
#include "core/csv.hpp"
#include "core/stats.hpp"
#include "core/types.hpp"
#include "hal/capture.hpp"
#include "hal/flashfs.hpp"
#include "hal/pins.hpp"
#include "hal/soft_pattern.hpp"
#include "hal/usb_msc.hpp"
#include "hardware/pio.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

// Cross-core hand-off: the REAL app::Spsc type, core1 producer -> core0 consumer.
app::Spsc<core::SteppedRow, cfg::kResultRingLen> g_results;
volatile bool g_core1_done = false;

constexpr float kTdcRef = 30.0f;
constexpr float kSparkDelayUs = 800.0f;
constexpr float kCaptureCountsPerUs = cfg::kCaptureCountsPerUs;
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

// One reduced row for the current RPM step (Gate-D synthetic-spark logic).
void measure_step(hal::Capture& capt, std::uint32_t rpm, std::uint32_t hold_revs,
                  std::uint32_t samples, float delay_counts, core::SteppedRow& row) {
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
    std::uint64_t spark_t = evs[base].ticks + std::uint64_t(delay_counts);
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
  row.rpm_cmd = rpm;
  row.cyl = 0;
  row.n = got;
  row.mean_btdc_deg = w.mean();
  row.median_btdc_deg = mm.median;
  row.stddev_btdc_deg = w.stddev_sample();
  row.min_btdc_deg = mm.min;
  row.max_btdc_deg = mm.max;
  row.mean_dwell_us = 0.0f;
  row.stddev_dwell_us = 0.0f;
}

// RAM-resident park: core 1 spins here (NOT executing XIP flash) while core 0
// writes flash, so the flush cannot fault core 1.
void __not_in_flash_func(core1_park)() {
  while (true) {
    tight_loop_contents();
  }
}

// core 1 entry: run the sweep on core 1, push rows to core 0, then park in RAM.
void core1_main() {
  static hal::SoftPattern pg;
  static hal::Capture capt;
  pg.init(hal::kPickup);
  capt.init_pickup_selfcapture(0, pio1, hal::kPickup, 0);
  capt.start();

  core::Config c = sweep_config();
  const float delay_counts = kSparkDelayUs * kCaptureCountsPerUs;
  app::SteppedSchedule sched(c);
  while (!sched.done()) {
    std::uint32_t rpm = sched.current_rpm();
    // Restart emission cleanly at this RPM so the first captured edge is tooth-0.
    pg.idle_low();
    hal::CaptureEvent scratch[192];
    while (capt.drain(scratch, 192) > 0) {
    }
    pg.start(c, rpm ? rpm : 1, time_us_64());
    float rev_ms = 60000.0f / float(rpm);
    std::uint32_t wait_ms =
        std::uint32_t(float(c.sweep_settle_revs + c.sweep_samples + 2) * rev_ms) + 5;
    sleep_ms(wait_ms);
    core::SteppedRow row{};
    measure_step(capt, rpm, c.sweep_settle_revs, c.sweep_samples, delay_counts, row);
    g_results.push(row);
    sched.advance_step();
  }
  pg.idle_low();
  g_core1_done = true;
  core1_park(); // RAM-safe: core 0 may now write flash without faulting core 1
}

} // namespace

int main() {
  const uint led = PICO_DEFAULT_LED_PIN;
  gpio_init(led);
  gpio_set_dir(led, GPIO_OUT);
  gpio_put(led, 1);

  // Launch the deterministic domain on core 1.
  multicore_launch_core1(core1_main);

  // Drain reduced rows across the SPSC ring until core 1 signals completion.
  static core::SteppedRow rows[16];
  int nrows = 0;
  while (!g_core1_done || !g_results.empty()) {
    core::SteppedRow r{};
    while (g_results.pop(r)) {
      if (nrows < 16)
        rows[nrows++] = r;
    }
    sleep_ms(2);
  }
  // core 1 is now parked in RAM (core1_park), so the flash write below is safe.

  // Flush the MAP CSV to the flash FAT volume.
  bool fmt = hal::fs_format();
  bool mnt = hal::fs_mount();
  char line[192]; // stepped header is ~140 chars now (tach diagnostic columns)
  std::size_t hn = core::csv_stepped_header(line, sizeof line);
  bool wrote = hal::fs_append("MAP_F01.CSV", std::string_view(line, hn));
  for (int i = 0; i < nrows; ++i) {
    std::size_t rn = core::csv_stepped_row(line, sizeof line, rows[i]);
    wrote = hal::fs_append("MAP_F01.CSV", std::string_view(line, rn)) && wrote;
  }

  // Expose the volume read-only over USB-MSC (writer lock).
  hal::msc_set_readonly(true);
  tusb_rhport_init_t dev_init = {};
  dev_init.role = TUSB_ROLE_DEVICE;
  dev_init.speed = TUSB_SPEED_AUTO;
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  const std::uint32_t start = to_ms_since_boot(get_absolute_time());
  std::uint32_t last_blink = start, last_banner = 0;
  bool ledst = true;
  while (to_ms_since_boot(get_absolute_time()) - start < 60000) {
    tud_task();
    const std::uint32_t t = to_ms_since_boot(get_absolute_time());
    if (t - last_blink >= 100) {
      last_blink = t;
      ledst = !ledst;
      gpio_put(led, ledst);
    }
    if (tud_cdc_connected() && t - last_banner >= 1000) {
      last_banner = t;
      char b[128];
      std::snprintf(b, sizeof b,
                    "GATEF core1_done=%d rows=%d fmt=%d mnt=%d wrote=%d file=MAP_F01.CSV\r\n",
                    g_core1_done ? 1 : 0, nrows, fmt ? 1 : 0, mnt ? 1 : 0, wrote ? 1 : 0);
      tud_cdc_write_str(b);
      tud_cdc_write_flush();
    }
  }
  reset_usb_boot(0, 0);
  while (true) {
  }
}
