// HIL Gate G: validate the RAMP path on silicon, wireless. core 1 runs a
// continuous RPM ramp (RampSchedule -> SoftPattern::set_rpm), self-captures
// GP18, synthesizes a spark per rev at a fixed time delay, and streams per-spark
// core::RampRow across the REAL app::Spsc to core 0. core 0 flushes RAMPU001.CSV
// to flash and exposes it over USB-MSC.
//
// DIAGNOSTIC/SELF-RECOVERING design (a prior version hung core 1 and bricked the
// flash loop): USB comes up FIRST so the CDC banner reports progress even if
// core 1 stalls; core 1's loop has a wall-time cap so a loop-hang becomes a clean
// completion (distinguishes loop-hang from hard fault via the banner); core 0
// ALWAYS reset_usb_boot() after 60 s so the board returns to BOOTSEL regardless.
#include "app/run_modes.hpp"
#include "app/spsc.hpp"
#include "core/angle.hpp"
#include "core/csv.hpp"
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

namespace {

app::Spsc<core::RampRow, cfg::kResultRingLen> g_ramp;
volatile bool g_core1_done = false;
volatile int g_pushed = 0;
volatile int g_stage = 0; // core1 progress marker: 1=inited 2=started 3=loop 4=post 5=done

constexpr float kTdcRef = 30.0f;
constexpr float kSparkDelayUs = 800.0f;
constexpr float kIntraAngle[8] = {0, 10, 60, 70, 120, 130, 180, 190};
constexpr std::uint64_t kCore1CapUs = 3'000'000; // 3 s wall cap on the ramp loop

core::Config ramp_config() {
  core::Config c{};
  c.wheel_deg = 360.0f;
  c.tooth_count = 4;
  c.teeth[0] = core::ToothSpan{0.0f, 10.0f};
  c.teeth[1] = core::ToothSpan{60.0f, 70.0f};
  c.teeth[2] = core::ToothSpan{120.0f, 130.0f};
  c.teeth[3] = core::ToothSpan{180.0f, 190.0f};
  c.cyl_count = 1;
  c.run_mode = core::RunMode::RampUp;
  c.sweep_start_rpm = 2000;
  c.sweep_end_rpm = 6000;
  c.ramp_up_rpm_per_s = 8000.0f;
  c.ramp_down_rpm_per_s = 8000.0f;
  return c;
}

void push_retry(const core::RampRow& r) {
  for (int i = 0; i < 50000 && !g_ramp.push(r); ++i)
    tight_loop_contents();
  ++g_pushed;
}

void __not_in_flash_func(core1_park)() {
  while (true)
    tight_loop_contents();
}

void core1_main() {
  static hal::SoftPattern pg;
  static hal::Capture capt;
  pg.init(hal::kPickup);
  capt.init_pickup_selfcapture(0, pio1, hal::kPickup, 0);
  capt.start();
  g_stage = 1;

  core::Config c = ramp_config();
  const float delay_counts = kSparkDelayUs * cfg::kCaptureCountsPerUs;
  app::RampSchedule sched(c, /*up=*/true);

  // Discard stale captured edges so the first emitted edge is captured as tooth-0,
  // then begin continuous emission at the ramp's starting RPM.
  std::uint32_t rpm0 = std::uint32_t(sched.current_rpm() + 0.5f);
  if (rpm0 == 0)
    rpm0 = 1;
  hal::CaptureEvent scratch[192];
  while (capt.drain(scratch, 192) > 0) {
  }
  pg.start(c, rpm0, time_us_64());
  g_stage = 2;

  static hal::CaptureEvent edges[512];
  std::size_t nedges = 0;
  std::size_t next_rev = 0;
  std::uint64_t last_us = time_us_64();
  std::uint64_t t_start = last_us;
  g_stage = 3;

  while (!sched.done() && nedges < 480 && (time_us_64() - t_start) < kCore1CapUs) {
    std::uint64_t now = time_us_64();
    sched.tick(float(now - last_us) / 1.0e6f);
    last_us = now;
    float rpm = sched.current_rpm();
    std::uint32_t r = std::uint32_t(rpm + 0.5f);
    // Continuous rpm update: SoftPattern retimes at the next rev boundary (no
    // stop/start, no phase reset), keeping the 8-edge/rev tooth-0-first structure.
    pg.set_rpm(r ? r : 1);
    if (nedges < 480)
      nedges += capt.drain(edges + nedges, 480 - nedges);
    while ((next_rev + 1) * 8 < nedges) {
      std::size_t base = next_rev * 8;
      std::uint64_t tooth0 = edges[base].ticks;
      std::uint64_t period_counts = edges[(next_rev + 1) * 8].ticks - tooth0;
      std::uint32_t rev_period_us = std::uint32_t(float(period_counts) / cfg::kCaptureCountsPerUs);
      float meas_rpm = (rev_period_us > 0) ? (60.0e6f / float(rev_period_us)) : 0.0f;
      std::uint64_t spark_t = tooth0 + std::uint64_t(delay_counts);
      std::size_t i = base;
      while (i + 1 < nedges && edges[i + 1].ticks <= spark_t)
        ++i;
      core::EdgeRef before{edges[i].ticks, kIntraAngle[(i - base) & 7]};
      core::EdgeRef after{edges[i + 1].ticks, kIntraAngle[(i + 1 - base) & 7]};
      core::RampRow row{};
      row.rpm_cmd = std::uint32_t(meas_rpm + 0.5f);
      row.cyl = 0;
      row.rev_period_us = rev_period_us;
      row.btdc_deg = core::to_btdc(core::interp_angle(spark_t, before, after), kTdcRef, 360.0f);
      row.dwell_us = 0.0f;
      push_retry(row);
      ++next_rev;
    }
  }
  g_stage = 4;
  pg.idle_low();
  g_core1_done = true;
  g_stage = 5;
  core1_park();
}

void banner(const char* phase, int nrows, int flushed) {
  char b[160];
  std::snprintf(b, sizeof b, "GATEG %s stage=%d done=%d pushed=%d nrows=%d flushed=%d\r\n", phase,
                g_stage, g_core1_done ? 1 : 0, g_pushed, nrows, flushed);
  tud_cdc_write_str(b);
  tud_cdc_write_flush();
}

} // namespace

int main() {
  const uint led = PICO_DEFAULT_LED_PIN;
  gpio_init(led);
  gpio_set_dir(led, GPIO_OUT);
  gpio_put(led, 1);

  hal::fs_mount(); // read-only mount (no flash write yet); ok before core1 launch
  hal::msc_set_readonly(true);
  tusb_rhport_init_t dev_init = {};
  dev_init.role = TUSB_ROLE_DEVICE;
  dev_init.speed = TUSB_SPEED_AUTO;
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  multicore_launch_core1(core1_main);

  static core::RampRow rows[512];
  int nrows = 0;
  int flushed = 0;
  const std::uint32_t start = to_ms_since_boot(get_absolute_time());
  std::uint32_t last_blink = start, last_banner = 0;
  bool ledst = true;

  while (to_ms_since_boot(get_absolute_time()) - start < 60000) {
    tud_task();
    core::RampRow r{};
    while (g_ramp.pop(r))
      if (nrows < 512)
        rows[nrows++] = r;

    const std::uint32_t t = to_ms_since_boot(get_absolute_time());
    if (t - last_blink >= 100) {
      last_blink = t;
      ledst = !ledst;
      gpio_put(led, ledst);
    }

    // Flush ONCE, only after core1 is done+parked (safe flash write vs XIP).
    if (g_core1_done && !flushed) {
      hal::fs_format();
      hal::fs_mount();
      char line[96];
      std::size_t hn = core::csv_ramp_header(line, sizeof line);
      hal::fs_append("RAMPU001.CSV", std::string_view(line, hn));
      for (int i = 0; i < nrows; ++i) {
        std::size_t rn = core::csv_ramp_row(line, sizeof line, rows[i]);
        hal::fs_append("RAMPU001.CSV", std::string_view(line, rn));
      }
      flushed = 1;
    }

    if (tud_cdc_connected() && t - last_banner >= 1000) {
      last_banner = t;
      banner(flushed ? "SERVE" : "WAIT", nrows, flushed);
    }
  }
  reset_usb_boot(0, 0);
  while (true) {
  }
}
