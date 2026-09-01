// HIL Gate H (real ignitor): drive the pickup pattern into the
// LIVE ignitor and capture its REAL ignition outputs (GP16/GP17) to extract the
// actual BTDC-vs-RPM map. Reuses the PRODUCTION core-1 pipeline (app::core1_entry:
// pickup drive + real ignition capture + tach diagnostic + per-cyl reduce), driven
// by a self-recovering HIL core 0 that auto-starts the configured run, flushes
// MAP_H01.CSV, and exposes it over USB-MSC.
//
// Config is FILE-DRIVEN: the geometry / TDC / rpm range / run mode come from
// CONFIG.INI on the on-board FAT volume (parsed by core::load_config), NOT from a
// C++ hardcode. If CONFIG.INI is absent or invalid, the built-in pristine default
// (cfg::kDefaultConfigIni -- the single source of truth for SRV250 geometry) is
// used and written out so the file exists to edit. The volume is writable while
// idle and read-only during a run, so the config can be edited over USB between
// runs; the final flush preserves CONFIG.INI across the map's fresh-format.
//
// The pickup pattern is now generated on core1 in software (hal::SoftPattern), so
// there is no PIO/DMA reconfigure glitch; live progress is read over SWD
// (g_ic.cur_rpm / steps_done), so core0 does NOT write flash mid-run (a flash
// write locks out core1, which would pause the software pattern and desync the
// ignitor). Only the final map + config flush touches flash, after the run ends.
#include "app/core1_main.hpp" // app::core1_entry(), app::g_ic
#include "constants.hpp"      // cfg::kDefaultConfigIni
#include "core/config.hpp"    // core::load_config
#include "core/csv.hpp"
#include "core/types.hpp"
#include "hal/flashfs.hpp"
#include "hal/gpio.hpp"
#include "hal/pins.hpp"
#include "hal/usb_msc.hpp"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include <cstdint>
#include <cstring>
#include <string_view>

namespace {

// The exact config text we are running (from CONFIG.INI, or the built-in default).
// Retained so the final flush can rewrite it verbatim across the fresh-format,
// keeping the user's edits intact.
char g_cfg_text[cfg::kConfigTextBufBytes];
std::size_t g_cfg_len = 0;

void seed_default_config_text() {
  std::size_t i = 0;
  for (; cfg::kDefaultConfigIni[i] != '\0' && i < sizeof(g_cfg_text) - 1; ++i)
    g_cfg_text[i] = cfg::kDefaultConfigIni[i];
  g_cfg_text[i] = '\0';
  g_cfg_len = i;
}

// Load CONFIG.INI from the FS if present+valid; otherwise fall back to the pristine
// built-in default. Always leaves g_cfg_text/g_cfg_len holding the text in use.
core::Config load_config_from_fs() {
  std::size_t len = 0;
  if (hal::fs_read("CONFIG.INI", g_cfg_text, sizeof(g_cfg_text) - 1, len) && len > 0) {
    g_cfg_text[len] = '\0';
    g_cfg_len = len;
    core::LoadResult lr = core::load_config(std::string_view(g_cfg_text, g_cfg_len));
    if (lr.ok)
      return lr.config;
    // Present but invalid -> fall through to the default (so a bad edit can't brick
    // the run; the default is rewritten at flush, replacing the bad file).
  }
  seed_default_config_text();
  return core::load_config(std::string_view(g_cfg_text, g_cfg_len)).config; // always valid
}

} // namespace

int main() {
  hal::init_io(); // GP18 low, GP16/17/19 inputs, LED off
  const uint led = PICO_DEFAULT_LED_PIN;
  gpio_set_dir(led, GPIO_OUT);
  gpio_put(led, 1);

  hal::fs_mount();
  const core::Config cfg = load_config_from_fs(); // FILE-DRIVEN (see header)

  // Writable while idle so CONFIG.INI can be edited over USB; flipped read-only for
  // the duration of a run below (host writes program flash and would fence core1).
  hal::msc_set_readonly(false);
  tusb_rhport_init_t dev_init = {};
  dev_init.role = TUSB_ROLE_DEVICE;
  dev_init.speed = TUSB_SPEED_AUTO;
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  // Publish config, launch the production pipeline on core 1, then auto-start.
  app::g_ic.config = cfg;
  multicore_launch_core1(app::core1_entry);
  sleep_ms(50); // let core1 claim HW + multicore_lockout_victim_init()
  app::g_ic.run_request.store(true, std::memory_order_release);

  // Row buffer sized for the widest sweep (100..10000 step 100 -> 200 rows).
  static constexpr int kMaxRows = 256;
  static core::SteppedRow rows[kMaxRows];
  int nrows = 0;
  int flushed = 0;
  const std::uint32_t start = to_ms_since_boot(get_absolute_time());
  std::uint32_t last_blink = start;
  bool ledst = true;

  // Long window: the full-range sweep is dominated by the slow low-rpm steps. We
  // flush as soon as the sweep completes (below), then keep serving MSC until the
  // window closes and the board self-recovers to BOOTSEL. Live progress is read
  // over SWD (g_ic.cur_rpm / steps_done) -- core0 writes NO flash mid-run.
  while (to_ms_since_boot(get_absolute_time()) - start < 600000) {
    tud_task();

    // Read-only exactly while a run is active: no host writes can land on flash
    // while core1 is emitting/capturing (a host write would lock core1 out and
    // pause the software pattern). Editable again as soon as the run ends.
    hal::msc_set_readonly(app::g_ic.run_active.load(std::memory_order_acquire));

    core::SteppedRow r{};
    while (app::g_ic.results.pop(r))
      if (nrows < kMaxRows)
        rows[nrows++] = r;

    const std::uint32_t t = to_ms_since_boot(get_absolute_time());
    if (t - last_blink >= 100) {
      last_blink = t;
      ledst = !ledst;
      gpio_put(led, ledst);
    }

    const bool done = app::g_ic.run_done.load(std::memory_order_acquire);
    const bool lost = app::g_ic.lost_sync.load(std::memory_order_acquire);
    // FINAL flush: once, on completion. Fresh-format for a clean map, then REWRITE
    // CONFIG.INI (preserving any user edits across the format) and the full map.
    // Uses a bounded lockout so a wedged core1 can never block core0's self-recover.
    if ((done || lost) && !flushed) {
      flushed = 1;
      if (multicore_lockout_start_timeout_us(500000)) {
        hal::fs_format();
        hal::fs_mount();
        hal::fs_append("CONFIG.INI", std::string_view(g_cfg_text, g_cfg_len));
        char line[192];
        std::size_t hn = core::csv_stepped_header(line, sizeof line);
        hal::fs_append("MAP_H01.CSV", std::string_view(line, hn));
        for (int i = 0; i < nrows; ++i) {
          std::size_t rn = core::csv_stepped_row(line, sizeof line, rows[i]);
          hal::fs_append("MAP_H01.CSV", std::string_view(line, rn));
        }
        multicore_lockout_end_blocking();
      }
    }
  }
  reset_usb_boot(0, 0);
  while (true) {
  }
}
