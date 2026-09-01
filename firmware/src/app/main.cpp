// Core-0 integration super-loop.
//
// Core 0 owns USB (custom CDC+MSC composite exposing the flash FAT), the FatFs
// volume, the FSM + LED, and BOOTSEL. Core 1 owns the
// deterministic capture+reduce pipeline. The ONLY data that crosses core1->core0
// is reduced result rows (core::SteppedRow for stepped runs, core::RampRow for
// ramp/hold) over the SPSC rings in g_ic; a handful of atomics coordinate run
// start/stop and the flash-write window.
//
// Two hard interlocks are enforced here:
//   * WRITER LOCK: while a run holds the FS, the MSC medium is reported read-only
//     (hal::msc_set_readonly(true)) so the host PC cannot mutate the volume out
//     from under the firmware. It is released back to read-write once the FS is
//     free again (run finished + results flushed, or run aborted).
//   * FLASH-WRITE-IDLE: core0 NEVER writes flash while core1 is actively
//     capturing (g_ic.capture_idle == false). Result rows are buffered in SRAM
//     during the run and flushed only once capture is idle at end-of-run. This
//     avoids XIP stalls colliding with the deterministic capture path.
//
// The full runtime behavior (multicore + SPSC + MSC coexistence, BOOTSEL start,
// CONFIG.INI flow) integrates hardware-only paths and is exercised on silicon,
// not by the host unit tests.

#include "app/core1_main.hpp"
#include "core/config.hpp"
#include "core/csv.hpp"
#include "core/fileio.hpp"
#include "core/fsm.hpp"
#include "core/hold_ring.hpp"
#include "hal/flashfs.hpp"
#include "hal/flashfs_backend.hpp"
#include "hal/gpio.hpp"
#include "hal/usb_msc.hpp"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "version.hpp" // generated (build dir): build::kVersion from git describe

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

namespace {

std::uint32_t now_ms() { return to_ms_since_boot(get_absolute_time()); }

// ---- LED pattern renderer (core0-side) --------------------------------------
// core::led_for() maps a State to a LedPattern; here we turn that pattern into a
// concrete on/off level for the current millisecond. Purely time-based (a modulo
// of now_ms), no sleeps -- the caller sets the LED once per loop tick so nothing
// starves tud_task().
bool led_level(core::LedPattern p, std::uint32_t t) {
  switch (p) {
  case core::LedPattern::Off:
    return false;
  case core::LedPattern::Blink1Hz:
    return (t % 1000u) < 500u; // 1 Hz, 50% duty
  case core::LedPattern::Blink10Hz:
    return (t % 100u) < 50u; // 10 Hz, 50% duty
  case core::LedPattern::Heartbeat: {
    // Two short pulses near the start of a 1 s cycle, then quiet.
    const std::uint32_t ph = t % 1000u;
    return ph < 80u || (ph >= 160u && ph < 240u);
  }
  case core::LedPattern::DoubleBlink: {
    // Two ~120 ms blinks per 1 s (flush indicator).
    const std::uint32_t ph = t % 1000u;
    return ph < 120u || (ph >= 240u && ph < 360u);
  }
  case core::LedPattern::SolidBriefly:
    return true; // Done state is momentary; render solid on.
  case core::LedPattern::ErrorTriple: {
    // Three quick blinks then a pause, over a 1.6 s cycle.
    const std::uint32_t ph = t % 1600u;
    return ph < 120u || (ph >= 240u && ph < 360u) || (ph >= 480u && ph < 600u);
  }
  }
  return false;
}

// In-SRAM buffer for one sweep's reduced rows. The default full-range sweep is
// ~200 rows (100 steps x 2 cylinders), so 256 leaves only modest headroom; rows
// beyond the cap are dropped rather than overrun (comment at the drain site).
constexpr std::size_t kMaxRows = 256;

// Write a leading "# firmware: <git-describe>" comment into the currently-open
// output file, so every result CSV records which firmware produced it (data
// provenance). '#' is already the comment convention in these files (ramp
// TRUNCATED marker); readers skip it with comment='#'.
void write_version_stamp(hal::FlashFsBackend& be) {
  char vs[64];
  const int n = std::snprintf(vs, sizeof vs, "# firmware: %s\n", build::kVersion);
  if (n > 0)
    be.append(std::string_view(vs, static_cast<std::size_t>(n)));
}

// Perform the end-of-run flush to a fresh MAP_xxx.CSV. PRECONDITION: caller has
// verified g_ic.capture_idle == true (flash-write-idle interlock). Writes the CSV
// header then every buffered row. Returns after releasing the writer lock so the
// host is read-write again while the FS is idle.
void flush_results(hal::FlashFsBackend& be, const core::SteppedRow* rows, std::size_t n) {
  char name[13];
  // Scan the live FS for the lowest free MAP_xxx (never overwrite an existing file;
  // a RAM counter would reset to 1 on reboot and clobber last session's map).
  hal::fs_next_free_name(name, sizeof name, "MAP_%03u.CSV");
  be.set_output_file(name);
  be.open_output(); // stream the flush (append falls back to per-call if this fails)
  write_version_stamp(be);

  char hdr[192]; // stepped header is ~140 chars now (tach diagnostic columns)
  const std::size_t hn = core::csv_stepped_header(hdr, sizeof hdr);
  if (hn > 0)
    be.append(std::string_view(hdr, hn));
  for (std::size_t i = 0; i < n; ++i)
    core::write_stepped_row(be, rows[i]);
  be.close_output(); // sync + close the streamed file

  // FS is free again -> re-enable host writes (writer lock only while a
  // run holds the FS; RW when idle so the PC can manage files between runs).
  hal::msc_set_readonly(false);
}

// Drain the ramp per-spark rows core 1 produces into the SRAM buffer. Ramp is a
// higher-rate stream than stepped, so this MUST run every loop tick. Overflow
// beyond kMaxRampRows stops storing and raises the truncation flag, but keeps
// popping so core 1's producer ring never wedges (dropped rows are acceptable --
// the raw trajectory tolerates gaps; the flag makes the truncation non-silent).
void drain_ramp(std::array<core::RampRow, cfg::kMaxRampRows>& buf, std::size_t& n) {
  core::RampRow row;
  while (app::g_ic.ramp_results.pop(row)) {
    if (n < cfg::kMaxRampRows)
      buf[n++] = row;
    else
      app::g_ic.ramp_truncated.store(true, std::memory_order_relaxed);
  }
}

// End-of-run flush for a ramp -> RAMPU%03u.CSV (up) / RAMPD%03u.CSV (down); both
// are 8.3-valid (8-char base + .CSV) with LFN off. Same precondition + writer
// lock release as flush_results. If the run truncated, a trailing integrity line
// is appended so truncation is never silent.
void flush_ramp_results(hal::FlashFsBackend& be, const core::RampRow* rows, std::size_t n,
                        bool up) {
  char name[13];
  // Scan the live FS for the lowest free RAMPU/RAMPD xxx (never overwrite existing).
  hal::fs_next_free_name(name, sizeof name, up ? "RAMPU%03u.CSV" : "RAMPD%03u.CSV");
  be.set_output_file(name);
  be.open_output(); // stream the flush
  write_version_stamp(be);

  char hdr[64]; // ramp header is ~44 chars
  const std::size_t hn = core::csv_ramp_header(hdr, sizeof hdr);
  if (hn > 0)
    be.append(std::string_view(hdr, hn));
  for (std::size_t i = 0; i < n; ++i)
    core::write_ramp_row(be, rows[i]);

  // Integrity flag: make an over-cap ramp explicit in the file itself.
  if (app::g_ic.ramp_truncated.load(std::memory_order_relaxed))
    be.append(std::string_view("# TRUNCATED: ramp exceeded kMaxRampRows\n"));
  be.close_output(); // sync + close the streamed file

  hal::msc_set_readonly(false);
}

// End-of-hold flush -> HOLD%03u.CSV: the recent-rows window in chronological order
// (oldest -> newest). Same writer-lock release as flush_ramp_results.
void flush_hold(hal::FlashFsBackend& be,
                const core::RecentRing<core::RampRow, cfg::kHoldWindowRows>& ring) {
  char name[13];
  // Scan the live FS for the lowest free HOLD xxx (never overwrite existing files;
  // survives reboots, unlike the old RAM counter that reset to 1 -> collisions).
  hal::fs_next_free_name(name, sizeof name, "HOLD%03u.CSV");
  be.set_output_file(name);
  be.open_output(); // stream the flush (fast even for the full 2048-row window)
  write_version_stamp(be);

  char hdr[64];
  const std::size_t hn = core::csv_ramp_header(hdr, sizeof hdr);
  if (hn > 0)
    be.append(std::string_view(hdr, hn));
  ring.for_each_chronological([&](const core::RampRow& r) { core::write_ramp_row(be, r); });
  be.close_output(); // sync + close the streamed file

  hal::msc_set_readonly(false);
}

// RAM-resident cooperative park (HOLD BOOTSEL poll). While core1 needs to
// float the QSPI CS to read BOOTSEL, core0 must not fetch from flash. When core1
// sets park_request, spin here with interrupts DISABLED (so no flash-resident ISR
// runs and touches XIP) until core1 clears it, acking via park_ack so core1 knows
// it is safe to float CS. Bounded by a backstop iteration count -- time_us_64()
// cannot be used here (it would fetch flash while CS is floated). Deliberately does
// NOT use multicore_lockout: that is a single-victim primitive and core0 is the
// flush-write controller. Called every super-loop tick; a no-op unless a HOLD run's
// core1 poll is asking to park.
static void __no_inline_not_in_flash_func(core0_park_if_requested)() {
  if (!app::g_ic.park_request.load(std::memory_order_acquire))
    return;
  const std::uint32_t flags = save_and_disable_interrupts();
  app::g_ic.park_ack.store(true, std::memory_order_release);
  for (std::uint32_t i = 0; i < 2000000u; ++i) { // ~tens of ms backstop if core1 dies
    if (!app::g_ic.park_request.load(std::memory_order_acquire))
      break;
    tight_loop_contents();
  }
  app::g_ic.park_ack.store(false, std::memory_order_release);
  restore_interrupts(flags);
}

} // namespace

int main() {
  // 1. GPIO boot states: GP18 pickup output LOW, ign-sense inputs,
  //    LED off.
  hal::init_io();

  // 2. Bring up the FAT volume. Format on first boot (mount fails on a blank
  //    flash), then mount the fresh volume.
  if (!hal::fs_mount()) {
    hal::fs_format();
    hal::fs_mount();
  }

  // 2b. First-boot default config: if CONFIG.INI is absent, write the
  //     pristine baked-in default (cfg::kDefaultConfigIni) so a fresh device
  //     ships an editable config on the mounted USB drive. fs_read returns false
  //     only when the file cannot be opened (absent); a present-but-larger file
  //     yields a partial read (true), so an existing user config is never
  //     clobbered. This runs BEFORE multicore_launch_core1 (core 1 is not yet
  //     running), so the flash write needs no multicore lockout. After this,
  //     load+validate proceeds normally (step 5) -- a user who edits CONFIG.INI
  //     into an invalid state still gets CONFIG_INVALID (no silent fallback).
  {
    char probe[16];
    std::size_t plen = 0;
    if (!hal::fs_read("CONFIG.INI", probe, sizeof probe, plen))
      hal::fs_append("CONFIG.INI", cfg::kDefaultConfigIni);
  }

  // 2c. Refresh VERSION.TXT on the MSC volume so the user can see which firmware
  //     is running by opening the drive. Write only when absent or changed (a
  //     truncating fs_open_write), so an unchanged reboot does no flash write.
  //     Same pre-core1 window as 2b, so no multicore lockout is needed.
  {
    const std::string_view want(build::kVersion);
    char cur[64];
    std::size_t clen = 0;
    bool up_to_date = false;
    if (hal::fs_read("VERSION.TXT", cur, sizeof cur, clen)) {
      std::string_view have(cur, clen);
      while (!have.empty() && (have.back() == '\n' || have.back() == '\r' || have.back() == ' '))
        have.remove_suffix(1);
      up_to_date = (have == want);
    }
    if (!up_to_date && hal::fs_open_write("VERSION.TXT")) {
      hal::fs_stream_write(want);
      hal::fs_stream_write(std::string_view("\n"));
      hal::fs_close_write();
    }
  }

  // 3. Bring up USB (custom CDC+MSC composite from prober_usb; the flash volume
  //    is exposed as the MSC medium). Same tusb_init pattern as the MSC HIL gate.
  //    Start read-write: the host may manage CONFIG.INI / result files while the
  //    instrument is idle; the writer lock engages only for the duration of a run.
  hal::msc_set_readonly(false);
  tusb_rhport_init_t dev_init = {};
  dev_init.role = TUSB_ROLE_DEVICE;
  dev_init.speed = TUSB_SPEED_AUTO;
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  // 4. Hand the deterministic domain to core 1. It claims PIO/DMA once then waits
  //    on g_ic.run_request.
  multicore_launch_core1(app::core1_entry);

  // NOTE: core0 is deliberately NOT a multicore_lockout victim. The HOLD BOOTSEL
  // poll parks core0 via the cooperative park_request/park_ack handshake
  // (core0_park_if_requested below), NOT the SDK lockout -- that primitive is
  // single-victim and core0 is the flash-write CONTROLLER, so being a victim too
  // corrupts the shared lockout state and hangs the flush.

  // 5. Load + validate CONFIG.INI, then drive the FSM out of Boot. On success the
  //    validated Config is published to g_ic for core 1 to snapshot at run start.
  static hal::FlashFsBackend backend;
  core::State state = core::State::Boot;
  {
    const core::Transition tr = core::step(state, core::Event::Init); // -> ConfigLoad
    state = tr.next;
  }
  const core::LoadResult loaded = core::load_config_via_backend(backend);
  if (loaded.ok) {
    app::g_ic.config = loaded.config;                         // publish before any run is requested
    state = core::step(state, core::Event::ConfigValid).next; // -> Ready
  } else {
    state = core::step(state, core::Event::ConfigInvalid).next; // -> ConfigInvalid
  }

  // In-SRAM result buffers for the active run: stepped (reduced per-step rows) and
  // ramp (per-spark rows, much larger cap -- see cfg::kMaxRampRows). Only one is
  // used per run depending on cfg.run_mode.
  static std::array<core::SteppedRow, kMaxRows> rows;
  std::size_t n_rows = 0;
  static std::array<core::RampRow, cfg::kMaxRampRows> ramp_rows;
  std::size_t n_ramp = 0;
  // HOLD run mode: fixed-capacity overwrite-oldest window of per-spark rows (see
  // core/hold_ring.hpp) -- a forever-running measurement can't buffer the whole
  // stream, so only the most recent cfg::kHoldWindowRows rows are kept.
  static core::RecentRing<core::RampRow, cfg::kHoldWindowRows> hold_ring;

  // 6. Tick-based FSM super-loop. Each tick: service USB, render the
  //    LED, derive an Event from state + shared flags, step the FSM, execute the
  //    action. No blocking sleeps -- tud_task must run continuously.
  for (;;) {
    tud_task();                // service USB (enumeration, MSC READ10/WRITE10, CDC)
    core0_park_if_requested(); // honor a HOLD poll's cooperative park

    const std::uint32_t t = now_ms();
    hal::led_set(led_level(core::led_for(state), t));

    core::Event ev = core::Event::Tick; // default: no-op tick
    switch (state) {
    case core::State::Ready:
      // Poll BOOTSEL ONLY in Ready (it briefly pauses XIP; core0-only, once/tick).
      if (hal::bootsel_pressed() ||
          app::g_ic.debug_start.exchange(false, std::memory_order_acq_rel))
        ev = core::Event::StartPressed;
      break;

    case core::State::Running: {
      // Drain result rows into SRAM as core 1 produces them (no flash writes while
      // capturing). Stepped: reduced per-step rows; ramp: per-spark rows (higher
      // rate -- drained every tick); Hold: per-spark rows into the fixed
      // overwrite-oldest recent-rows window (see core/hold_ring.hpp). Overflow
      // beyond the cap is dropped, not overrun (ramp additionally flags
      // truncation; see drain_ramp -- Hold never truncates, it just ages out).
      if (app::g_ic.config.run_mode == core::RunMode::Stepped) {
        core::SteppedRow row;
        while (app::g_ic.results.pop(row)) {
          if (n_rows < kMaxRows)
            rows[n_rows++] = row;
        }
      } else if (app::g_ic.config.run_mode == core::RunMode::Hold) {
        core::RampRow row;
        while (app::g_ic.ramp_results.pop(row))
          hold_ring.push(row); // overwrite-oldest; bounded window
      } else {
        drain_ramp(ramp_rows, n_ramp);
      }
      // Lost-sync takes precedence (on abort core1 sets BOTH lost_sync and
      // run_done); route to Error rather than Flush.
      if (app::g_ic.lost_sync.load(std::memory_order_acquire))
        ev = core::Event::LostSync;
      else if (app::g_ic.run_done.load(std::memory_order_acquire))
        ev = core::Event::RunComplete;
      break;
    }

    case core::State::Flush: {
      // Sweep up any rows that raced in after RunComplete was observed. Same
      // three-way branch as Running above -- a bool can't carry three modes, so
      // Hold gets its own arm rather than falling into the ramp else (which would
      // silently lose Hold rows into ramp_rows).
      if (app::g_ic.config.run_mode == core::RunMode::Stepped) {
        core::SteppedRow row;
        while (app::g_ic.results.pop(row)) {
          if (n_rows < kMaxRows)
            rows[n_rows++] = row;
        }
      } else if (app::g_ic.config.run_mode == core::RunMode::Hold) {
        core::RampRow row;
        while (app::g_ic.ramp_results.pop(row))
          hold_ring.push(row);
      } else {
        drain_ramp(ramp_rows, n_ramp);
      }
      // FLASH-WRITE-IDLE INTERLOCK: write flash ONLY when core 1
      // reports capture idle. At end-of-run it is idle; if not yet, hold off and
      // retry next tick (USB keeps being serviced meanwhile).
      if (app::g_ic.capture_idle.load(std::memory_order_acquire)) {
        // RP2040 multicore-flash XIP hazard: flash_range_erase/_program on this
        // core stalls XIP for both cores, but only THIS core's interrupts are
        // disabled during the operation (see diskio_flash::program_sector) -- if
        // core 1 were still fetching from flash it would hard-fault. Force core 1
        // into its RAM-resident lockout handler for the duration of the write;
        // capture_idle (checked above) additionally ensures no capture is active,
        // but only the lockout guarantees core 1 isn't executing XIP at all.
        //
        // DUAL-WRITER COHERENCY: the host also writes this FAT volume (deleting old
        // result files while idle). The device's cached FatFs view (g_fs, mounted
        // at boot) goes STALE after those host writes; flushing a stale FAT resurrects
        // the deleted files and corrupts the volume (host `rm` reappears, Linux
        // remounts read-only). Re-mount here so FatFs re-reads the CURRENT on-flash
        // FAT before writing. The writer lock (msc_readonly, engaged since BeginRun)
        // holds the host read-only for the duration of this write, so the two writers
        // never interleave.
        hal::fs_mount();
        // The flush STREAMS (open once / buffered writes / close once, see
        // FlashFsBackend::open_output) instead of re-opening the file per row, so
        // even HOLD's 2048-row window flushes fast enough that tud_task() is not
        // starved long enough to corrupt an in-flight MSC transfer (the O(n^2)
        // per-row fs_append was what starved USB -> TinyUSB panic on HOLD).
        multicore_lockout_start_blocking();
        if (app::g_ic.config.run_mode == core::RunMode::Stepped)
          flush_results(backend, rows.data(), n_rows);
        else if (app::g_ic.config.run_mode == core::RunMode::Hold)
          flush_hold(backend, hold_ring);
        else
          flush_ramp_results(backend, ramp_rows.data(), n_ramp,
                             app::g_ic.config.run_mode == core::RunMode::RampUp);
        multicore_lockout_end_blocking();
        // The firmware just changed the FAT; tell the host to re-read so the new
        // CSV appears without a physical USB replug (a UNIT ATTENTION on Test Unit
        // Ready -> the OS re-reads the volume in place).
        hal::msc_signal_media_change();
        ev = core::Event::FlushDone;
      }
      break;
    }

    case core::State::Done:
    case core::State::Error:
      ev = core::Event::Tick; // -> Ready (ReturnReady)
      break;

    default:
      break; // Boot/ConfigLoad/ConfigInvalid: nothing to do on a tick
    }

    const core::Transition tr = core::step(state, ev);
    switch (tr.action) {
    case core::Action::BeginRun:
      // Engage the writer lock, reset BOTH SRAM buffers + the truncation flag +
      // handshake flags, and ask core 1 to start the run.
      n_rows = 0;
      n_ramp = 0;
      hold_ring.clear();
      app::g_ic.ramp_truncated.store(false, std::memory_order_relaxed);
      hal::msc_set_readonly(true); // writer lock: host read-only during the run
      app::g_ic.lost_sync.store(false, std::memory_order_relaxed);
      app::g_ic.run_done.store(false, std::memory_order_relaxed);
      app::g_ic.run_request.store(true, std::memory_order_release);
      break;

    case core::Action::ReturnReady:
      // Returning to Ready from Done or Error. Ensure host RW (covers the Error
      // path, which never reached flush_results) and clear stale flags so the next
      // run starts clean.
      hal::msc_set_readonly(false);
      app::g_ic.run_done.store(false, std::memory_order_relaxed);
      app::g_ic.lost_sync.store(false, std::memory_order_relaxed);
      break;

    // These three actions are no-ops at the transition itself:
    //  * WriteResults: the flash write happens in the Flush-state handler above,
    //    under the capture-idle interlock.
    //  * LoadConfig: config was loaded at boot (step 5); Boot->ConfigLoad is a no-op.
    //  * None: nothing to do.
    case core::Action::WriteResults:
    case core::Action::LoadConfig:
    case core::Action::None:
      break;
    }
    state = tr.next;
  }
}
