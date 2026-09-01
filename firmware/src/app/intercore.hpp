#pragma once
// Intercore -- the shared core0<->core1 contract.
//
// Core 1 owns the deterministic domain (PIO/DMA capture + the CPU pickup emitter + the
// math/reduce pipeline). Core 0 owns USB/flash/FSM. The ONLY data that crosses
// core1->core0 is reduced result rows -- core::SteppedRow (stepped) and
// core::RampRow (ramp/hold) -- handed over the two lock-free SPSC rings below; a
// handful of std::atomic flags coordinate run start/stop and the flash-write
// window. Everything here is SDK-free so it also compiles for the host unit tests.
//
// Concurrency contract:
//   * The atomics are plain aligned 32-bit / bool values -- safe for the
//     Cortex-M0+ single-copy-atomic loads/stores used by std::atomic here.
//   * `results` is an app::Spsc: core1 is the SOLE producer (push), core0 is the
//     SOLE consumer (pop). No other access pattern is safe (see spsc.hpp).
//   * Timebase64 (owned by Capture on core1) MUST NOT be read from core0 -- only
//     fully-reduced rows cross the boundary, never raw capture ticks.
#include "app/spsc.hpp"
#include "core/config.hpp"
#include "core/csv.hpp"

#include <atomic>
#include <cstddef>

namespace app {

// The ring only buffers rows in flight: core0 drains it every loop tick, so a
// handful of queued rows is the most it ever holds (a full sweep is ~200 rows,
// streamed through core0 into its SRAM buffer, never buffered here at once).
inline constexpr std::size_t kResultRingLen = cfg::kResultRingLen;

struct Intercore {
  core::Config config{};                // set by core0 before requesting a run
  std::atomic<bool> run_request{false}; // core0 -> core1: begin a run
  std::atomic<bool> run_active{false};  // core1: a run is in progress
  std::atomic<bool> capture_idle{
      true}; // core1: true when NOT actively capturing (core0 may write flash)
  std::atomic<bool> lost_sync{false}; // core1 -> core0: lost-sync flagged
  std::atomic<bool> run_done{false};  // core1 -> core0: run finished, all rows pushed
  // Live progress (core1 -> core0, diagnostics only): the rpm of the step in
  // progress and how many steps have completed. Lets core0 surface a heartbeat so a
  // stall is visible at the exact rpm it hung, instead of inferred from silence.
  std::atomic<std::uint32_t> cur_rpm{0};
  std::atomic<std::uint32_t> steps_done{0};
  Spsc<core::SteppedRow, kResultRingLen> results{};

  // Ramp run mode: a ramp emits a per-spark core::RampRow instead
  // of a per-step reduction. Its rows cross the same core1->core0 boundary over a
  // dedicated SPSC ring (core1 sole producer, core0 sole consumer). The per-spark
  // rate is much higher than the stepped rate, so core0 must drain this every loop
  // tick; a momentarily-full ring drops the overflow row (acceptable -- the raw
  // trajectory tolerates a dropped sample, and core0 flags truncation on its own
  // SRAM buffer if the whole ramp exceeds cfg::kMaxRampRows).
  Spsc<core::RampRow, kResultRingLen> ramp_results{};
  std::atomic<bool> ramp_truncated{false}; // core0 -> self: ramp buffer overflowed

  // HOLD run mode. All SWD-visible; atomics keep the Cortex-M0+
  // single-copy-atomic guarantee and prevent dead-store elimination of values the
  // firmware writes but never reads in-core (they are read externally over SWD).
  std::atomic<std::uint32_t> hold_target_rpm{0}; // core0/SWD -> core1: live target
  std::atomic<bool> stop_request{false};         // SWD/dev -> core1: stop a hold
  std::atomic<bool> debug_start{false};          // SWD/dev -> core0 Ready: start a run
  std::atomic<std::uint32_t> hold_heartbeat{0};  // core1 -> SWD: liveness counter
  // Cooperative park handshake for the HOLD BOOTSEL poll. core1 sets
  // park_request and waits for park_ack; core0 acks by spinning IRQs-off in RAM
  // (not touching flash) so core1 can float the QSPI CS to read BOOTSEL safely.
  // Deliberately NOT the SDK multicore_lockout: that is a single-victim primitive
  // (one shared request-id/mutex) and core0 is already the flash-write controller,
  // so making core0 a victim too corrupts the shared state and hangs the flush.
  std::atomic<bool> park_request{false}; // core1 -> core0: enter the RAM park spin
  std::atomic<bool> park_ack{false};     // core0 -> core1: parked (safe to float CS)
};

} // namespace app
