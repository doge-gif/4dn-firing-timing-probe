#include "app/core1_main.hpp"

#include "app/run_modes.hpp"
#include "core/angle.hpp"
#include "core/config.hpp"
#include "core/csv.hpp"
#include "core/skew.hpp"
#include "core/stats.hpp"
#include "core/tach.hpp"
#include "core/types.hpp"
#include "hal/capture.hpp"
#include "hal/gpio.hpp"
#include "hal/pins.hpp"
#include "hal/soft_pattern.hpp"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

// This is the deterministic domain. It mirrors the SAME reduction that the
// single-core self-test (firmware/hiltest/selftest.cpp) proved -- per RPM step
// reconfigure the pickup pattern, capture edges, bracket a spark time between
// captured pickup edges, interp_angle -> to_btdc -> Welford + median/min/max ->
// SteppedRow -- except the sparks now come from the real ignition-sense capture
// channels (GP16/GP17) instead of a synthetic time delay, and the reduced rows
// are pushed to core0 over the SPSC ring instead of printed.
//
// The pure reduction math here is host-unit-tested. The real-ignition spark path
// (bracketing captured ignition RISING edges against captured pickup edges,
// per-cylinder) can only be exercised on hardware with a real ignitor + wiring
// loom connected -- on the bench there are no ignition-sense edges, so that path
// is written to be structurally correct and heavily commented, and yields an
// empty (no-fire) reduction.

namespace app {

// The single shared core0<->core1 state block (declared extern in the header).
Intercore g_ic;

namespace {

// ---- Fixed hardware assignment ---------------------------------------------
// SoftPattern: core1 CPU bit-bang (GP18 as SIO output)              -> drives GP18
// Capture    : pio1 sm0/sm1/sm2/sm3    (+1 DMA each)                 -> reads pins
//   slot 0 = pickup self-capture GP18 (idle-LOW,  first edge RISING)
//   slot 1 = ignition sense    GP17 (idle-HIGH, first edge FALLING) -> cyl 0
//   slot 2 = ignition sense    GP16 (idle-HIGH, first edge FALLING) -> cyl 1
//   slot 3 = tach diagnostic   GP19 (idle-HIGH, first edge FALLING) -> reference
// Budget: pio0 entirely FREE (pattern gen moved to CPU); pio1 uses 4 SMs (FULL).
// DMA: 0 + 4 = 4 (of 12). The pickup pad is SIO-driven and pio1 reads its input
// synchronizer, so self-capture is unaffected (capture_program_init never calls
// pio_gpio_init). More capture channels could now reclaim any pio0 SM.
constexpr std::size_t kSlotPickup = 0;
constexpr std::size_t kSlotIgn0 = 1;
constexpr std::size_t kSlotIgn1 = 2;
constexpr std::size_t kSlotTach = 3;

// CaptureEvent.ch id for the pickup channel. Ignition channels are stamped with
// the cylinder index (0/1) so DwellPairer and cfg.tdc_ref_deg[] index directly;
// the pickup channel needs an id that never collides with a cylinder index.
constexpr std::uint8_t kPickupChId = 0xFF;
// CaptureEvent.ch id for the tach reference-diagnostic channel. Distinct from the
// pickup (0xFF) and cylinder indices (0/1) so routing never collides.
constexpr std::uint8_t kTachChannel = 0xFE;

// Capture counter is sysclk/2 = 62.5 MHz (see hal::kCaptureTicksPerCount), so a
// DELTA of two `ticks` in microseconds = delta / 62.5. Used only for dwell (the
// angle math needs no absolute rate -- interp_angle works in raw ticks).
constexpr float kCaptureCountsPerUs = cfg::kCaptureCountsPerUs;

// Rolling pickup-edge history depth. Sparks are almost always bracketable within
// the same drain() (pickup events are drained before ign events, so the history
// already covers the drain instant); this ring only needs to outlive a spark
// whose trailing pickup edge lands in the NEXT drain, plus the settle margin.
constexpr std::size_t kPkRingCap = 64;

// Per-cylinder caps for one step.
constexpr std::size_t kMaxSamples = 64;      // BTDC/dwell samples reduced per step
constexpr std::size_t kMaxPending = 8;       // sparks awaiting their trailing pickup edge
constexpr std::size_t kEventsPerDrain = 192; // CaptureEvent scratch per drain() call

// Fast "no fire at this rpm" detection (range scan): if NO spark has
// appeared within settle + this many revs, the ignitor is not firing at this rpm
// (below cranking / past redline). We record an empty row and move on rather than
// burning the full sample timeout -- important at low rpm where one rev is long.
constexpr std::uint32_t kNoFireMarginRevs = 6;

// Hard wall-time cap on a ramp run so a stall / non-terminating schedule can never
// hang core1 (self-recovering HIL-gate rule; see run_ramp). Generous vs a normal
// full-range ramp (~20 s at 500 rpm/s).
constexpr std::uint64_t kRampWallCapUs = 40'000'000; // 40 s

// One captured pickup edge: its time, its intra-rev wheel angle (kept SMALL, in
// [0,wheel)), and an integer rev index. bracket_pk reconstructs the monotonic
// spark angle in a b-relative frame from (intra, rev) so a long sweep's large rev
// count never costs float precision; to_btdc folds the result into [0,wheel). rev
// stays an exact integer (used for the settle window and the rev-boundary wrap).
struct PkEdge {
  std::uint64_t t = 0;
  float intra = 0.0f; // intra-rev wheel angle [0, wheel)
  std::uint32_t rev = 0;
};

// A spark (ignition RISING edge) awaiting bracketing pickup edges. dwell is the
// charge duration paired by DwellPairer at the spark instant (fall->rise).
struct PendingSpark {
  std::uint64_t t = 0;
  bool has_dwell = false;
  std::uint64_t dwell_ticks = 0;
};

// Per-step accumulator + reducer for one full RPM step. Not persisted across
// steps (constructed fresh each step) except through the pipeline's own fields.
struct Reducer {
  // Pickup history ring (see kPkRingCap). pk_total_ counts ALL pickup edges seen
  // this step; the valid window is the last min(pk_total_, kPkRingCap) entries.
  PkEdge pk_[kPkRingCap]{};
  std::uint32_t pk_total_ = 0;

  // Per-cylinder reduction state.
  core::Welford btdc_[core::kMaxCylinders]{};
  core::Welford dwell_[core::kMaxCylinders]{};
  float btdc_buf_[core::kMaxCylinders][kMaxSamples]{};
  std::uint32_t got_[core::kMaxCylinders]{};

  // Per-cylinder pending sparks (small; drained each pass).
  PendingSpark pend_[core::kMaxCylinders][kMaxPending]{};
  std::size_t npend_[core::kMaxCylinders]{};

  // One DwellPairer instance covers all cylinders (state is per ch internally).
  core::DwellPairer pairer_{};

  // ---- Tach REFERENCE diagnostics (system input latency + sync-lock RPM).
  // REFERENCE only -- never used to correct BTDC. On the bench there is no tach
  // signal (it exists only with the real ignitor), so no tach edges arrive and
  // these stay empty -> the row fields default to 0. This path is exercised only
  // with a real ignitor connected. See process_tach_edge below for the
  // edge-matching logic.
  std::uint64_t last_tooth0_t_ = 0; // last emitted tooth-0 RISING edge (from pickup)
  bool have_tooth0_ = false;
  core::TachDiag tach_{}; // per-step sync-lock RPM + input-latency accumulator

  // Geometry cache for this step.
  float intra_angle_[core::kMaxTeeth * 2]{}; // wheel angle of each edge in a rev
  std::size_t edges_per_rev_ = 0;
  float wheel_deg_ = 360.0f;
  std::uint32_t settle_revs_ = 0;
  std::uint32_t step_rev0_ = 0; // sync-rev index at the current step's entry
  std::uint32_t want_ = 0;
  std::uint32_t rpm_cmd_ = 0; // commanded rpm for this step (stamped into each row)
  std::uint8_t cyl_count_ = 1;
  const core::Config* cfg_ = nullptr;

  // Sync-gap tooth-0 alignment (self-healing; config-derived threshold). The
  // largest inter-edge gap is the sync gap; crossing it re-aligns intra_idx_ to 0
  // and bumps sync_rev_ every revolution, so a spurious startup edge or any single
  // miscount corrupts at most one rev instead of shifting the whole map. Replaces
  // the fragile "first captured edge == tooth-0, index == pk_total_" assumption
  // (which the raw capture proved wrong -- pk[0] is a spurious startup edge).
  std::uint64_t last_pk_t_ = 0;
  bool have_last_pk_ = false;
  float sync_thresh_deg_ = 0.0f;        // from cfg (build_intra_table)
  std::uint64_t sync_thresh_ticks_ = 0; // sync_thresh_deg_ scaled to this step's rpm
  std::size_t intra_idx_ = 0;           // edge index within the current rev
  std::uint32_t sync_rev_ = 0;          // revs since the first sync gap
  bool synced_ = false;                 // a sync gap has been seen -> angles valid

  // Per-channel capture-clock skew correction. Each capture SM's counter PAUSES
  // ~2.75 counts while handling an edge (mov/in/in/push don't decrement it), so a
  // channel's timestamps fall behind real time by k_cpe_ * (edges it has handled).
  // The pickup (8 edges/rev) drifts far faster than an ignition line (~2/rev), so
  // a spark's time vs a tooth's time -- compared ACROSS the two counters -- gains a
  // growing error over a run (the ATDC drift we chased). Fix: add each channel's
  // accumulated pause back, putting all channels on one pause-free timebase.
  // Per-channel edge counts feed the fixed-K correction (cfg::kCounterPauseCountsPerEdge).
  // ch idx: 0 pickup, 1 cyl0, 2 cyl1, 3 tach.
  std::uint64_t ch_edges_[4]{};
};

// Map a CaptureEvent.ch id to a ch_edges_ index (0 pickup, 1 cyl0, 2 cyl1, 3 tach),
// or -1 if it is not a corrected channel.
inline int skew_ch_idx(std::uint8_t ch) {
  if (ch == kPickupChId)
    return 0;
  if (ch == kTachChannel)
    return 3;
  if (ch < 2)
    return static_cast<int>(ch) + 1;
  return -1;
}

// Config-derived sync-gap threshold (degrees). The pickup's largest inter-EDGE
// angular gap is the sync gap (SRV250: 170 deg, tooth-3 trailing -> tooth-0
// leading); the largest NON-sync gap is the widest inter-tooth gap (50 deg). Any
// gap wider than the midpoint of those two is unambiguously the sync gap. Derived
// purely from cfg.teeth -- nothing hardcoded. 0 if geometry is degenerate (which
// disables sync detection -> the pipeline simply never brackets, never lies).
float sync_thresh_deg_from_cfg(const core::Config& cfg) {
  std::size_t tc = cfg.tooth_count > core::kMaxTeeth ? core::kMaxTeeth : cfg.tooth_count;
  if (tc < 2)
    return 0.0f;
  const float wheel = cfg.wheel_deg > 0.0f ? cfg.wheel_deg : 360.0f;
  float edges[core::kMaxTeeth * 2];
  std::size_t n = 0;
  for (std::size_t i = 0; i < tc; ++i) {
    edges[n++] = cfg.teeth[i].leading_deg;
    edges[n++] = cfg.teeth[i].trailing_deg;
  }
  float largest = 0.0f, second = 0.0f;
  for (std::size_t i = 0; i < n; ++i) {
    const float g = (i + 1 < n) ? (edges[i + 1] - edges[i]) : (edges[0] + wheel - edges[i]);
    if (g > largest) {
      second = largest;
      largest = g;
    } else if (g > second) {
      second = g;
    }
  }
  return 0.5f * (largest + second);
}

// Convert a sync-gap threshold in wheel-degrees to capture-count ticks at `rpm`.
// Gaps scale with rpm; the threshold tracks so detection is rpm-independent in
// effect. Lenient by construction (midpoint of the two largest gaps), so it stays
// correct even if the actual rpm drifts from commanded within a rev. sync_deg and
// wheel_deg are both wheel-frame degrees; the proper fix is a strong Deg units type
// threaded through the angle math (deferred as a larger change), so a swap is for now
// guarded only by the named call site.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
std::uint64_t sync_thresh_ticks(float sync_deg, float wheel_deg, std::uint32_t rpm) {
  if (sync_deg <= 0.0f || rpm == 0)
    return 0;
  const double rev_ticks = (60.0e6 / double(rpm)) * double(kCaptureCountsPerUs);
  const double wheel = wheel_deg > 0.0f ? double(wheel_deg) : 360.0;
  return static_cast<std::uint64_t>(double(sync_deg) / wheel * rev_ticks);
}

// Flatten the config's tooth spans into the per-edge intra-revolution angle table
// (leading = rising, trailing = falling), the same order the pickup emits edges.
void build_intra_table(Reducer& rd, const core::Config& cfg) {
  rd.cfg_ = &cfg;
  rd.wheel_deg_ = cfg.wheel_deg;
  rd.cyl_count_ = cfg.cyl_count ? cfg.cyl_count : 1;
  std::size_t tc = cfg.tooth_count;
  if (tc > core::kMaxTeeth)
    tc = core::kMaxTeeth;
  std::size_t n = 0;
  for (std::size_t i = 0; i < tc; ++i) {
    rd.intra_angle_[n++] = cfg.teeth[i].leading_deg;
    rd.intra_angle_[n++] = cfg.teeth[i].trailing_deg;
  }
  rd.edges_per_rev_ = n;
  rd.sync_thresh_deg_ = sync_thresh_deg_from_cfg(cfg); // per-rpm ticks set in run_step
}

// Find the bracketing pair (before,after) around spark_t in a pickup-edge ring
// window and return the spark angle IN A b-RELATIVE FRAME (b's rev == 0), or NaN
// if not yet bracketable (the trailing edge has not been captured, or the leading
// edge aged out). The b-relative frame keeps the interpolation in small angles
// regardless of how many revs the run has accumulated; the caller's to_btdc folds
// mod wheel, so the dropped b.rev*wheel offset does not change the result. `a` is
// b's rev or exactly one ahead, so a.intra + (a.rev-b.rev)*wheel is monotonic vs
// b.intra. Operates on the raw ring + running total so BOTH the stepped Reducer
// and the ramp RampReducer share the exact same math. Hot path. The three numeric
// params (pk_total, wheel_deg, spark_t) are distinct quantities; the proper guard is
// the deferred Ticks/Deg units layer, so for now the two callers pass named values.
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
float __time_critical_func(bracket_pk)(const PkEdge* pk, std::uint32_t pk_total, float wheel_deg,
                                       std::uint64_t spark_t, std::uint32_t& rev_out) {
  const std::uint32_t valid = (pk_total < kPkRingCap) ? pk_total : kPkRingCap;
  if (valid < 2)
    return NAN;
  const std::uint32_t oldest = pk_total - valid; // absolute index of oldest kept
  for (std::uint32_t k = oldest; k + 1 < pk_total; ++k) {
    const PkEdge& b = pk[k % kPkRingCap];
    const PkEdge& a = pk[(k + 1) % kPkRingCap];
    if (b.t <= spark_t && spark_t < a.t) {
      core::EdgeRef before{b.t, b.intra};
      core::EdgeRef after{a.t, a.intra + float(a.rev - b.rev) * wheel_deg};
      rev_out = b.rev;
      return core::interp_angle(spark_t, before, after);
    }
  }
  return NAN;
}

// Stepped-Reducer adapter over bracket_pk (kept for call-site readability).
float __time_critical_func(bracket_angle)(const Reducer& rd, std::uint64_t spark_t,
                                          std::uint32_t& rev_out) {
  return bracket_pk(rd.pk_, rd.pk_total_, rd.wheel_deg_, spark_t, rev_out);
}

// Try to resolve every pending spark for one cylinder against the current pickup
// history; on success feed Welford + the median buffer and drop the spark.
// Sparks older than the retained pickup window (should not happen if the ring is
// sized right) are also dropped so pending never wedges. Hot path.
void __time_critical_func(resolve_pending)(Reducer& rd, std::uint8_t cyl) {
  std::size_t w = 0; // compaction write index for still-pending sparks
  const std::uint32_t valid = (rd.pk_total_ < kPkRingCap) ? rd.pk_total_ : kPkRingCap;
  const std::uint64_t oldest_t = (valid > 0) ? rd.pk_[(rd.pk_total_ - valid) % kPkRingCap].t : 0;
  for (std::size_t i = 0; i < rd.npend_[cyl]; ++i) {
    PendingSpark& s = rd.pend_[cyl][i];
    std::uint32_t rev = 0;
    float ang = bracket_angle(rd, s.t, rev);
    if (std::isnan(ang)) {
      // Not bracketable. If the spark predates the retained window it can never
      // be resolved -> drop it. Otherwise keep it for the next drain.
      if (valid >= 2 && s.t < oldest_t) {
        continue; // drop (compacted out)
      }
      rd.pend_[cyl][w++] = s;
      continue;
    }
    // Bracketed. Honor the settle window (skip the first settle_revs revs OF THIS
    // STEP -- rev is a continuous count across the whole sweep, so offset by the
    // step's entry rev), then accumulate BTDC + dwell up to the wanted sample count.
    if (rev >= rd.step_rev0_ + rd.settle_revs_ && rd.got_[cyl] < rd.want_) {
      const float tdc = rd.cfg_->tdc_ref_deg[cyl];
      const float btdc = core::to_btdc(ang, tdc, rd.wheel_deg_);
      rd.btdc_[cyl].add(btdc);
      if (rd.got_[cyl] < kMaxSamples)
        rd.btdc_buf_[cyl][rd.got_[cyl]] = btdc;
      if (s.has_dwell)
        rd.dwell_[cyl].add(float(s.dwell_ticks) / kCaptureCountsPerUs);
      ++rd.got_[cyl];
    }
    // Resolved -> dropped (compacted out) regardless of settle/want gating.
  }
  rd.npend_[cyl] = w;
}

// Reduce one tach reference edge (REFERENCE diagnostics only; never corrects
// BTDC). The DUT tach emits one RISING edge per crank rev aligned to tooth-0's
// rising edge, so:
//   * system latency = tach_rising - the emitted tooth-0 rising edge (a composite
//     input-path delay; clean-edge/bench value), accumulated into a Welford;
//   * sync-lock RPM comes from the interval between consecutive tach edges.
// Exercised only with a real ignitor connected: the tach<->tooth-0 pairing below assumes the most
// recent emitted tooth-0 rising edge (tracked from the pickup self-capture)
// precedes this tach edge within the same drain window. That holds because
// drain() returns the pickup slot before the tach slot, so tooth-0 for this rev
// is already recorded. On the bench no tach edge ever arrives, so this path is
// simply never exercised until a real ignitor is present. Hot path -> RAM.
void __time_critical_func(process_tach_edge)(Reducer& rd, std::uint64_t ticks, bool rising) {
  if (!rising) {
    return; // only the rising reference edge (once per rev) is meaningful
  }
  // Sync-lock RPM (rising-to-rising interval) + input-path latency vs the emitted
  // tooth-0 rising edge (both skew-corrected). Accumulated per step; begin_step
  // resets the accumulator so these track the current rpm, not the whole sweep.
  rd.tach_.on_rising(ticks, rd.have_tooth0_, rd.last_tooth0_t_);
}

// Process one drained batch of CaptureEvents: append pickup edges to the history,
// pair + queue ignition sparks, then resolve pending sparks. Because drain()
// returns each channel's events contiguously and time-ordered (pickup slot first,
// then ign slots), the pickup history is fully up to date before any spark in the
// same batch is examined. Hot path -> RAM-resident.
void __time_critical_func(process_batch)(Reducer& rd, const hal::CaptureEvent* ev, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    const hal::CaptureEvent& e = ev[i];
    // Capture-clock skew correction (see Reducer): add back this channel's
    // accumulated counter-pause so pickup and ignition timestamps live in ONE
    // pause-free timebase -- otherwise a spark (ignition channel) compared to a
    // tooth (pickup channel) drifts ATDC over a run. Shared with the ramp path via
    // core::skew_corrected_ticks (adds cfg::kCounterPauseCountsPerEdge per handled edge).
    const int ci = skew_ch_idx(e.ch);
    std::uint64_t ct = e.ticks;
    if (ci >= 0) {
      ct = core::skew_corrected_ticks(e.ticks, rd.ch_edges_[ci]);
      ++rd.ch_edges_[ci];
    }
    if (e.ch == kPickupChId) {
      // SYNC-GAP alignment (self-healing): the 170-deg sync gap (tooth-3 trailing
      // -> tooth-0 leading) is the largest inter-edge gap; crossing it re-aligns
      // intra_idx_ to 0 and bumps sync_rev_, re-deriving tooth-0 every revolution.
      const bool is_sync = rd.have_last_pk_ && rd.sync_thresh_ticks_ &&
                           (ct - rd.last_pk_t_) >= rd.sync_thresh_ticks_;
      rd.last_pk_t_ = ct;
      rd.have_last_pk_ = true;
      if (is_sync) {
        rd.intra_idx_ = 0;
        if (rd.synced_)
          ++rd.sync_rev_; // a full rev completed since the previous sync
        rd.synced_ = true;
      } else if (rd.synced_) {
        ++rd.intra_idx_;
      }
      PkEdge& slot = rd.pk_[rd.pk_total_ % kPkRingCap];
      slot.t = ct;
      slot.rev = rd.sync_rev_;
      if (rd.synced_ && rd.intra_idx_ < rd.edges_per_rev_) {
        slot.intra = rd.intra_angle_[rd.intra_idx_]; // small; rev kept separately
        // Tooth-0's RISING (leading) edge is intra index 0 -> tach latency ref.
        if (rd.intra_idx_ == 0 && e.rising) {
          rd.last_tooth0_t_ = ct;
          rd.have_tooth0_ = true;
        }
      } else {
        slot.intra = NAN; // not yet synced, or a glitch overran the rev -> unbracketable
      }
      ++rd.pk_total_;
      continue;
    }
    // Tach reference-diagnostic channel (REFERENCE only; never corrects BTDC).
    if (e.ch == kTachChannel) {
      process_tach_edge(rd, ct, e.rising);
      continue;
    }
    // Ignition channel: e.ch is the cylinder index.
    if (e.ch >= rd.cyl_count_ || e.ch >= core::kMaxCylinders)
      continue;
    // Predictive dwell: a FALLING edge (charge start) may legitimately
    // precede its tooth; DwellPairer records fall->rise, so falling edges are
    // never anomalous. Only the RISING edge (spark) is bracketed against pickup.
    core::DwellResult dr = rd.pairer_.on_edge(e.ch, e.rising, ct);
    if (e.rising) {
      if (rd.npend_[e.ch] < kMaxPending) {
        PendingSpark& s = rd.pend_[e.ch][rd.npend_[e.ch]++];
        s.t = ct;
        s.has_dwell = dr.has_dwell;
        s.dwell_ticks = dr.dwell_ticks;
      }
      // else: pending queue saturated (should not happen at realistic edge rates);
      // the spark is simply not counted this pass.
    }
  }
  for (std::uint8_t c = 0; c < rd.cyl_count_ && c < core::kMaxCylinders; ++c)
    resolve_pending(rd, c);
}

// True once every active cylinder has gathered its wanted sample count.
bool step_complete(const Reducer& rd) {
  for (std::uint8_t c = 0; c < rd.cyl_count_ && c < core::kMaxCylinders; ++c) {
    if (rd.got_[c] < rd.want_)
      return false;
  }
  return true;
}

// Reduce one cylinder's accumulated samples into a SteppedRow (Welford mean/std +
// median/min/max on the sample buffer + dwell mean/std). Not on the tightest hot
// path (runs once per cyl per step) so it stays in flash. The step's commanded rpm
// lives in rd.rpm_cmd_ (set once per step), so it is not re-passed per cylinder.
core::SteppedRow reduce_row(Reducer& rd, std::uint8_t cyl) {
  core::SteppedRow row{};
  row.rpm_cmd = rd.rpm_cmd_;
  row.cyl = cyl;
  const std::uint32_t got = rd.got_[cyl];
  row.n = got;
  if (got == 0) {
    // No spark bracketed at this rpm (below cranking / past redline). Emit a
    // marker row: n=0 with timing fields left at 0, still carrying the tach
    // reference diagnostics. Callers/consumers read n==0 as "no fire here".
    row.mean_sys_latency_us = rd.tach_.latency_us_mean();
    row.tach_rpm = rd.tach_.rpm();
    return row;
  }
  const std::size_t nbuf = (got < kMaxSamples) ? got : kMaxSamples;
  core::MedianResult mm = core::median_minmax(rd.btdc_buf_[cyl], nbuf);
  row.mean_btdc_deg = rd.btdc_[cyl].mean();
  row.median_btdc_deg = mm.median;
  row.stddev_btdc_deg = rd.btdc_[cyl].stddev_sample();
  row.min_btdc_deg = mm.min;
  row.max_btdc_deg = mm.max;
  row.mean_dwell_us = rd.dwell_[cyl].mean();
  row.stddev_dwell_us = rd.dwell_[cyl].stddev_sample();
  // Tach REFERENCE diagnostics (per step, replicated onto every cylinder's row;
  // never used to correct BTDC). On the bench there is no tach signal, so both
  // Welfords are empty -> mean()==0 and tach_rpm_from_cycle_ticks(0)==0, i.e. the
  // columns read 0.00 (expected). This diagnostic is exercised only with a real ignitor connected.
  row.mean_sys_latency_us = rd.tach_.latency_us_mean();
  row.tach_rpm = rd.tach_.rpm();
  return row;
}

// Bounded drain of any stale/backlog captured edges so a fresh run starts clean.
// Bounded by wall time (not "drain to empty"): the ignitor may keep driving its
// ign/tach outputs indefinitely, so draining to empty could never return -- 20 ms
// is ample to clear the finite pickup backlog before emission begins.
void drain_stale(hal::Capture& capt) {
  hal::CaptureEvent scratch[kEventsPerDrain];
  const std::uint64_t deadline = time_us_64() + 20000u; // 20 ms cap
  while (capt.drain(scratch, kEventsPerDrain) > 0) {
    if (time_us_64() >= deadline)
      break;
  }
}

// Reset a Reducer's PER-STEP reduction state (Welford, sample buffers, sample
// counts, pending sparks) while PRESERVING the continuous pickup-edge history and
// SYNC-GAP alignment (pk_/pk_total_/intra_idx_/sync_rev_). The pattern runs
// uninterrupted across the whole sweep and re-syncs every rev, so alignment is
// continuous; step_rev0_ records the sync-rev at step entry so the settle window is
// measured from the step, not the run.
void begin_step(Reducer& rd) {
  for (std::uint8_t c = 0; c < core::kMaxCylinders; ++c) {
    rd.btdc_[c] = core::Welford{};
    rd.dwell_[c] = core::Welford{};
    rd.got_[c] = 0;
    rd.npend_[c] = 0; // drop transition-rev sparks from the previous rpm
  }
  // Tach REFERENCE diagnostics are per-step too: without this reset they
  // accumulate across the WHOLE sweep, so tach_rpm reports the running (~harmonic)
  // mean of every prior rpm instead of the current step, and mean_sys_latency_us
  // is likewise sweep-cumulative (also drops the interval straddling the step).
  rd.tach_.reset();
  rd.step_rev0_ = rd.sync_rev_;
}

// Run one RPM step against the SHARED sweep Reducer `rd`: gather samples for every
// active cylinder, or detect that the ignitor is not firing at this rpm and leave
// an empty (n==0) reduction. The CPU pattern generator `pg` is already running
// (started once by run_sweep) and has been told this step's rpm; run_step only
// drains + measures (emission runs from pg's timer-alarm IRQ). Returns true iff a
// spark was seen.
bool run_step(hal::SoftPattern& pg, hal::Capture& capt, const app::SteppedSchedule& sched,
              Reducer& rd, std::uint32_t rpm) {
  begin_step(rd);
  rd.rpm_cmd_ = rpm; // stamped into every row this step (see reduce_row)
  rd.sync_thresh_ticks_ = sync_thresh_ticks(rd.sync_thresh_deg_, rd.wheel_deg_, rpm);
  const float rev_us = 60.0e6f / float(rpm ? rpm : 1);
  // Settle in revs (config sweep_settle_revs). Emission is continuous across the whole
  // sweep, so the ignitor never loses sync between steps -- only its rpm estimate
  // tracks the step change -- and a few revs suffice (no wall-time settle needed).
  rd.settle_revs_ = sched.revs_to_settle();
  rd.want_ = sched.samples_wanted();
  if (rd.want_ > kMaxSamples)
    rd.want_ = kMaxSamples;

  // Two deadlines (range scan), scaled by the revolution period:
  //  * no_fire: if NO spark has appeared within settle + kNoFireMarginRevs revs,
  //    the ignitor is not firing at this rpm (below cranking / past redline) --
  //    bail fast instead of burning the full window (one rev is long at cranking).
  //  * sample: once firing, the generous window to gather sweep_samples; if it
  //    cannot finish we keep whatever samples we got.
  // Either way the SWEEP CONTINUES (a non-firing rpm is data, not an abort -- it
  // maps the cranking/redline boundary).
  const std::uint64_t t_start = time_us_64();
  const std::uint64_t no_fire_deadline =
      t_start + std::uint64_t(float(rd.settle_revs_ + kNoFireMarginRevs) * rev_us) + 5000u;
  const std::uint32_t budget_revs = rd.settle_revs_ + rd.want_ + 8;
  const std::uint64_t sample_deadline =
      t_start + std::uint64_t(float(budget_revs) * rev_us * 2.0f) + 5000u;

  hal::CaptureEvent evs[kEventsPerDrain];
  bool saw_spark = false;
  while (!step_complete(rd)) {
    std::size_t n = capt.drain(evs, kEventsPerDrain);
    if (n > 0) {
      for (std::size_t i = 0; i < n; ++i) {
        if (evs[i].ch < rd.cyl_count_ && evs[i].rising)
          saw_spark = true; // an ignition-channel RISING edge == a real spark
      }
      process_batch(rd, evs, n);
    }
    const std::uint64_t t = time_us_64();
    if (!saw_spark && t >= no_fire_deadline)
      break; // never fired at this rpm -> empty row and move on
    if (t >= sample_deadline)
      break;               // fired but could not finish the sample count -> take what we have
    tight_loop_contents(); // busy-wait (no sleep); emission runs from the timer-alarm IRQ
  }

  // Reduce + hand every active cylinder's row to core0 (n==0 == no fire here).
  for (std::uint8_t c = 0; c < rd.cyl_count_ && c < core::kMaxCylinders; ++c) {
    core::SteppedRow row = reduce_row(rd, c);
    (void)g_ic.results.push(row); // best-effort; core0 drains continuously
  }
  return saw_spark;
}

// Execute a full stepped sweep from the requested config. The CPU pattern
// generator emits CONTINUOUSLY for the whole sweep (started once here, rpm changed
// per step) -- no stop/start, so no reconfigure glitch and the ignitor never loses
// sync. One shared Reducer keeps the pickup-edge count aligned across all steps.
void run_sweep(hal::SoftPattern& pg, hal::Capture& capt) {
  const core::Config cfg = g_ic.config; // snapshot: core0 set it before requesting
  capt.start();
  drain_stale(capt); // clear any backlog so the first emitted edge is pk_total_ 0
  g_ic.capture_idle.store(false, std::memory_order_release); // measuring for the whole sweep

  Reducer rd;
  build_intra_table(rd, cfg);

  g_ic.steps_done.store(0, std::memory_order_relaxed);
  app::SteppedSchedule sched(cfg);
  std::uint32_t rpm0 = sched.current_rpm();
  pg.start(cfg, rpm0 ? rpm0 : 1, time_us_64()); // begin continuous emission at tooth-0

  // Run EVERY step to completion. A non-firing rpm no longer aborts the sweep --
  // run_step leaves an empty row and we continue, mapping exactly where the ignitor
  // starts and stops firing (cranking / redline edges).
  while (!sched.done()) {
    const std::uint32_t rpm = sched.current_rpm();
    g_ic.cur_rpm.store(rpm, std::memory_order_relaxed); // heartbeat: step in progress
    pg.set_rpm(rpm ? rpm : 1);                          // applied at the next rev boundary
    (void)run_step(pg, capt, sched, rd, rpm);
    g_ic.steps_done.fetch_add(1, std::memory_order_relaxed);
    sched.advance_step();
  }

  pg.idle_low();
  capt.stop();
  g_ic.capture_idle.store(true, std::memory_order_release);
}

// ============================ RAMP RUN MODE ================================
// Instead of holding each RPM and reducing to one row, a ramp
// continuously integrates RPM and emits ONE RampRow PER SPARK (no Welford, no
// median -- a median over a moving trajectory is meaningless). Each spark's row
// carries the commanded RPM at that instant, the measured revolution period, the
// °BTDC (same bracket+interp+to_btdc math as stepped), and the paired dwell.

// A spark awaiting its trailing pickup edge, plus the commanded RPM stamped at
// the moment the spark (ignition RISING edge) was captured.
struct RampPending {
  std::uint64_t t = 0;
  bool has_dwell = false;
  std::uint64_t dwell_ticks = 0;
  std::uint32_t rpm_cmd = 0;
};

// Per-spark accumulator for a ramp. Mirrors Reducer's pickup-history + dwell +
// pending machinery, but emits raw rows instead of reducing. No settle gating
// (every bracketable spark is emitted) and no tach diagnostics (RampRow
// has no tach columns; the tach reference path is a stepped-only diagnostic).
struct RampReducer {
  PkEdge pk_[kPkRingCap]{};
  std::uint32_t pk_total_ = 0;

  core::DwellPairer pairer_{};
  RampPending pend_[core::kMaxCylinders][kMaxPending]{};
  std::size_t npend_[core::kMaxCylinders]{};

  float intra_angle_[core::kMaxTeeth * 2]{};
  std::size_t edges_per_rev_ = 0;
  float wheel_deg_ = 360.0f;
  std::uint8_t cyl_count_ = 1;
  const core::Config* cfg_ = nullptr;

  // Revolution period from consecutive tooth-0 (leading) pickup edges, measured
  // continuously across the WHOLE ramp: the CPU emitter (SoftPattern) never stops
  // -- it just retimes at rev boundaries -- so there is no stop/start gap to
  // straddle and nothing to reset here (unlike the old stop/load/start staircase).
  std::uint64_t last_tooth0_t_ = 0;
  bool have_tooth0_ = false;
  std::uint32_t cur_rev_period_us_ = 0;

  // Sync-gap tooth-0 alignment (self-healing; see Reducer). sync_thresh_ticks_ is
  // refreshed each loop from the commanded rpm (the ramp's rpm moves continuously).
  std::uint64_t last_pk_t_ = 0;
  bool have_last_pk_ = false;
  float sync_thresh_deg_ = 0.0f;
  std::uint64_t sync_thresh_ticks_ = 0;
  std::size_t intra_idx_ = 0;
  std::uint32_t sync_rev_ = 0;
  bool synced_ = false;

  // Commanded RPM (rounded) from the RampSchedule, refreshed each loop and
  // stamped onto every spark captured until the next refresh.
  std::uint32_t cur_rpm_cmd_ = 0;

  // Per-channel capture-clock skew correction (see core/skew.hpp and the stepped
  // Reducer). ch idx: 0 pickup, 1 cyl0, 2 cyl1, 3 tach.
  std::uint64_t ch_edges_[4]{};
};

// Flatten tooth spans into the per-edge intra-rev angle table (leading,trailing
// per tooth), in emitted-edge order. Cold (once per run).
void build_ramp_intra(RampReducer& rd, const core::Config& cfg) {
  rd.cfg_ = &cfg;
  rd.wheel_deg_ = cfg.wheel_deg;
  rd.cyl_count_ = cfg.cyl_count ? cfg.cyl_count : 1;
  std::size_t tc = cfg.tooth_count;
  if (tc > core::kMaxTeeth)
    tc = core::kMaxTeeth;
  std::size_t n = 0;
  for (std::size_t i = 0; i < tc; ++i) {
    rd.intra_angle_[n++] = cfg.teeth[i].leading_deg;
    rd.intra_angle_[n++] = cfg.teeth[i].trailing_deg;
  }
  rd.edges_per_rev_ = n;
  rd.sync_thresh_deg_ = sync_thresh_deg_from_cfg(cfg); // ticks refreshed per loop in run_ramp
}

// Resolve every pending spark for one cylinder against the current pickup
// history; on success emit a per-spark RampRow to core0 and drop the spark.
// Best-effort push: a momentarily-full ring drops the row (core0 tolerates it).
// Hot path -> RAM.
void __time_critical_func(resolve_ramp)(RampReducer& rd, std::uint8_t cyl) {
  std::size_t w = 0; // compaction write index for still-pending sparks
  const std::uint32_t valid = (rd.pk_total_ < kPkRingCap) ? rd.pk_total_ : kPkRingCap;
  const std::uint64_t oldest_t = (valid > 0) ? rd.pk_[(rd.pk_total_ - valid) % kPkRingCap].t : 0;
  for (std::size_t i = 0; i < rd.npend_[cyl]; ++i) {
    RampPending& s = rd.pend_[cyl][i];
    std::uint32_t rev = 0;
    float ang = bracket_pk(rd.pk_, rd.pk_total_, rd.wheel_deg_, s.t, rev);
    if (std::isnan(ang)) {
      if (valid >= 2 && s.t < oldest_t)
        continue; // aged out -> drop (compacted)
      rd.pend_[cyl][w++] = s;
      continue;
    }
    core::RampRow row{};
    row.rpm_cmd = s.rpm_cmd;
    row.cyl = cyl;
    row.rev_period_us = rd.cur_rev_period_us_;
    row.btdc_deg = core::to_btdc(ang, rd.cfg_->tdc_ref_deg[cyl], rd.wheel_deg_);
    row.dwell_us = s.has_dwell ? (float(s.dwell_ticks) / kCaptureCountsPerUs) : 0.0f;
    (void)g_ic.ramp_results.push(row); // best-effort; overflow drops (see intercore.hpp)
    // Resolved -> dropped (compacted out).
  }
  rd.npend_[cyl] = w;
}

// Process one drained batch for the ramp: append pickup edges (updating the
// tooth-0 rev-period measurement), pair+queue ignition sparks with the current
// commanded RPM, then resolve. Same drain ordering guarantees as process_batch.
void __time_critical_func(process_batch_ramp)(RampReducer& rd, const hal::CaptureEvent* ev,
                                              std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    const hal::CaptureEvent& e = ev[i];
    // Capture-clock skew: put every channel on one pause-free timebase BEFORE use,
    // exactly as process_batch does. `ct` replaces raw e.ticks everywhere below.
    const int ci = skew_ch_idx(e.ch);
    std::uint64_t ct = e.ticks;
    if (ci >= 0) {
      ct = core::skew_corrected_ticks(e.ticks, rd.ch_edges_[ci]);
      ++rd.ch_edges_[ci];
    }
    if (e.ch == kPickupChId) {
      // Sync-gap alignment (self-healing; same as the stepped Reducer).
      const bool is_sync = rd.have_last_pk_ && rd.sync_thresh_ticks_ &&
                           (ct - rd.last_pk_t_) >= rd.sync_thresh_ticks_;
      rd.last_pk_t_ = ct;
      rd.have_last_pk_ = true;
      if (is_sync) {
        rd.intra_idx_ = 0;
        if (rd.synced_)
          ++rd.sync_rev_;
        rd.synced_ = true;
      } else if (rd.synced_) {
        ++rd.intra_idx_;
      }
      PkEdge& slot = rd.pk_[rd.pk_total_ % kPkRingCap];
      slot.t = ct;
      slot.rev = rd.sync_rev_;
      if (rd.synced_ && rd.intra_idx_ < rd.edges_per_rev_) {
        slot.intra = rd.intra_angle_[rd.intra_idx_];
        // Tooth-0 leading edge (intra index 0, rising) -> revolution period source.
        if (rd.intra_idx_ == 0 && e.rising) {
          if (rd.have_tooth0_ && ct > rd.last_tooth0_t_) {
            const std::uint64_t d = ct - rd.last_tooth0_t_;
            rd.cur_rev_period_us_ = static_cast<std::uint32_t>(float(d) / kCaptureCountsPerUs);
          }
          rd.last_tooth0_t_ = ct;
          rd.have_tooth0_ = true;
        }
      } else {
        slot.intra = NAN; // not yet synced, or a glitch overran the rev
      }
      ++rd.pk_total_;
      continue;
    }
    // Tach reference channel: NOT used in ramp mode (RampRow has no tach columns).
    if (e.ch == kTachChannel)
      continue;
    if (e.ch >= rd.cyl_count_ || e.ch >= core::kMaxCylinders)
      continue;
    core::DwellResult dr = rd.pairer_.on_edge(e.ch, e.rising, ct);
    if (e.rising) {
      if (rd.npend_[e.ch] < kMaxPending) {
        RampPending& s = rd.pend_[e.ch][rd.npend_[e.ch]++];
        s.t = ct;
        s.has_dwell = dr.has_dwell;
        s.dwell_ticks = dr.dwell_ticks;
        s.rpm_cmd = rd.cur_rpm_cmd_;
      }
    }
  }
  for (std::uint8_t c = 0; c < rd.cyl_count_ && c < core::kMaxCylinders; ++c)
    resolve_ramp(rd, c);
}

// Execute a full ramp (up or down) from the requested config. capture_idle stays
// FALSE for the whole ramp (core0 must not write flash mid-ramp); it is set true
// only at the end. The pickup pattern is loaded ONCE at the ramp's starting RPM
// and thereafter the commanded RPM is updated every loop pass via
// hal::SoftPattern::set_rpm -- the CPU emitter simply changes its rev period at the
// next rev boundary, so pickup emission never stops and never resets phase. That
// keeps the pickup rev-counting / tooth-0 tracking / pending-spark bracketing in
// RampReducer continuous across the WHOLE ramp -- there is no per-segment reset.
void run_ramp(hal::SoftPattern& pg, hal::Capture& capt, bool up) {
  const core::Config cfg = g_ic.config; // snapshot: core0 set it before requesting
  capt.start();

  app::RampSchedule sched(cfg, up);
  RampReducer rd;
  build_ramp_intra(rd, cfg);

  // One-time start at the ramp's initial RPM: drain stale captured edges so the
  // first emitted pickup edge is captured as index 0 (tooth-0 leading -- see
  // process_batch_ramp's count-based alignment), then begin continuous emission.
  std::uint32_t rpm = static_cast<std::uint32_t>(std::lroundf(sched.current_rpm()));
  if (rpm == 0)
    rpm = 1;
  drain_stale(capt);
  pg.start(cfg, rpm, time_us_64());
  rd.cur_rpm_cmd_ = rpm;
  rd.sync_thresh_ticks_ = sync_thresh_ticks(rd.sync_thresh_deg_, rd.wheel_deg_, rpm);

  // Actively capturing/measuring for the whole ramp: hold off core0 flash writes.
  g_ic.capture_idle.store(false, std::memory_order_release);

  hal::CaptureEvent evs[kEventsPerDrain];
  std::uint64_t prev_us = time_us_64();
  std::uint64_t last_pickup_us = prev_us;
  const std::uint64_t loop_start_us = prev_us;

  for (;;) {
    // 1. Integrate the schedule by real elapsed wall time (SDK clock -- schedule
    //    integration only, NOT the PIO capture timebase).
    const std::uint64_t now_us = time_us_64();
    // Wall-time cap: a hard upper bound on the ramp so a stall (or a schedule that
    // never reaches its bound) can never hang core1 forever -- it exits and core0
    // flushes whatever was collected (self-recovering, per the HIL-gate rule).
    if (now_us - loop_start_us > kRampWallCapUs)
      break;
    const float dt_s = float(now_us - prev_us) / 1.0e6f;
    prev_us = now_us;
    sched.tick(dt_s);
    rd.cur_rpm_cmd_ = static_cast<std::uint32_t>(std::lroundf(sched.current_rpm()));

    // 2. Drain + process (stamps sparks with the just-refreshed commanded RPM).
    const std::size_t nn = capt.drain(evs, kEventsPerDrain);
    if (nn > 0) {
      for (std::size_t i = 0; i < nn; ++i) {
        if (evs[i].ch == kPickupChId) {
          last_pickup_us = now_us;
          break;
        }
      }
      process_batch_ramp(rd, evs, nn);
    }

    // 3. Continuous rpm update: whenever the commanded RPM has moved, tell the
    //    emitter -- it retimes at the next rev boundary (no stop/start, no phase
    //    reset). This runs every loop pass, making the ramp a true continuous sweep.
    const std::uint32_t rpm_cmd = rd.cur_rpm_cmd_ ? rd.cur_rpm_cmd_ : 1;
    pg.set_rpm(rpm_cmd);
    rd.sync_thresh_ticks_ = sync_thresh_ticks(rd.sync_thresh_deg_, rd.wheel_deg_, rpm_cmd);

    // 4. Lost-sync watchdog: no pickup edges within a few revs of the
    //    current commanded period -> the pickup/spark stream is dead/desynced.
    const float rev_us = 60.0e6f / float(rpm_cmd);
    const std::uint64_t pk_timeout = std::uint64_t(rev_us * 16.0f) + 5000u;
    if (now_us - last_pickup_us > pk_timeout) {
      g_ic.lost_sync.store(true, std::memory_order_release);
      break;
    }

    if (sched.done())
      break;
    tight_loop_contents(); // busy-wait (no sleep); emission runs from the timer-alarm IRQ
  }

  // Final drain: flush late captures + resolve trailing sparks (bounded passes).
  for (int pass = 0; pass < 8; ++pass) {
    const std::size_t nn = capt.drain(evs, kEventsPerDrain);
    if (nn == 0)
      break;
    process_batch_ramp(rd, evs, nn);
  }

  pg.idle_low();
  capt.stop();
  g_ic.capture_idle.store(true, std::memory_order_release);
}

// Execute a HOLD: emit the pickup pattern at a constant, live-tunable RPM and
// measure per-spark BTDC continuously, until stop_request / lost_sync. Reuses the
// ramp reducer + pipeline (so the skew correction applies). Ramp-in floor is
// min(kHoldRampInFloorRpm, hold_rpm) so a sub-idle target is honoured; the single
// rate-limited set_rpm path is bidirectional (slews up OR down toward the target).
void run_hold(hal::SoftPattern& pg, hal::Capture& capt) {
  static constexpr std::uint32_t kHoldRampInFloorRpm = 800;
  const core::Config cfg = g_ic.config;
  capt.start();

  RampReducer rd;
  build_ramp_intra(rd, cfg);

  std::uint32_t target_rpm = cfg.hold_rpm ? cfg.hold_rpm : 1;
  g_ic.hold_target_rpm.store(target_rpm, std::memory_order_relaxed);
  const float slew_rpm_per_s = cfg.ramp_up_rpm_per_s; // >0 by config validation

  // cur_rpm is a FLOAT accumulator so fractional slew progress is never lost
  // (mirrors RampSchedule::tick). An earlier integer-step version truncated
  // slew_rpm_per_s*dt_s to 0 on every tight-loop iteration, and the +1 floor made
  // cur snap to target in <1 ms -- defeating the ramp-in that keeps a cold ignitor
  // firing. rpm_cmd is the rounded value handed to the emitter and used for timing.
  float cur_rpm = float(target_rpm < kHoldRampInFloorRpm ? target_rpm : kHoldRampInFloorRpm);
  if (cur_rpm < 1.0f)
    cur_rpm = 1.0f;
  std::uint32_t rpm_cmd = static_cast<std::uint32_t>(std::lroundf(cur_rpm));
  drain_stale(capt);
  pg.start(cfg, rpm_cmd, time_us_64());
  rd.cur_rpm_cmd_ = rpm_cmd;
  rd.sync_thresh_ticks_ = sync_thresh_ticks(rd.sync_thresh_deg_, rd.wheel_deg_, rpm_cmd);

  g_ic.capture_idle.store(false, std::memory_order_release);

  hal::CaptureEvent evs[kEventsPerDrain];
  std::uint64_t prev_us = time_us_64();
  std::uint64_t last_pickup_us = prev_us;

  // PC-less stop: core1 polls BOOTSEL itself inside the sync gap,
  // cooperatively parking core0 (IRQs-off RAM spin) so the QSPI CS float is XIP-safe.
  static constexpr std::uint64_t kPollGuardUs = 500;     // no emission edge due within this
  static constexpr std::uint64_t kPollPeriodUs = 100000; // ~10 Hz throttle
  static constexpr std::uint64_t kParkWaitUs = 400;      // max wait for core0 to ack the park
  // Ignore the start-press: don't treat BOOTSEL as STOP until it's been released once.
  bool armed = false;       // becomes true after the first observed release
  bool prev_pressed = true; // assume held at entry (the start press)
  std::uint64_t last_poll_us = time_us_64();

  for (;;) {
    const std::uint64_t now_us = time_us_64();

    if (g_ic.stop_request.load(std::memory_order_acquire))
      break;

    // Bidirectional float-accumulated slew toward the live target at the configured
    // rate; clamp so cur never overshoots the target in either direction.
    target_rpm = g_ic.hold_target_rpm.load(std::memory_order_relaxed);
    if (target_rpm == 0)
      target_rpm = 1;
    const float dt_s = float(now_us - prev_us) / 1.0e6f;
    prev_us = now_us;
    const float ftarget_rpm = float(target_rpm);
    const float delta_rpm = slew_rpm_per_s * dt_s;
    if (cur_rpm < ftarget_rpm)
      cur_rpm = (cur_rpm + delta_rpm < ftarget_rpm) ? cur_rpm + delta_rpm : ftarget_rpm;
    else if (cur_rpm > ftarget_rpm)
      cur_rpm = (cur_rpm - delta_rpm > ftarget_rpm) ? cur_rpm - delta_rpm : ftarget_rpm;
    rpm_cmd = static_cast<std::uint32_t>(std::lroundf(cur_rpm));
    if (rpm_cmd == 0)
      rpm_cmd = 1;
    rd.cur_rpm_cmd_ = rpm_cmd;

    const std::size_t nn = capt.drain(evs, kEventsPerDrain);
    if (nn > 0) {
      for (std::size_t i = 0; i < nn; ++i) {
        if (evs[i].ch == kPickupChId) {
          last_pickup_us = now_us;
          break;
        }
      }
      process_batch_ramp(rd, evs, nn);
    }

    pg.set_rpm(rpm_cmd);
    rd.sync_thresh_ticks_ = sync_thresh_ticks(rd.sync_thresh_deg_, rd.wheel_deg_, rpm_cmd);

    // Poll BOOTSEL from THIS core inside the pickup sync gap. Throttled + guarded so
    // no emission edge is delayed. Park core0 via the COOPERATIVE handshake, NOT the
    // SDK multicore_lockout: that primitive is single-relationship (one shared
    // request-id/mutex) and core0 is already the flush-write controller, so making
    // core0 a victim too corrupts the shared state and hangs the flush. Instead ask
    // core0 to spin IRQs-off in RAM (park_request/park_ack) and only float the QSPI
    // CS once it has acked AND no edge is imminent (re-checked after the ack wait,
    // since the wait runs with core1 IRQs on and emission continuing).
    if (now_us - last_poll_us >= kPollPeriodUs && pg.us_until_next_edge(now_us) > kPollGuardUs) {
      last_poll_us = now_us;
      g_ic.park_request.store(true, std::memory_order_release);
      bool parked = false;
      const std::uint64_t ack_deadline_us = time_us_64() + kParkWaitUs;
      while (time_us_64() < ack_deadline_us) {
        if (g_ic.park_ack.load(std::memory_order_acquire)) {
          parked = true;
          break;
        }
        tight_loop_contents();
      }
      bool did_read = false;
      bool pressed = false;
      if (parked && pg.us_until_next_edge(time_us_64()) > kPollGuardUs) {
        pressed = hal::bootsel_pressed(); // IRQs off + CS float; core0 parked IRQs-off
        did_read = true;
      }
      g_ic.park_request.store(false, std::memory_order_release); // release core0
      if (did_read) {
        if (!pressed)
          armed = true;                  // saw a release -> a later press = stop
        else if (armed && !prev_pressed) // debounced rising edge
          g_ic.stop_request.store(true, std::memory_order_release);
        prev_pressed = pressed;
      }
    }

    // Lost-sync watchdog (same shape as run_ramp).
    const float rev_us = 60.0e6f / float(rpm_cmd);
    const std::uint64_t pk_timeout = std::uint64_t(rev_us * 16.0f) + 5000u;
    if (now_us - last_pickup_us > pk_timeout) {
      g_ic.lost_sync.store(true, std::memory_order_release);
      break;
    }

    g_ic.hold_heartbeat.fetch_add(1, std::memory_order_relaxed);
    tight_loop_contents();
  }

  // Final bounded drain (flush trailing captures + resolve trailing sparks).
  for (int pass = 0; pass < 8; ++pass) {
    const std::size_t nn = capt.drain(evs, kEventsPerDrain);
    if (nn == 0)
      break;
    process_batch_ramp(rd, evs, nn);
  }

  pg.idle_low();
  capt.stop();
  g_ic.capture_idle.store(true, std::memory_order_release);
}

} // namespace

void core1_entry() {
  // Claim hardware ONCE (SM/DMA claims must not be repeated per run). The pickup
  // pattern is now generated in software on THIS core (SoftPattern, GP18 as SIO
  // output) -- pio0 is entirely free. All four capture channels live on pio1
  // (pickup self-capture + both ign sense lines + the tach reference diagnostic),
  // regardless of cfg.cyl_count; the pins are fixed and unused channels simply
  // observe idle lines. pio1 is FULL (4 SMs); pio0 is unused.
  static hal::SoftPattern pg;
  static hal::Capture capt;
  pg.init(hal::kPickup);
  capt.init_pickup_selfcapture(kSlotPickup, pio1, hal::kPickup, kPickupChId);
  capt.init_ignition(kSlotIgn0, pio1, hal::kIgnPins[0], /*ch_id=*/0); // ch_id == cyl 0
  capt.init_ignition(kSlotIgn1, pio1, hal::kIgnPins[1], /*ch_id=*/1); // ch_id == cyl 1
  // Tach REFERENCE-diagnostic channel on GP19 (idle-HIGH open-collector node).
  // Present only with the real ignitor; yields no edges on the bench.
  capt.init_tach(kSlotTach, pio1, hal::kTach, kTachChannel);

  // RP2040 multicore-flash XIP hazard: while core0 erases/programs flash, this
  // core must NOT be fetching from XIP flash (it will hard-fault). Register as a
  // lockout victim so core0 can force this core into a RAM-resident holding spot
  // (multicore_lockout_start_blocking/_end_blocking) for the duration of a flash
  // write. Registration only; the wait loop below keeps interrupts enabled, which
  // the lockout mechanism (an inter-core FIFO IRQ) requires to be able to catch us.
  multicore_lockout_victim_init();

  for (;;) {
    if (g_ic.run_request.load(std::memory_order_acquire)) {
      g_ic.run_request.store(false, std::memory_order_relaxed); // consume
      g_ic.lost_sync.store(false, std::memory_order_relaxed);
      g_ic.run_done.store(false, std::memory_order_relaxed);
      g_ic.stop_request.store(false, std::memory_order_relaxed);
      g_ic.run_active.store(true, std::memory_order_release);

      // Branch on the requested run mode (core0 published cfg before requesting).
      switch (g_ic.config.run_mode) {
      case core::RunMode::Stepped:
        run_sweep(pg, capt);
        break;
      case core::RunMode::RampUp:
        run_ramp(pg, capt, /*up=*/true);
        break;
      case core::RunMode::RampDown:
        run_ramp(pg, capt, /*up=*/false);
        break;
      case core::RunMode::Hold:
        run_hold(pg, capt);
        break;
      }

      g_ic.run_active.store(false, std::memory_order_release);
      g_ic.run_done.store(true, std::memory_order_release);
    }
    tight_loop_contents();
  }
}

} // namespace app
