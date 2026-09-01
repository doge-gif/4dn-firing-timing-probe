#pragma once
// CPU pickup pattern generator, driven by a HARDWARE TIMER-ALARM IRQ.
//
// History: emission moved off the PIO+DMA self-refilling chain to the CPU. That
// chain reconfigured its DMA channel while transfers were still in flight, which
// trips RP2040-E13 (aborting an in-flight channel clears ABORT early and mis-fires
// a spurious completion IRQ; a mid-flight restart is then unpredictable) -- so the
// reconfigure intermittently corrupted or wedged the emitted pickup, desyncing the
// ignitor. A first cut emitted edges from the core1 poll loop -- but that made
// emission timing hostage to the main loop's per-spark reduce work: under load,
// pg.poll() was delayed, the emitted pickup jittered, and the ignitor retarded /
// flat-lined at high rpm (observed with a controlled load test). This version emits
// from a hardware timer-alarm IRQ, so the edge timing is INDEPENDENT of whatever
// the main loop is doing -- clean, constant ~1 us-resolution emission at all rpm
// regardless of reduce load.
//
// Accuracy note: the pickup is only the STIMULUS; the actual emitted GP18 edge is
// timestamped by the pio1 capture at 16 ns, so measurement accuracy is unchanged.
// The pin is SIO-driven; pio1 reads the same pad (capture_program_init never calls
// pio_gpio_init), so self-capture is unaffected and pio0 stays free.
//
// The owning loop no longer needs to do anything for emission.
#include "core/config.hpp"
#include "core/types.hpp"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/time.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace hal {

class SoftPattern {
public:
  void init(unsigned pin) {
    pin_ = pin;
    gpio_init(pin_);
    gpio_set_dir(pin_, GPIO_OUT);
    gpio_put(pin_, 0); // idle LOW (pickup deasserted)
    running_ = false;
    s_active_ = this;
    if (alarm_ < 0) {
      alarm_ = static_cast<int>(hardware_alarm_claim_unused(true));
      // Callback + IRQ enable land on the CALLING core (core1 in the app), so the
      // alarm fires and re-arms entirely on the deterministic-domain core.
      hardware_alarm_set_callback(static_cast<uint>(alarm_), &SoftPattern::alarm_cb);
    }
  }

  // Begin emitting cfg's tooth pattern at `rpm`, phased so tooth-0 leading (the
  // first RISING edge) is emitted at now_us. Per-edge intra-rev angles come from
  // cfg.teeth (leading,trailing order per tooth) -- nothing hardcoded. rpm and now_us
  // are distinct quantities (commanded RPM vs a microsecond timestamp); the proper fix
  // is strong Rpm/TimeUs units, deferred as a larger change.
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  void start(const core::Config& cfg, std::uint32_t rpm, std::uint64_t now_us) {
    wheel_deg_ = cfg.wheel_deg > 0.0f ? cfg.wheel_deg : 360.0f;
    std::size_t tc = cfg.tooth_count > core::kMaxTeeth ? core::kMaxTeeth : cfg.tooth_count;
    nedges_ = 0;
    for (std::size_t i = 0; i < tc; ++i) {
      edge_ang_[nedges_++] = cfg.teeth[i].leading_deg;
      edge_ang_[nedges_++] = cfg.teeth[i].trailing_deg;
    }
    rpm_ = pending_rpm_ = rpm ? rpm : 1;
    rev_period_us_ = 60.0e6 / double(rpm_);
    rev_start_us_ = now_us;
    edge_i_ = 0;
    level_ = 1u;            // first edge drives HIGH (tooth-0 leading == rising)
    next_edge_us_ = now_us; // emit edge 0 immediately
    running_ = nedges_ > 0;
    if (running_)
      arm(); // fires edge 0 now (missed target -> catch-up path) and schedules edge 1
  }

  // Change commanded rpm; applied at the NEXT revolution boundary (in the ISR) so
  // the rev in progress is never distorted. pending_rpm_ is an aligned 32-bit store
  // -> single-copy-atomic vs the ISR's read.
  void set_rpm(std::uint32_t rpm) { pending_rpm_ = rpm ? rpm : 1; }

  // Microseconds until the next scheduled emission edge, relative to `now`. 0 if
  // idle or already due. TEAR-SAFE: next_edge_us_ is 64-bit and written by the
  // alarm ISR on this core; a plain read can tear on Cortex-M0+ and a torn
  // over-estimate would defeat the poll guard. Bracket with an IRQ disable.
  std::uint64_t us_until_next_edge(std::uint64_t now_us) const {
    const std::uint32_t flags = save_and_disable_interrupts();
    const std::uint64_t next = next_edge_us_;
    const bool run = running_;
    restore_interrupts(flags);
    if (!run || next <= now_us)
      return 0;
    return next - now_us;
  }

  void idle_low() {
    running_ = false;
    if (alarm_ >= 0)
      hardware_alarm_cancel(static_cast<uint>(alarm_));
    gpio_put(pin_, 0);
  }

private:
  // Emit the edge currently at next_edge_us_ and advance state to the following
  // edge (drift-free: each edge time derived from rev_start_us_, never accumulated).
  void fire_one() {
    gpio_put(pin_, level_);
    level_ ^= 1u;
    if (++edge_i_ >= nedges_) {
      edge_i_ = 0;
      rev_start_us_ += static_cast<std::uint64_t>(std::llround(rev_period_us_));
      if (pending_rpm_ != rpm_) {
        rpm_ = pending_rpm_;
        rev_period_us_ = 60.0e6 / double(rpm_);
      }
      // nedges_ is even (2 edges/tooth) so level_ is back to 1 (rising) here.
    }
    next_edge_us_ =
        rev_start_us_ + static_cast<std::uint64_t>(std::llround(
                            double(edge_ang_[edge_i_]) / double(wheel_deg_) * rev_period_us_));
  }

  // Arm the alarm for next_edge_us_. If that time already passed (we fell behind, or
  // at start), emit that edge immediately and advance until the target is in the
  // future -- so a late IRQ can never wedge emission (same catch-up as the old poll).
  void arm() {
    while (
        hardware_alarm_set_target(static_cast<uint>(alarm_), from_us_since_boot(next_edge_us_))) {
      fire_one();
    }
  }

  void on_alarm() {
    if (!running_)
      return;
    fire_one(); // the armed edge (next_edge_us_) has arrived -> emit + advance
    arm();      // schedule the following edge
  }

  static void alarm_cb(uint) {
    if (s_active_)
      s_active_->on_alarm();
  }

  inline static SoftPattern* s_active_ = nullptr; // the instance the alarm ISR services

  unsigned pin_ = 0;
  int alarm_ = -1;                        // claimed hardware alarm (0..3), -1 = none
  float edge_ang_[core::kMaxTeeth * 2]{}; // per-edge intra-rev angle (deg)
  std::size_t nedges_ = 0;
  float wheel_deg_ = 360.0f;
  std::uint32_t rpm_ = 1;
  volatile std::uint32_t pending_rpm_ = 1; // written by main loop, read by ISR
  double rev_period_us_ = 0.0;
  std::uint64_t rev_start_us_ = 0; // time of the current rev's tooth-0 leading
  std::uint64_t next_edge_us_ = 0; // time of the edge the alarm is armed for
  std::size_t edge_i_ = 0;         // next edge index within the rev
  unsigned level_ = 1u;            // level to drive at the next edge
  bool running_ = false;
};

} // namespace hal
