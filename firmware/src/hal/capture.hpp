#pragma once
// Capture -- dual-edge timestamp capture.
//
// Each channel = one PIO capture SM (see capture.pio) watching one pin + one DMA
// channel draining that SM's RX FIFO into an SRAM ring. Edges are timestamped in
// PIO with zero per-edge CPU work; the CPU calls drain() opportunistically
// (non-real-time) to convert the raw 32-bit snapshots into 64-bit CaptureEvents
// via the pure core::decode_capture helper.
//
// Two channel flavours differ ONLY in their idle level (polarity anchor):
//   * ignition sense (GP16/GP17): idles HIGH, first edge FALLING (coil charge).
//   * pickup self-capture (GP18): idles LOW, first edge RISING (tooth leading).
// See core::decode_capture and init_ignition/init_pickup_selfcapture below.

#include "constants.hpp"
#include "core/capture_decode.hpp"
#include "core/timebase.hpp"
#include "hardware/pio.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace hal {

// Capture counter rate reconciliation (see capture.pio "TICK-UNIT DECISION"):
// the SM's poll loop is 2 PIO cycles per count, so the capture counter ticks at
// sysclk/2 = 62.5 MHz. Multiply captured deltas by this to convert
// capture-counts into 125 MHz sysclk ticks.
inline constexpr std::uint32_t kCaptureTicksPerCount = cfg::kCaptureTicksPerCount;

// Strong id for a capture channel (stamped into CaptureEvent.ch). A distinct type
// so an init_* call site cannot silently swap it with the adjacent GPIO `pin`.
struct ChannelId {
  std::uint8_t v = 0;
};

// A decoded edge in the shared 64-bit timebase (counts, NOT yet sysclk ticks --
// multiply a DELTA of two `ticks` by kCaptureTicksPerCount for sysclk ticks).
// `ticks` is 64-bit to feed core::EdgeRef / core::DwellPairer directly.
struct CaptureEvent {
  std::uint64_t ticks = 0; // 64-bit timestamp (capture-counts)
  std::uint8_t ch = 0;     // channel index (caller-assigned, e.g. cylinder)
  bool rising = false;     // true = this edge is a rising transition
};

// SRAM ring depth per channel (words). Power of two so the DMA ring-wrap
// alignment trick applies (ring_size_bits). Large enough that drain(), called
// every core1 loop iteration, never falls behind the edge rate even at RPM_MAX.
inline constexpr std::size_t kCaptureRingLen = cfg::kCaptureRingLen;

// One capture channel: SM + DMA + ring + per-channel decode state.
struct CaptureChannel {
  PIO pio = nullptr;
  uint sm = 0;
  uint pin = 0;
  uint offset = 0;
  int dma = -1;
  std::uint8_t ch_id = 0;

  // DMA-filled ring of raw 32-bit snapshots. 32-byte aligned to 2^ring_size_bits
  // words so the DMA write pointer wraps in hardware.
  alignas(kCaptureRingLen *
          sizeof(std::uint32_t)) std::array<std::uint32_t, kCaptureRingLen> ring{};

  // Software read index into `ring`; the DMA write index is derived from the
  // channel's transfer_count register (see capture.cpp).
  std::size_t read_idx = 0;

  // Per-channel decode state (31->64 extension; polarity now sampled
  // per edge in PIO, not alternated). 31-bit: capture word bit 31 holds the level.
  core::Timebase64 tb{core::kCaptureTsBits};
  bool level = false; // last decoded level (set per edge by decode_capture)
  bool started = false;
};

class Capture {
public:
  // Configure an IGNITION-sense channel (idle-HIGH, first edge FALLING).
  //   slot : index into this Capture's channel array (0..kMaxChannels-1)
  //   pio  : PIO block to use (see the self-capture note re: avoiding drive conflicts)
  //   pin  : input pin (GP16/GP17)
  //   ch_id: channel id stamped into CaptureEvent.ch (e.g. cylinder index)
  void init_ignition(std::size_t slot, PIO pio, uint pin, std::uint8_t ch_id);

  // Configure the GP18 PICKUP self-capture channel (idle-LOW, first edge
  // RISING). Used to timestamp the pickup generator's own GP18 output with NO
  // loopback wire: pass a PIO block that does NOT drive GP18 so we only passively
  // read the pad.
  void init_pickup_selfcapture(std::size_t slot, PIO pio, uint pin, std::uint8_t ch_id);

  // Configure the GP19 TACH reference-diagnostic channel. Reads the DUT ignitor's
  // tach output (one rising edge per crank rev, at tooth-0), which is present ONLY
  // with the real ignitor -- on the bench there is no signal, so this yields no
  // edges (expected). The node is a 12 V open-collector-ish line that idles HIGH
  // (NPN pulldown), so it uses the SAME idle-HIGH polarity path as init_ignition
  // (first edge FALLING). REFERENCE diagnostics only (system input latency +
  // sync-lock RPM); never used to correct BTDC.
  void init_tach(std::size_t slot, PIO pio, uint pin, std::uint8_t ch_id);

  // Enable ALL configured SMs in one synchronized batch so their free-running
  // counters are phase-locked into a single shared timebase, then
  // start their DMA channels. Call AFTER all init_* calls.
  void start();

  // Halt all SMs + DMA.
  void stop();

  // Drain newly captured edges from every channel's ring into `out` (up to
  // `max`), decoding each raw snapshot to a 64-bit timestamp + polarity. Returns
  // the number of events written. Non-real-time; called opportunistically (the
  // app drains every core1 loop iteration).
  // Marked __time_critical_func so the ring read/decode runs from RAM.
  std::size_t drain(CaptureEvent* out, std::size_t max);

  static constexpr std::size_t kMaxChannels = 4; // GP16, GP17, GP18, GP19(tach)

  // Diagnostics: the PIO program offset + SM each channel got (to verify program
  // memory did not overflow -- a -1 offset shows up here as a huge value).
  uint channel_offset(std::size_t slot) const { return chans_[slot].offset; }
  uint channel_sm(std::size_t slot) const { return chans_[slot].sm; }

private:
  void init_channel(std::size_t slot, PIO pio, uint pin, ChannelId ch_id, bool idle_high);
  uint load_program(PIO pio); // add the capture program ONCE per PIO block, cached
  void arm_dma(CaptureChannel& c);
  std::size_t drain_channel(CaptureChannel& c, CaptureEvent* out, std::size_t max);

  std::array<CaptureChannel, kMaxChannels> chans_{};
  std::size_t nchans_ = 0;

  // Shared program offset per PIO block. All capture SMs on a block run the SAME
  // program, so it is loaded ONCE (a per-channel copy would exhaust the 32-slot
  // PIO instruction memory once 3+ channels share a block -- pio_add_program then
  // returns -1 with no panic and the SM runs garbage).
  struct ProgLoad {
    PIO pio = nullptr;
    uint offset = 0;
  };
  std::array<ProgLoad, NUM_PIOS> loaded_{};
};

} // namespace hal
