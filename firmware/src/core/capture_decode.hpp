#pragma once
// capture_decode -- PURE, SDK-free helper that turns a raw 32-bit PIO capture
// word into a 64-bit shared-timebase timestamp plus the edge's logic level.
//
// CAPTURE WORD FORMAT (hal/capture.pio): the SM samples the PIN LEVEL at each
// edge and packs it with the counter snapshot:
//   bit 31       = pin level at this edge (1 = line now HIGH => rising edge,
//                  0 = now LOW => falling edge)
//   bits [30:0]  = 31-bit free-running up-counter snapshot (62.5 MHz, wraps ~34s)
//
// Polarity is thus CARRIED PER EDGE. The earlier design inferred it by strict
// alternation from an assumed idle level; that was fragile -- a single dropped
// edge (or a mis-seeded start phase) silently inverted every edge afterwards,
// with no symptom but drifting BTDC. Sampling the level in PIO removes the guess.

#include "core/timebase.hpp"

#include <cstdint>

namespace core {

// Pushed-word layout (must match capture.pio). Bit 31 holds the level; the low 31
// bits hold the timestamp, so the channel Timebase64 is constructed 31-bit.
inline constexpr unsigned kCaptureLevelBit = 31;
inline constexpr unsigned kCaptureTsBits = 31;

// Decode one captured edge for a single channel.
//
//   raw   : the 32-bit capture word the SM pushed for THIS edge (bit 31 = level,
//           bits [30:0] = 31-bit counter snapshot; see format note above).
//   tb    : the channel's Timebase64, constructed 31-bit (core::kCaptureTsBits).
//           update()d with `raw` (it masks off the level bit), so its high word
//           increments whenever the 31-bit payload steps backwards (a 2^31 wrap).
//   level : OUTPUT -- set to the pin level captured in bit 31 (true = this edge is
//           rising, false = falling). No seeding required and no dependence on
//           prior calls: each edge stands alone, so a dropped edge cannot flip the
//           polarity of the edges after it.
//
// Semantics: each call represents exactly one detected edge. On return, `level`
// holds the level AFTER this edge (the steady level the line just transitioned
// to); the return value is the 64-bit timestamp.
std::uint64_t decode_capture(std::uint32_t raw, Timebase64& tb, bool& level);

} // namespace core
