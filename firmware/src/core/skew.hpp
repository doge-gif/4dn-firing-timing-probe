#pragma once
// Capture-clock skew correction (see constants.hpp kCounterPauseCountsPerEdge and
// core1_main.cpp process_batch). Each capture SM's counter pauses a fixed number
// of counts per edge it handles, so a channel's timestamps fall behind real time
// by k * (edges it has handled). Adding that back puts all channels on ONE
// pause-free timebase, so a cross-channel comparison (spark vs tooth) no longer
// drifts ATDC over a run. Pure + SDK-free so it is exercised by host unit tests.
#include "constants.hpp"

#include <cmath>
#include <cstdint>

namespace core {

// raw_ticks = a capture timestamp for this channel; edge_index = how many edges
// this channel has ALREADY handled before this one (0 for its first edge).
inline std::uint64_t skew_corrected_ticks(std::uint64_t raw_ticks, std::uint64_t edge_index) {
  // Keep both params in one expression so bugprone-easily-swappable-parameters'
  // SuppressParametersUsedTogether heuristic applies (a two-statement split trips it).
  return raw_ticks + static_cast<std::uint64_t>(std::llround(cfg::kCounterPauseCountsPerEdge *
                                                             static_cast<double>(edge_index)));
}

} // namespace core
