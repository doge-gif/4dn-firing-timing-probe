#pragma once
// core1_main -- the core-1 deterministic domain entry.
//
// core1_entry() is passed to multicore_launch_core1() by core0. It owns the
// pickup emitter (SoftPattern, a CPU timer-alarm driving GP18 as SIO -- pio0 is
// left entirely free) and the Capture hardware (pio1), and runs the full
// capture->reduce pipeline, handing reduced rows to core0 via the shared g_ic
// contract. Only core1 touches the PIO/DMA/Capture/Timebase after launch.
#include "app/intercore.hpp"

namespace app {

// The single shared core0<->core1 state block. Defined once in core1_main.cpp.
extern Intercore g_ic;

// Core-1 loop entry point (never returns). Pass to multicore_launch_core1().
void core1_entry();

} // namespace app
