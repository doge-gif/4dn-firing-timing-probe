/*
 * prober_sections.ld -- link-time bound on RAM-hot (.time_critical) code.
 *
 * The `.time_critical` code is copied into the 264 KB SRAM at boot. Cap it so
 * that RAM-hot code growing too large is a BUILD FAILURE, not silent RAM
 * exhaustion at runtime.
 *
 * Cap: 12 KB. (Raised from the original 8 KB: mirroring the fixed-K capture-clock
 * skew correction into the ramp/HOLD pipeline (process_batch_ramp, a RAM-hot
 * __time_critical_func) put .time_critical at ~8.9 KB, past the original
 * conservative 8 KB bound. RAM is not the constraint -- prober uses ~141 KB of the
 * 264 KB SRAM (~123 KB free) even before the HOLD ring -- so the guard is re-set to
 * 12 KB, keeping a meaningful ceiling with ~3 KB headroom.)
 *
 * Mechanism: the companion override linker/section_default_data.incl brackets
 * the `*(.time_critical*)` input (which pico-sdk 2.x folds into `.data`, so it
 * has no standalone output section) with the symbols __time_critical_start__
 * and __time_critical_end__. This file is supplied to GNU ld as an extra `-T`
 * script; its top-level ASSERT is evaluated after section layout, when those
 * symbols are known, and fails the link with the message below if the bound is
 * exceeded.
 *
 * Wired onto the `prober` target in ../src/CMakeLists.txt.
 */

ASSERT(__time_critical_end__ - __time_critical_start__ <= 12K,
    "time_critical section too large")
