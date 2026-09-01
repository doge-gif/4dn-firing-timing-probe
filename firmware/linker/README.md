# linker/

Link-time bound on the RAM-hot `.time_critical` code.

The `.time_critical` code is copied into the 264 KB SRAM at boot, so it is
capped at **12 KB** with a linker `ASSERT`: RAM-hot code that grows past the cap
is a BUILD FAILURE instead of silent RAM exhaustion at runtime.

## Files

- `prober_sections.ld` — extra `-T` fragment holding the top-level `ASSERT`
  (the 12 KB cap). Wired onto the `prober` target via `target_link_options` in
  `../src/CMakeLists.txt`.
- `section_default_data.incl` — override of the pico-sdk 2.3.0 file of the same
  name. Adds the `__time_critical_start__` / `__time_critical_end__` symbols
  that bracket `*(.time_critical*)`; the `ASSERT` asserts on their difference.
  Wired in via `pico_add_linker_script_override_path(prober ...)`.

## Why bracket symbols instead of `SIZEOF(.time_critical)`

In pico-sdk 2.x the `*(.time_critical*)` input sections are folded into the
`.data` output section (in SDK 1.x they were a standalone `.time_critical`
output section). So `SIZEOF(.time_critical)` is always 0 and a naive
`ASSERT(SIZEOF(.time_critical) <= 8K)` can never fire. Keeping the input inside
`.data` is also required for correctness: `crt0.S` copies
`[__data_start__, __data_end__)` from flash to SRAM at boot via a fixed copy
table, so relocating `.time_critical` into its own output section would leave
the hot functions uninitialised in RAM. Bracketing measures the code in place.

## Maintenance note

`section_default_data.incl` is a faithful copy of the vendored SDK file with
only the two bracket symbols added. If the vendored pico-sdk is bumped, re-sync
this override against the new `section_default_data.incl`.
