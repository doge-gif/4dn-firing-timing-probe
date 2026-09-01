# Architecture

How the prober turns a bare ignitor into an advance map, and how the firmware
and hardware are organised.

## Measurement principle

The SRV250 ignitor fires from a variable-reluctance crank pickup. The prober
*replaces the engine*: it synthesizes the pickup pulse train at a known,
commanded RPM and measures when the ignitor fires in response.

1. **Stimulus** — a CPU-timed pattern generator drives GP18 with the trigger
   wheel's edge sequence (4 teeth at 60/60/60/180°, each 10° wide) at the
   commanded RPM.
2. **Capture** — the PIO timestamps *both* edges of the two coil-drive outputs
   and of the emitted pickup itself, all against one free-running counter.
3. **Reconstruction** — each spark's crank angle is interpolated between the two
   pickup edges that bracket it; °BTDC = (TDC_ref − angle) mod wheel.

Because the pickup is emitted by the prober, the commanded RPM is exact and the
spark timing is measured, not assumed.

## Signal path and timebase

- **Pattern gen** (`hal/soft_pattern.hpp`): edges are emitted from a hardware
  timer-alarm IRQ, each edge time derived from the rev start plus a geometric
  fraction of the rev period — drift-free, and independent of main-loop load.
  RPM changes apply at the next rev boundary.
- **Capture** (`hal/capture.*`, `hal/capture.pio`): one PIO state machine per
  input pin does dual-edge timestamping into an SRAM ring via DMA, with the pin
  level packed into the timestamp so a dropped edge can't invert polarity. All
  SMs share clkdiv 1.0; the capture counter ticks at sysclk/2 = **62.5 MHz**
  (`kCaptureTicksPerCount = 2`). All capture SMs start in one synchronized batch, so
  they share a single phase-locked timebase.
- **Skew correction** (`core/skew.hpp`): each capture SM's counter pauses a
  fixed ~2.75 counts per edge it handles, so a fast channel (pickup, 8
  edges/rev) drifts against a slow one (a coil, ~2/rev). Each channel's
  accumulated pause is added back so all channels share one pause-free timebase
  — otherwise a spark compared to a tooth drifts over a run.

## BTDC computation

For each spark, `bracket_pk` finds the two emitted pickup edges `b`,`a` with
`b.t ≤ spark_t < a.t` and `interp_angle` interpolates the crank angle between
their known wheel angles. This is a **ratio within one revolution**, so the
capture counter's absolute rate cancels out — the result is robust to clock-rate
drift, and skew-corrected on top. `to_btdc` folds it against the cylinder's TDC.
The stepped and ramp/hold reducers share this exact math.

## Run modes

- **STEPPED** — the pattern runs continuously while the schedule steps RPM from
  `SWEEP_START_RPM` to `SWEEP_END_RPM` by `SWEEP_STEP_RPM`. At each step it lets
  the ignitor settle `SWEEP_SETTLE_REVS` revs, then averages `SWEEP_SAMPLES`
  sparks per cylinder (Welford mean/σ + median/min/max) into one row. An RPM
  with no fire is emitted as an `n=0` marker row (maps the cranking/redline
  edges). → `MAP_###.CSV`.
- **RAMP_UP / RAMP_DOWN** — RPM is integrated continuously by wall-clock time at
  `RAMP_UP_RPM_PER_S` / `RAMP_DOWN_RPM_PER_S` between the sweep bounds; every
  bracketable spark emits one raw row (no averaging). Hard-capped at 40 s.
  → `RAMPU###.CSV` / `RAMPD###.CSV`.
- **HOLD** — slews to `HOLD_RPM` and holds it indefinitely, one row per spark
  into a fixed most-recent-N window, until BOOTSEL is pressed. → `HOLD###.CSV`.

RAMP and HOLD share the per-spark reducer; STEPPED has its own averaging
reducer. All three share the pattern gen, capture, skew correction, and BTDC
math.

## Configuration model

Two tiers, deliberately split:

- **Baked geometry** (`firmware/src/constants.hpp`) — the DUT/wheel facts that
  are a property of the board, not an operator setting: `kWheelDeg`, the tooth
  spans, per-cylinder `kTdcRefDeg`, and the ignition-sense pins `kIgnPins`. The
  per-cylinder arrays are ordered in lockstep (cylinder *i* uses `kIgnPins[i]`
  and `kTdcRefDeg[i]`). Change these only for a different engine/board, then
  rebuild.
- **Run parameters** (`CONFIG.INI` on the USB volume) — `RUN_MODE`, the
  `SWEEP_*` scalars, optional `HOLD_RPM`, and the `RAMP_*_RPM_PER_S` rates. Read
  once at boot. A fresh device writes the baked default
  (`firmware/src/default_config.ini`, embedded via C23 `#embed`).

INI format: `key = value`; `;` starts a comment anywhere on a line; `:`
separates list items (e.g. per-cylinder values); keys are case-insensitive.

## Firmware structure

```
core/   Pure, SDK-free logic (INI parse, config validation, geometry, angle
        interpolation, skew, stats, CSV, FSM). Compiled for the host and unit-
        tested with doctest — no hardware needed.
hal/    RP2040 SDK layer: CPU pattern gen + PIO capture, DMA, GPIO, USB-MSC, flash
        FAT (FatFs). Isolated behind narrow interfaces.
app/    The application: dual-core wiring, run modes, the flush path.
```

**Dual-core split** (`app/`):

- **core0** (`main.cpp`) — USB (TinyUSB MSC), the tick-based FSM, and the
  end-of-run flush to flash. Owns all flash writes.
- **core1** (`core1_main.cpp`) — the deterministic domain: pattern emission,
  capture drain, per-spark reduction. Never touches flash during a run.

Rows cross core1→core0 over lock-free SPSC rings. Flash is written only at
end-of-run, while capture is idle and core1 is held in a RAM-resident lockout,
so an XIP flash program can't fault the other core.

**FSM states and LED** (`core/fsm.cpp`):

| State | LED | Meaning |
|-------|-----|---------|
| Ready | 1 Hz blink | idle; press BOOTSEL to start |
| Running | heartbeat | a run is in progress |
| Flush | double-blink | writing the CSV to flash |
| Done | brief solid | flush complete → returns to Ready |
| Error | triple-blink | lost sync (run aborted; no file written) |
| ConfigInvalid | 10 Hz blink | `CONFIG.INI` rejected at boot |

## Hardware

A Raspberry Pi Pico on a custom carrier PCB (KiCad project in
[`schematics/`](../schematics/README.md); it also runs on a breadboard).

The ignitor's internal output behaviour that the sense wiring and capture polarity
depend on — idle-high coil-drive outputs (12 V pull-up, pulled low during dwell) and
an open-collector tach — is derived from a
[reverse-engineered TNDF17 schematic](https://github.com/doge-gif/denso-TNDF17-reverse-engineer/blob/master/original_denso_TNDF17.pdf).

- **Power** — external 12 V (from a USB-C PD trigger negotiated by a CH221K)
  feeds the ignitor and an LM7805 12 V→5 V regulator; the 5 V drives Pico VSYS
  through a Schottky, source-OR'd with USB VBUS so the board runs standalone or
  over USB.
- **Pickup output** — GP18 → 100 Ω series resistor → the ignitor's pickup input;
  the 3V3 edge is enough to trigger its input stage.
- **Ignition sense (×2)** — each coil-drive line has a strong ≈470 Ω pull-up to
  +12 V, then a 2.7 kΩ / 1 kΩ divider scales the node down to just under 3.3 V at the
  MCU pin. Non-inverting, so the ignitor's collector polarity is preserved:
  idle-high, dwell-low, **spark = rising edge** — matching the firmware's idle-high
  capture.
- **Tach input (optional)** — GP19 via a low-Vf Schottky + 10 kΩ pull-up to 3V3;
  the ignitor's tach is an open-collector node, present only with a real ignitor.
- **Kill switch** — CONN_G pulled up; leave it open for run-enabled.

**Pin map:**

| GPIO | Signal | Notes |
|------|--------|-------|
| GP16 | Ignition sense, cylinder 2 | CONN_E |
| GP17 | Ignition sense, cylinder 1 | CONN_F |
| GP18 | Pickup output | SIO-driven; also PIO self-captured |
| GP19 | Tach input (optional) | reference diagnostic; absent on the bench |
| GP25 | Status LED | onboard |
| BOOTSEL | Start button | read via the QSPI-CS technique |

Note the cylinder↔connector mapping: CONN_E is cylinder 2 (GP16) and CONN_F is
cylinder 1 (GP17), so `kIgnPins = {17, 16}` (cyl0, cyl1).

## Output CSV formats

- **Stepped** (`MAP_###.CSV`):
  `rpm_cmd,cyl,n,mean_btdc_deg,median_btdc_deg,stddev_btdc_deg,min_btdc_deg,max_btdc_deg,mean_dwell_us,stddev_dwell_us,mean_sys_latency_us,tach_rpm`
- **Ramp / Hold** (`RAMPU###.CSV`, `RAMPD###.CSV`, `HOLD###.CSV`):
  `rpm_cmd,cyl,rev_period_us,btdc_deg,dwell_us`

Files are never overwritten — the firmware scans the volume for the lowest free
`###` on each flush.
