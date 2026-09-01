#pragma once
// constants.hpp -- the ONE home for every magic number / config constant in the
// firmware. Pure constants only: no logic, no functions, no Config struct --
// just `inline constexpr` scalars plus the one baked-in default config.ini
// string. Interlocked constants are DERIVED from each other here so they can
// never drift (e.g. the PIO tick rate is computed from the system clock and the
// PIO clkdiv).
//
// LOAD-BEARING INVARIANT: this header is included by core/ (and thus by the host
// unit tests), so it MUST stay SDK-free -- NO pico/*, hardware/*, tusb.h, ff.h;
// plain types only (unsigned, std::uint*_t, float, char[]). Only <cstdint> and
// <cstddef> are permitted here.
#include <cstddef>
#include <cstdint>

namespace cfg {

// ---- Clock group (interlocked, derived) -------------------------------------
// The RP2040 system clock. Everything timing-related derives from this.
inline constexpr std::uint32_t kSysClockHz = 125'000'000;

// PIO clock divider (pattern-gen SM runs at kSysClockHz / kPioClkDiv).
inline constexpr float kPioClkDiv = 1.0f;

// PIO tick rate: the unit of Edge::ticks_to_next (== 125e6). The single source
// that core::kPioTicksPerSec (geometry) and core::kPioClockHz (config) alias.
inline constexpr float kPioTicksPerSec = float(kSysClockHz) / kPioClkDiv;

// Capture SM poll loop is 2 PIO cycles per counter tick, so the capture counter
// ticks at sysclk/2 = 62.5 MHz (see hal/capture.pio "TICK-UNIT DECISION").
inline constexpr std::uint32_t kCaptureCyclesPerCount = 2;

// Capture-count -> pattern-tick conversion factor (hal/capture.hpp): multiply a
// DELTA of two capture `ticks` by this to get pattern-ticks.
inline constexpr std::uint32_t kCaptureTicksPerCount = kCaptureCyclesPerCount;

// Capture counter rate in counts per microsecond (== 62.5). Used by HIL gates to
// convert a microsecond delay into capture counts.
inline constexpr float kCaptureCountsPerUs =
    kPioTicksPerSec / float(kCaptureCyclesPerCount) / 1.0e6f;

// ---- Capture-clock skew --------------------------------------------------
// Each capture SM's free-running counter PAUSES this many counts while handling
// one edge (mov/in/in/push don't decrement X). DETERMINISTIC PIO cycle cost, so
// a FIXED CONSTANT (a runtime EMA calibration wobbled -> 150 deg+ scatter).
// Measured: 22.0 counts deficit / 8 edges per pickup rev = 2.75 (stdev 1.8 / 35 revs).
inline constexpr double kCounterPauseCountsPerEdge = 2.75;

// ---- Pins (RP2040 GPIO numbers) ---------------------------------------------
// Fixed board I/O. The ignition-sense pins are per-cylinder and live with the
// rest of the baked DUT geometry below (kIgnPins), because pin<->cylinder<->TDC
// are interlocked and should be edited together.
inline constexpr unsigned kPickup = 18;
inline constexpr unsigned kTach = 19;
inline constexpr unsigned kLed = 25;

// tach = 1 rising edge per crank rev, at tooth-0. Backed by scope captures of the
// running ignitor (off / not-moving / 1182 rpm / 5890 rpm) in the reverse-
// engineering repo:
// https://github.com/doge-gif/denso-TNDF17-reverse-engineer/tree/master/scope_shots
inline constexpr unsigned kTachPulsesPerRev = 1;

// ---- Capacities / ring sizes ------------------------------------------------
inline constexpr std::size_t kMaxTeeth = 16;        // wheel teeth per rev (SRV250 uses 4)
inline constexpr std::size_t kMaxCylinders = 4;     // ignition outputs (SRV250 uses 2)
inline constexpr std::size_t kMaxIniKeys = 24;      // INI key table depth
inline constexpr std::size_t kCaptureRingLen = 256; // capture SRAM ring depth (words)
inline constexpr std::size_t kResultRingLen = 64;   // core1->core0 result SPSC ring depth

// ---- Baked DUT / wheel geometry (hardware-fixed; NOT in CONFIG.INI) ----------
// These describe the physical engine, trigger wheel, and ignition-sense wiring:
// a property of the BOARD/DUT, not an operator setting, so they are compiled in
// here (single source of truth) rather than read from CONFIG.INI. Change them
// only for a different engine/board, then rebuild. The per-cylinder arrays
// (kTdcRefDeg, kIgnPins) are ordered the SAME way -- cylinder i uses kIgnPins[i]
// AND kTdcRefDeg[i] -- so the pin<->cylinder<->TDC interlock cannot silently
// reverse (it did once: CONN_E/CONN_F were swapped). Keep them in lockstep.
inline constexpr float kWheelDeg = 360.0f; // full mechanical span of the wheel

// Trigger-wheel tooth spans as (leading, trailing) angles in wheel-frame
// degrees, ascending and non-overlapping. SRV250: 4 teeth, 60/60/60/180
// spacing, each 10 deg wide.
inline constexpr std::size_t kToothCount = 4;
inline constexpr float kToothLeadingDeg[kToothCount] = {0.0f, 60.0f, 120.0f, 180.0f};
inline constexpr float kToothTrailingDeg[kToothCount] = {10.0f, 70.0f, 130.0f, 190.0f};

// Cylinders: per-cylinder TDC reference angle (wheel-frame deg) and the
// ignition-sense GPIO. The harness wires CONN_E -> GP16 and CONN_F -> GP17, but
// CONN_E is cylinder 2 and CONN_F is cylinder 1, so cyl0 (= cylinder 1) reads
// GP17 and cyl1 (= cylinder 2) reads GP16. Each sense pin is captured by its OWN
// PIO state machine (one SM per pin), so the pins need NOT be adjacent/ordered.
inline constexpr std::size_t kCylCount = 2;
inline constexpr float kTdcRefDeg[kCylCount] = {130.0f, 190.0f};
inline constexpr unsigned kIgnPins[kCylCount] = {17, 16}; // cyl0=CONN_F=GP17, cyl1=CONN_E=GP16

static_assert(kToothCount > 0 && kToothCount <= kMaxTeeth, "baked tooth count out of range");
static_assert(kCylCount > 0 && kCylCount <= kMaxCylinders, "baked cylinder count out of range");
static_assert(kWheelDeg > 0.0f, "wheel span must be positive");
static_assert(kIgnPins[0] != kIgnPins[1], "ignition sense pins must be distinct GPIOs");

// ---- RPM ceiling ------------------------------------------------------------
// N_min: the narrowest emitted interval must stay >= this many PIO ticks
// (RPM_MAX_HW).
inline constexpr float kRpmCeilMinTicks = 1000.0f;

// ---- Flash FS geometry ------------------------------------------------------
// Reserve the top 512 KiB of flash for the FAT volume. The flash OFFSET is NOT
// here: it derives from PICO_FLASH_SIZE_BYTES (target-only SDK macro) as
// PICO_FLASH_SIZE_BYTES - cfg::kFsBytes, kept in diskio_flash.cpp.
inline constexpr std::uint32_t kFsBytes = 512u * 1024u;
inline constexpr std::uint32_t kLogicalSectorSizeBytes = 512; // FatFs logical sector (FF_MAX_SS)

// ---- Ramp row buffering ------------------------------------------------------
// Per-spark ramp rows are buffered in SRAM and flushed at end-of-run; if a ramp
// would exceed this it is truncated with an integrity flag.
inline constexpr std::size_t kMaxRampRows = 4096;

// ---- HOLD run mode ----------------------------------------------------------
// Default constant RPM when the optional HOLD_RPM key is absent.
inline constexpr std::uint32_t kDefaultHoldRpm = 3000;
// Recent-rows ring depth for HOLD (core0). ~40 KB of RampRow, ~8 s at 8k rpm.
// A forever-running mode keeps only this window (never streams the full run).
inline constexpr std::size_t kHoldWindowRows = 2048;

// ---- Baked-in default config.ini --------------------------------------------
// The pristine SRV250 config shipped on a fresh device. Valid per the
// config validator; the user edits it on the mounted USB drive.
//
// The text lives in the sibling default_config.ini (a real, commented,
// mountable INI file -- the single source of truth) and is embedded here
// byte-for-byte via the C23 #embed directive. GCC 15+ supports #embed in C++
// on both toolchains (host g++ and arm-none-eabi-g++); #embed resolves the
// quoted path relative to THIS header's directory. The trailing '\0' makes the
// array a valid C string for the std::string_view / strlen consumers.
inline constexpr char kDefaultConfigIni[] = {
#embed "default_config.ini"
    , '\0'};

// Size of the RAM buffers that hold the ENTIRE CONFIG.INI text while it is read
// back and parsed (core/fileio.cpp load path; hiltest g_cfg_text). FatFs f_read
// fills the buffer and silently drops the rest, so a CONFIG.INI larger than this
// is truncated -- dropping trailing keys and failing validation at boot. The
// baked default must fit with headroom for user edits; the static_assert below
// turns "default too big" into a BUILD error instead of a bricked first boot.
inline constexpr std::size_t kConfigTextBufBytes = 2048;
static_assert(sizeof(kDefaultConfigIni) <= kConfigTextBufBytes,
              "default_config.ini exceeds the CONFIG.INI read buffer: trim its "
              "comments, or raise cfg::kConfigTextBufBytes (which also grows the "
              "buf[]/g_cfg_text[] buffers that key off it).");

} // namespace cfg
