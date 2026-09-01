# Development

Build, test, flash, and debug. Firmware structure and hardware are in
[architecture.md](architecture.md).

## Getting the source

The submodule dependencies are all top-level (`pico-sdk`, `doctest`, and
`tinyusb` — vendored directly rather than through pico-sdk's nested
`lib/tinyusb`, so no recursive fetch is needed). They are marked
`shallow = true`, so a single non-recursive init clones depth-1 snapshots only
(≈60 MB of `.git` instead of ≈470 MB); pico-sdk's unused nested libraries
(btstack, mbedtls, …) are never fetched. The checked-out files are identical.
(FatFs is not a submodule — it is vendored in-tree under `firmware/external/fatfs`
and needs no fetch; see [firmware/external/README.md](../firmware/external/README.md).)

```sh
git clone https://…/4dn-firing-timing-probe.git
# already cloned:
git submodule update --init
```

Two things when bumping the pico-sdk pin: re-pin `external/tinyusb` to the
tinyusb commit the new SDK expects (see `.gitmodules`), and fetch depth-matched
since shallow submodules lack other commits' history:
`git submodule update --remote --depth 1`.

## Toolchain

Everything runs inside one of two pinned environments — don't rely on host
tools:

- **Nix devShell** (local, recommended): `nix develop`. Provides
  `arm-none-eabi-gcc`, `cmake`, `probe-rs`, and the linters. Commands below
  assume you are inside it.
  Follow <https://nixos.org/download> to install Nix package manager.
- **Container** (what CI uses): `firmware/docker/run.sh bash -lc '…'`, built
  from the pinned image in `firmware/docker/`.

## Host unit tests

`core/` is SDK-free and tested with doctest on the host — no hardware:

```sh
cmake -S firmware -B build-host -DBUILD_TARGET=host
cmake --build build-host -j
ctest --test-dir build-host --output-on-failure
```

This is the default development path; run it before every commit.

## Firmware build

```sh
export PICO_SDK_PATH="$PWD/firmware/external/pico-sdk"
cmake -S firmware -B build-pico -DBUILD_TARGET=pico -DPICO_BOARD=pico
cmake --build build-pico --target prober -j
```

Outputs land in `build-pico/src/`: `prober.uf2` (for BOOTSEL/UF2) and
`prober.elf` (for probe-rs). Add `-DPROBER_HILTEST=ON` to also build the
on-silicon HIL self-test firmwares (`hil_ignitor`, `hil_ramp`, …); the normal
`prober` build does not need it.

## Flash

**UF2 (no probe):** hold BOOTSEL while plugging in the Pico → it mounts as
`RPI-RP2` → copy `build-pico/src/prober.uf2` onto it.

**SWD via probe-rs (CMSIS-DAP / DAPLINK):**

```sh
probe-rs download --chip RP2040 build-pico/src/prober.elf
probe-rs reset --chip RP2040
```

`download` reprograms only the firmware region — it does **not** touch the
top-of-flash FAT volume, so `CONFIG.INI` and result CSVs survive a reflash.

## Run and retrieve

1. Mount the Pico's USB drive; edit `CONFIG.INI` (run parameters only — see
   [architecture.md](architecture.md#configuration-model)).
2. **Reset** the Pico (config is read once at boot). LED patterns:

   | LED | State |
   |-----|-------|
   | 1 Hz blink | Ready — press BOOTSEL to start |
   | heartbeat | Running |
   | double-blink | Flushing the CSV |
   | brief solid | Done |
   | triple-blink | Error (lost sync; no file written) |
   | 10 Hz blink | Config invalid (fix `CONFIG.INI`) |

3. Press **BOOTSEL** to start. The volume is writer-locked read-only during a
   run.
4. **Wait for steady 1 Hz (Ready)** — the file is written only at end-of-run
   flush. Then replug the drive (the auto media-change is not reliable on all
   hosts) and copy the CSV.

## Debugging

- **SWD perturbs the pickup.** Poking RAM / halting over SWD during a *live
  run* garbles emission and measurement (and can even trip a USB panic). Read
  markers only when idle/Ready; for live signal checks use a scope on GP18 and
  the spark lines, not SWD.
- **Hung board:** `probe-rs gdb --chip RP2040 …` then attach `arm-none-eabi-gdb`
  (thread 1 = core0, thread 2 = core1) to read the fault PC / backtrace. Give
  the stub a few seconds to come up.
- **Corrupt FS recovery:** zero the FAT boot sector to force a reformat on next
  boot (the device rewrites a default `CONFIG.INI`):

  ```sh
  head -c 4096 /dev/zero > zero4k.bin
  probe-rs download --chip RP2040 --binary-format bin --base-address 0x10180000 zero4k.bin
  probe-rs reset --chip RP2040
  ```

  (`probe-rs erase` full-chip is unreliable on the DAPLINK; zeroing the boot
  sector is the working path.)

## Full CI locally

Build + lint (rumdl/shfmt/shellcheck/hadolint) + host & pico builds +
clang-format + clang-tidy, exactly as CI runs it:

```sh
firmware/docker/run.sh bash -lc '../ci/run.sh'
```

## Commit hooks

`lefthook` runs `clang-format`, `gersemi`, `rumdl`, `shfmt`, and `shellcheck` on
commit — **commit from inside `nix develop`** so the hooks are on PATH.
`clang-format` frequently reformats and fails the first attempt; the reliable
pattern is `clang-format -i <files>` → `git add` → `git commit`.
