# firmware/external — vendored dependencies

Third-party code the firmware builds against, pinned in-tree so a checkout is
fully reproducible.

## Submodules (pinned at a commit SHA; `shallow = true` in `.gitmodules`)

- **pico-sdk** (`pico-sdk/`): the official Raspberry Pi Pico C/C++ SDK — board
  support package, startup code, hardware abstraction layer, and CMake toolchain
  integration for the RP2040 target. Its own nested libraries (tinyusb, btstack,
  lwIP, mbedTLS, cyw43-driver, …) are deliberately NOT fetched; we vendor tinyusb
  directly instead (below).
- **tinyusb** (`tinyusb/`): the USB device stack, vendored DIRECTLY here rather
  than through pico-sdk's nested `lib/tinyusb`, so one non-recursive
  `git submodule update --init` populates everything. Kept pinned to the tinyusb
  commit the pico-sdk pin expects (see `.gitmodules`; pointed at by
  `PICO_TINYUSB_PATH` in `../CMakeLists.txt`).
- **doctest** (`doctest/`): a header-only C++ unit-testing framework, used only in
  the host-side test build (native compilation, not cross-compiled).

## Vendored source (not a submodule)

- **fatfs** (`fatfs/`): ELM ChaN FatFs, copied in-tree. Origin, version, and
  license are recorded in `../../THIRD_PARTY_MANIFEST.tsv`.

Cloning, the shallow-fetch details, and the tinyusb ↔ pico-sdk version coupling
when bumping a pin are documented once in
[docs/development.md](../../docs/development.md#getting-the-source) — follow that
rather than a plain `git fetch && git checkout`, which the shallow clones can't
satisfy.
