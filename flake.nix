{
  description = "4dn-firing-timing-probe: RP2040 bench instrument firmware dev toolchain";

  inputs = {
    # Pinned to a specific nixos-unstable revision. flake.lock locks the exact
    # narHash, so the devShell is reproducible; this rev provides every tool below.
    nixpkgs.url = "github:NixOS/nixpkgs/34ab99075ac4f7e40cf037eef32cb1c360bb85e9";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    { nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };

        # ARM bare-metal toolchain. The `gcc-arm-embedded` attribute resolves
        # against the pinned nixpkgs rev, so we use it directly. The
        # documented fallback is `pkgs.pkgsCross.arm-embedded.buildPackages.gcc`.
        # Either way the binary is invoked as `arm-none-eabi-gcc`.
        armToolchain = pkgs.gcc-arm-embedded;
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = [
            # Native compiler for host tests.
            pkgs.gcc

            # ARM bare-metal cross toolchain (arm-none-eabi-gcc).
            armToolchain

            # Build tooling.
            pkgs.cmake
            pkgs.ninja
            pkgs.python3
            pkgs.git

            # SWD flash + debug over a CMSIS-DAP probe (e.g. nanoDAP): flashes
            # the .elf directly and runs a GDB/DAP server. Provides `probe-rs`
            # (probe-rs-tools-0.32.0 in the pinned rev). Needs raw-USB
            # permission for the probe (udev rule already installed).
            pkgs.probe-rs-tools

            # clang-format / clang-tidy / clangd.
            pkgs.clang-tools

            # Linters / formatters.
            # NOTE: `rumdl` (rvben/rumdl markdown linter) is present directly in
            # the pinned nixpkgs rev (rumdl-0.2.60), so no overlay / external
            # flake / fetchCrate is needed. If a future rev drops it, provide it
            # via rustPlatform.buildRustPackage + fetchFromGitHub "rvben/rumdl".
            pkgs.rumdl
            pkgs.ruff
            pkgs.shellcheck
            pkgs.shfmt
            pkgs.hadolint
            pkgs.gersemi
            pkgs.lefthook
          ];

          # Print the pinned ARM toolchain version for confidence on shell entry.
          shellHook = ''
            echo "[devShell] $(arm-none-eabi-gcc --version | head -n1)"
          '';
        };
      }
    );
}
