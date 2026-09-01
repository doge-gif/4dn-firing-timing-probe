# CI/release builder image for the 4dn-firing-timing-probe firmware.
#
# This is a THIN wrapper over the SAME Nix flake the devShell uses (../../flake.nix).
# firmware/docker/run.sh runs `nix develop <repo> --command ...` inside this image,
# so `nix develop` realizes the exact pinned toolchain (arm-none-eabi-gcc, host gcc,
# cmake, probe-rs, and every linter) from flake.lock -- the CI toolchain therefore
# CANNOT drift from local dev.
#
# Why this shape: the previous image hand-duplicated the toolchain on Debian and
# drifted (its gcc predated the C23 #embed the firmware requires, so it could not
# build the repo at all). Sourcing everything from the flake removes that entire
# class of bug and needs no per-tool version pins here -- flake.lock (a nixpkgs
# narHash plus per-tool fixed-output hashes) is the single, cryptographic source of
# truth for every tool's provenance.
#
# The image carries no repo sources: run.sh bind-mounts the repo at its real host
# path and points `nix develop` at it, so the devShell wraps whatever command CI
# passes (e.g. ci/run.sh). The toolchain is substituted from cache.nixos.org on the
# first `nix develop` of each run; persisting/pushing a store-warmed image (e.g. to
# GHCR) to skip that is a later optimization, not required for correctness.
# Fully-qualified so `podman build` resolves it without an unqualified-search
# registry (Fedora enforces this; harmless on Docker/GitHub Actions runners).
FROM docker.io/nixos/nix:2.35.2

# Enable the flakes + nix-command features that run.sh's `nix develop` needs.
RUN mkdir -p /etc/nix \
    && printf 'experimental-features = nix-command flakes\n' >> /etc/nix/nix.conf
