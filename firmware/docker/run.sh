#!/usr/bin/env bash
set -euo pipefail
REPO="$(git rev-parse --show-toplevel)"
IMAGE="prober-toolchain:latest"
# Only allocate a TTY when we actually have one — CI has none, and -it without a TTY errors.
TTY=()
[ -t 0 ] && TTY=(-it)
# Same-path bind mount so compile DB paths match host; :Z for SELinux relabel
# (Fedora host). Working dir is firmware/ (dev build commands use -S . -B ../build-*).
# The command runs INSIDE the flake devShell (`nix develop "$REPO"`), so the exact
# pinned toolchain from flake.lock is on PATH — identical to a local `nix develop`.
# Runs as root in the container (no --userns=keep-id): the nixos/nix image is
# single-user with root-owned /nix, so a mapped non-root user could not use the
# store. Build outputs under the repo are therefore root-owned, which CI ignores.
exec podman run --rm "${TTY[@]}" \
	-v "$REPO:$REPO:Z" \
	-w "$REPO/firmware" \
	"$IMAGE" \
	nix develop "$REPO" --command "$@"
