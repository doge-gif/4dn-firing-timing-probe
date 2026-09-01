#!/usr/bin/env bash
# CI entry point — runs INSIDE the prober-toolchain container.
# Invoke via: firmware/docker/run.sh bash -lc '../ci/run.sh'
# (run.sh sets workdir to firmware/; ../ci/run.sh reaches this file from there.)
# Any CI system (GitHub Actions, GitLab CI, …) can call it the same way.
# run.sh executes this inside the flake devShell (`nix develop`), so the exact
# pinned toolchain from flake.lock is on PATH — the same one local dev uses.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)" # ci/ lives at repo root
cd "$ROOT"

# run_if_any TOOL_AND_FLAGS GLOB…
# Collects tracked files matching the globs via `git ls-files`, then invokes
# TOOL_AND_FLAGS with those files as positional arguments (word-split
# intentionally — each file becomes its own argument).  Returns early (exit 0)
# when no matching files exist so early-stage builds with sparse trees don't
# fail.  On a non-empty list the tool's exit code propagates to `set -e`;
# no `|| true` guard is added so real lint failures are never masked.
run_if_any() {
	local tool_and_flags="$1"
	shift
	local files
	files=$(git ls-files "$@")
	[ -n "$files" ] || return 0
	# SC2086: word-split of $files is intentional — each path is a separate arg.
	# SC2294: eval of $tool_and_flags is intentional — it carries flags, not just a name.
	# shellcheck disable=SC2086,SC2294
	eval "$tool_and_flags" $files
}

# ── 1) Repo-wide linting (fail fast on first error) ─────────────────────────
rumdl check docs/
shfmt -d firmware/docker/run.sh ci/run.sh scripts/gen-third-party-licenses.sh
shellcheck firmware/docker/run.sh ci/run.sh scripts/gen-third-party-licenses.sh
hadolint firmware/docker/Dockerfile
# Fail if THIRD_PARTY_LICENSES.md drifts from the manifest / upstream license
# files (stale hand edit, moved path, or changed upstream text).
scripts/gen-third-party-licenses.sh --check

# ── 2) Build + test in CI-only dirs (never clobber host clangd DBs) ─────────
cd "$ROOT/firmware"

# Start from empty CI build dirs: a stale CMakeCache left by a different toolchain
# (e.g. a previous image, or a local host configure) would otherwise pin an invalid
# compiler path. Real CI runs on a clean checkout where these do not exist; this
# also keeps re-runs on a dirty working tree hermetic.
rm -rf "$ROOT/build-host-ci" "$ROOT/build-pico-ci"

# EXPORT_COMPILE_COMMANDS so the core/*.cpp clang-tidy pass (step 3) can find the
# native compile DB here (host ABI); the pico DB below covers hal/ + app/.
cmake -S . -B ../build-host-ci -DBUILD_TARGET=host -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build ../build-host-ci
ctest --test-dir ../build-host-ci --output-on-failure

export PICO_SDK_PATH="$PWD/external/pico-sdk"
cmake -S . -B ../build-pico-ci \
	-DBUILD_TARGET=pico \
	-DPICO_BOARD=pico \
	-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build ../build-pico-ci

# ── 3) clang-format + clang-tidy against the CI compile DBs ─────────────────
cd "$ROOT"

# clang-format: check all C++ sources.
run_if_any "clang-format --dry-run -Werror" \
	'firmware/src/**/*.cpp' 'firmware/src/**/*.hpp'

# clang-tidy: route each layer to the correct compile DB so that
# host-only code uses the native DB and pico-specific code uses the
# cross-compile DB.  Using separate invocations avoids mixing ABIs.
run_if_any "clang-tidy -p firmware/../build-host-ci" \
	'firmware/src/core/*.cpp'
# clang-tidy uses clang, whose driver defaults to the HOST target and (under Nix)
# unconditionally pulls host glibc headers -- these fail on the 32-bit ARM code with
# "gnu/stubs-32.h not found" (Nix's host glibc has no 32-bit multilib). No --target /
# --sysroot / --gcc-* flag removes that default. So drop ALL of clang's built-in
# search dirs with -nostdinc and feed it ONLY the arm-none-eabi toolchain's own
# headers: newlib C, libstdc++ + the thumb/v6-m/nofp multilib (Cortex-M0+, soft-float),
# and gcc's builtin headers. --target makes clang treat the TU as ARM bare-metal.
# All paths are derived from the compiler so they track the pinned flake toolchain;
# these flags are CI-script-local (they do NOT modify .clang-tidy).
ARM_SYSROOT="$(arm-none-eabi-g++ -print-sysroot)"
ARM_VER="$(arm-none-eabi-g++ -dumpversion)"
ARM_CXX_INCL="${ARM_SYSROOT}/include/c++/${ARM_VER}"
ARM_GCC_INCL="$(dirname "$(arm-none-eabi-gcc -print-libgcc-file-name)")"
PICO_TIDY="clang-tidy -p firmware/../build-pico-ci"
PICO_TIDY+=" --extra-arg-before=--target=arm-none-eabi"
PICO_TIDY+=" --extra-arg=-nostdinc"
PICO_TIDY+=" --extra-arg=-isystem${ARM_CXX_INCL}"
PICO_TIDY+=" --extra-arg=-isystem${ARM_CXX_INCL}/arm-none-eabi/thumb/v6-m/nofp"
PICO_TIDY+=" --extra-arg=-isystem${ARM_SYSROOT}/include"
PICO_TIDY+=" --extra-arg=-isystem${ARM_GCC_INCL}/include"
PICO_TIDY+=" --extra-arg=-isystem${ARM_GCC_INCL}/include-fixed"
run_if_any "$PICO_TIDY" \
	'firmware/src/hal/*.cpp' 'firmware/src/app/*.cpp'
