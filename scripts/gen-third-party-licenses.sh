#!/usr/bin/env bash
# Generate THIRD_PARTY_LICENSES.md from THIRD_PARTY_MANIFEST.tsv.
#
# The manifest is the single source of truth: each row names a bundled component
# and the on-disk license file whose text is embedded VERBATIM into the output,
# so the shipped notices can never drift from the actual license files (and a
# hand-typed wrong copy can't sneak in).
#
# Usage:
#   scripts/gen-third-party-licenses.sh          # (re)write THIRD_PARTY_LICENSES.md
#   scripts/gen-third-party-licenses.sh --check  # fail if it is out of date
#
# CI (ci/run.sh) and the lefthook pre-push hook run --check, so a stale or
# hand-edited notices file, a missing license path, or a changed upstream
# license text fails the build instead of shipping a wrong copy.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

MANIFEST="THIRD_PARTY_MANIFEST.tsv"
OUTFILE="THIRD_PARTY_LICENSES.md"

die() {
	echo "gen-third-party-licenses: $*" >&2
	exit 1
}

# emit_section SCOPE — print one "###" block per manifest row whose scope column
# equals SCOPE, embedding the referenced license file verbatim in a fenced block.
emit_section() {
	local want_scope="$1"
	local path name spdx scope provenance
	while IFS=$'\t' read -r path name spdx scope provenance || [ -n "$path" ]; do
		# Skip blank lines and '#' comments.
		case "$path" in
		'' | \#*) continue ;;
		esac
		[ "$scope" = "$want_scope" ] || continue
		[ -f "$path" ] || die "license file not found: $path (row: $name)"
		# A code fence inside the file would terminate our block early.
		if grep -q '```' "$path"; then
			die "license file contains a code fence, cannot embed: $path"
		fi
		printf '### %s — %s\n\n' "$name" "$spdx"
		# Optional provenance line (origin+version for vendored copies). A bold
		# label + plain text (not whole-line emphasis) avoids rumdl MD036.
		[ -n "$provenance" ] && printf '**Provenance:** %s\n\n' "$provenance"
		printf '```text\n'
		cat "$path"
		# Ensure a newline before the closing fence even if the file lacks one.
		[ -n "$(tail -c1 "$path")" ] && printf '\n'
		printf '```\n\n'
	done <"$MANIFEST"
}

# generate — print the full document to stdout (with a possibly-blank tail that
# render() trims).
generate() {
	[ -f "$MANIFEST" ] || die "manifest not found: $MANIFEST"
	cat <<'EOF'
# Third-Party Licenses

<!-- GENERATED FILE — do not edit by hand.
     Regenerate with: scripts/gen-third-party-licenses.sh
     Source of truth: THIRD_PARTY_MANIFEST.tsv -->

This project bundles the third-party components below. Each is distributed under
its own license, reproduced verbatim here. Ship this file alongside the firmware
binary (prober.uf2) to satisfy those licenses' attribution requirements.

## Components in the firmware binary (prober.uf2)

EOF
	emit_section binary
	cat <<'EOF'
## Components used only for host tests (not in prober.uf2)

The following are build/test dependencies only. They are not linked into the
distributed firmware, and are listed here for completeness.

EOF
	emit_section test
}

# render — generate(), then drop trailing blank lines so the file ends with
# exactly one newline (MD047). Blank lines inside the document are preserved.
render() {
	generate | awk '
		/^$/ { blanks++; next }
		{ for (i = 0; i < blanks; i++) print ""; blanks = 0; print }
	'
}

case "${1:-}" in
"")
	render >"$OUTFILE"
	echo "gen-third-party-licenses: wrote $OUTFILE"
	;;
--check)
	tmp="$(mktemp)"
	trap 'rm -f "$tmp"' EXIT
	render >"$tmp"
	if ! diff -u "$OUTFILE" "$tmp"; then
		die "$OUTFILE is out of date — run scripts/gen-third-party-licenses.sh"
	fi
	echo "gen-third-party-licenses: $OUTFILE is up to date"
	;;
*)
	die "unknown argument: $1 (use --check or no argument)"
	;;
esac
