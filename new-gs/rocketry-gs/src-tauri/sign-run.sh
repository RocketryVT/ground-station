#!/usr/bin/env bash
# Cargo `runner` for macOS (see .cargo/config.toml).
#
# `tauri dev` builds `target/debug/rocketry-gs` and ld only adhoc/linker-signs
# it with no entitlements, so WKWebView refuses the page camera/mic access
# ("Could not create a 'com.apple.webkit.camera' sandbox extension" ->
# NotAllowedError: No AVVideoCaptureSource device). Cargo invokes this runner
# with the freshly built binary, AFTER linking and BEFORE running, on every
# rebuild -- so we re-sign here (replacing the linker signature) with the
# camera/mic entitlements and then exec the binary. The signature can't be
# wiped by a later relink because there is no later relink.
set -euo pipefail

BIN="$1"
shift

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENTITLEMENTS="$SCRIPT_DIR/Entitlements.dev.plist"
IDENTIFIER="org.rocketryatvirginiatech.rocketry-gs"

codesign --force --sign - --identifier "$IDENTIFIER" --entitlements "$ENTITLEMENTS" "$BIN" >/dev/null 2>&1 || {
  echo "sign-run: codesign failed for $BIN" >&2
  exit 1
}

exec "$BIN" "$@"
