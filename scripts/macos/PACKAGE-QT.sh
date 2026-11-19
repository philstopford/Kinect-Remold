#!/usr/bin/env bash
set -euo pipefail
[[ "$(uname -s)" == Darwin && "$(uname -m)" == arm64 ]] || { echo 'Build on Apple Silicon macOS.' >&2; exit 2; }
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
[[ "${1:-}" == --no-build ]] || bash "$ROOT/Kinect-Remold-macos.sh" --build-only
OUT="$ROOT/binaries/releases"
APP="$ROOT/binaries/macos/applications/KinectRemoldQt.app"
[[ -d "$APP" ]] || { echo 'Build the macOS app first.' >&2; exit 1; }
work="$(mktemp -d "${TMPDIR:-/tmp}/remold-release.XXXXXX")"
trap 'rm -rf "$work"' EXIT
stage="$work/KinectRemoldQt"
mkdir -p "$stage" "$OUT"
ditto "$APP" "$stage/KinectRemoldQt.app"
cp "$ROOT/docs/PORTABLE-DEPLOYMENT.md" "$stage/README.md"
cp "$ROOT/LICENSE" "$stage/LICENSE"
APP="$stage/KinectRemoldQt.app"
if [[ -n "${REMOLD_MACOS_SIGN_IDENTITY:-}" ]]; then
  codesign --force --deep --options runtime --timestamp --sign "$REMOLD_MACOS_SIGN_IDENTITY" "$APP"
fi
codesign --verify --deep --strict "$APP"
archive="$OUT/KinectRemoldQt-macos-arm64.zip"
ditto -c -k --sequesterRsrc --keepParent "$stage" "$archive"
if [[ -n "${REMOLD_MACOS_NOTARY_PROFILE:-}" ]]; then
  [[ -n "${REMOLD_MACOS_SIGN_IDENTITY:-}" ]] || { echo 'Notarization requires REMOLD_MACOS_SIGN_IDENTITY.' >&2; exit 2; }
  xcrun notarytool submit "$archive" --keychain-profile "$REMOLD_MACOS_NOTARY_PROFILE" --wait
  xcrun stapler staple "$APP"
  ditto -c -k --sequesterRsrc --keepParent "$stage" "$archive"
fi
(cd "$OUT" && shasum -a 256 "$(basename "$archive")" > "$(basename "$archive").sha256")
echo "$archive"
