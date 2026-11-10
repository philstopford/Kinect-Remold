#!/usr/bin/env bash
set -euo pipefail
[[ "$(uname -s)" == Darwin && "$(uname -m)" == arm64 ]] || {
  echo 'Apple Silicon macOS is required.' >&2; exit 2;
}
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
TEMP_ROOT="${TMPDIR:-/tmp}"; TEMP_ROOT="${TEMP_ROOT%/}"
BUILD="${REMOLD_WORK_ROOT:-$TEMP_ROOT/kinect-remold-build}/macos/native-usb"
OUTPUT="$ROOT/binaries/macos/drivers/native-usb"
APP="$ROOT/binaries/macos/applications/KinectRemoldQt.app/Contents/MacOS/KinectRemoldQt"
mode="${1:-}"
[[ "$mode" == '' || "$mode" == --build-only || "$mode" == --driver-only ]] || {
  echo 'Usage: Kinect-Remold-macos.sh [--build-only|--driver-only]' >&2; exit 2;
}
command -v cmake >/dev/null 2>&1 || { echo 'CMake is required.' >&2; exit 2; }
mkdir -p "$BUILD" "$OUTPUT"
for spec in "REMOLD_MACOS_RUNTIME360:/tmp/k360-remold-$(id -u)" "REMOLD_MACOS_RUNTIME_ONE:/tmp/kone-remold-$(id -u)"; do
  key="${spec%%:*}"; runtime="${spec#*:}"
  [[ ! -L "$runtime" ]] || { echo "Refusing symlinked runtime directory: $runtime" >&2; exit 2; }
  if [[ -e "$runtime" ]]; then
    [[ -d "$runtime" && "$(stat -f %u "$runtime")" == "$(id -u)" ]] || {
      echo "Runtime directory is not owned by this user: $runtime" >&2; exit 2;
    }
  else
    mkdir -m 700 "$runtime"
  fi
  chmod 700 "$runtime"
  export "$key=$runtime"
done
cmake -S "$ROOT/drivers/macos" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64
cmake --build "$BUILD" --config Release --parallel
install -m 0755 "$BUILD/kinect-remold-native-usb" "$OUTPUT/kinect-remold-native-usb"
if [[ "$mode" != --driver-only ]]; then bash "$ROOT/scripts/macos/BUILD-STUDIO.sh"; fi
if [[ "$mode" == --build-only ]]; then
  echo "macOS native build complete: $ROOT/binaries/macos"; exit 0
fi
"$OUTPUT/kinect-remold-native-usb" & driver_pid=$!
cleanup(){ kill "$driver_pid" 2>/dev/null || true; wait "$driver_pid" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
if [[ "$mode" == --driver-only ]]; then
  echo 'Native IOUSBHost discovery is running. Press Ctrl-C to stop.'
  wait "$driver_pid"
else
  # Use the plugins shipped with this bundle even when a parent shell has a
  # different Qt installation configured.
  QT_PLUGIN_PATH="$ROOT/binaries/macos/applications/KinectRemoldQt.app/Contents/PlugIns" "$APP"
fi
