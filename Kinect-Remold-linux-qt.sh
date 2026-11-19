#!/usr/bin/env bash
set -euo pipefail
[[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || {
  echo 'Linux AMD64 is required.' >&2; exit 2;
}
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
mode="${1:-}"
[[ "$mode" == '' || "$mode" == --build-only ]] || {
  echo 'Usage: Kinect-Remold-linux-qt.sh [--build-only]' >&2; exit 2;
}
APP="$ROOT/binaries/linux/applications/KinectRemoldQt/KinectRemoldQt"
DRIVER="$ROOT/binaries/linux/drivers/kinect-one-remold/kinect-one-remold"
app_ready=1
[[ -x "$APP" ]] || app_ready=0
for asset in libonnxruntime.so.1 models/movenet.onnx models/rtmpose-hand.onnx models/face-landmarks.onnx models/face-blendshapes.onnx; do
  [[ -f "$(dirname "$APP")/$asset" ]] || app_ready=0
done
if [[ "$mode" == --build-only || "$app_ready" == 0 ]]; then
  bash "$ROOT/scripts/linux/BUILD-QT.sh"
fi
if [[ "$mode" == --build-only || ! -x "$DRIVER" ]]; then
  bash "$ROOT/drivers/modules/kinect-one-remold/linux/BUILD.sh"
fi
if [[ "$mode" == --build-only ]]; then
  echo "Linux Qt build complete: $ROOT/binaries/linux"; exit 0
fi
# Respect an explicitly configured or installed service. Otherwise own a
# temporary per-user v2 runtime, whose lifetime follows the foreground GUI.
if [[ -n "${REMOLD_LINUX_RUNTIME_ONE:-}" || -S /run/kinect-one-remold/sdk.sock ]]; then
  exec "$APP"
fi
runtime="$(mktemp -d /tmp/remold-one.XXXXXX)"
export REMOLD_LINUX_RUNTIME_ONE="$runtime"
driver_pid=''
cleanup() {
  if [[ -n "$driver_pid" ]]; then
    kill "$driver_pid" 2>/dev/null || true
    wait "$driver_pid" 2>/dev/null || true
  fi
  rmdir "$runtime" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
REMOLD_RUNTIME_DIR="$runtime" "$DRIVER" & driver_pid=$!
for ((attempt=0; attempt<50; ++attempt)); do
  [[ ! -S "$runtime/sdk.sock" ]] || break
  kill -0 "$driver_pid" 2>/dev/null || { echo 'Kinect v2 runtime failed to start.' >&2; exit 1; }
  sleep 0.1
done
[[ -S "$runtime/sdk.sock" ]] || { echo 'Kinect v2 runtime startup timed out.' >&2; exit 1; }
"$APP"
