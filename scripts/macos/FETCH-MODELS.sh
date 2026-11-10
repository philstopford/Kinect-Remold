#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
# Keep the downloaded originals outside the source tree. Only verified copies
# are staged in the generated app bundle.
source "$ROOT/config/build.env"
# Keep verified downloads across reboots; the temporary build directory is
# cleared by macOS and would otherwise force a network fetch every time.
CACHE="${REMOLD_MODEL_CACHE:-$ROOT/binaries/model-cache}"
DEST="${1:-$ROOT/binaries/macos/applications/KinectRemoldQt.app/Contents/Resources/models}"
mkdir -p "$CACHE" "$DEST"
fetch() {
  local name="$1" url="$2" expected="$3"
  local cached="$CACHE/$name" temp="$CACHE/$name.download"
  if [[ -f "$cached" ]] && [[ "$(shasum -a 256 "$cached" | awk '{print $1}')" == "$expected" ]]; then
    :
  else
    rm -f "$temp"
    curl --fail --location --retry 3 --connect-timeout 15 --output "$temp" "$url"
    [[ "$(shasum -a 256 "$temp" | awk '{print $1}')" == "$expected" ]] || {
      rm -f "$temp"; echo "SHA-256 mismatch: $name" >&2; return 1;
    }
    mv "$temp" "$cached"
  fi
  install -m 0644 "$cached" "$DEST/$name"
}
fetch movenet.onnx "$REMOLD_MOVENET_MODEL_URL" "$REMOLD_MOVENET_MODEL_SHA256"
fetch rtmpose-hand.onnx "$REMOLD_RTMPOSE_HAND_MODEL_URL" "$REMOLD_RTMPOSE_HAND_MODEL_SHA256"
fetch face_landmarker.task "$REMOLD_FACE_LANDMARKER_URL" "$REMOLD_FACE_LANDMARKER_SHA256"
fetch face-landmarks.onnx "$REMOLD_FACE_LANDMARKS_ONNX_URL" "$REMOLD_FACE_LANDMARKS_ONNX_SHA256"
install -m 0644 "$ROOT/third_party/models/face-blendshapes.onnx" "$DEST/face-blendshapes.onnx"
install -m 0644 "$ROOT/third_party/models/README.md" "$DEST/face-model-NOTICE.txt"
install -m 0644 "$ROOT/third_party/models/APACHE-2.0-LICENSE" "$DEST/face-model-LICENSE.txt"
