#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
source "$ROOT/config/build.env"
WORK="${REMOLD_WORK_ROOT:-${TMPDIR:-/tmp}/kinect-remold-build}/linux/qt"
OUTPUT="$ROOT/binaries/linux/applications/KinectRemoldQt"
command -v cmake >/dev/null 2>&1 || { echo 'CMake is required.' >&2; exit 2; }
mkdir -p "$WORK" "$OUTPUT"
cmake -S "$ROOT/applications/qt" -B "$WORK" -DCMAKE_BUILD_TYPE=Release
cmake --build "$WORK" --config Release --parallel
install -m 0755 "$WORK/KinectRemoldQt" "$OUTPUT/KinectRemoldQt"
mkdir -p "$OUTPUT/licenses/libfreenect2"
cp "$ROOT/third_party/licenses/libfreenect2/"* "$OUTPUT/licenses/libfreenect2/"
onnx_library="$(ldd "$WORK/KinectRemoldQt" | awk '/libonnxruntime\.so/ {print $3; exit}')"
[[ -f "$onnx_library" ]] || { echo 'ONNX Runtime shared library was not found for packaging.' >&2; exit 1; }
install -m 0644 "$onnx_library" "$OUTPUT/libonnxruntime.so.1"
mkdir -p "$OUTPUT/models"
fetch_model() {
  local name="$1" url="$2" expected="$3" dest="$OUTPUT/models/$1"
  if [[ -f "$dest" && "$(sha256sum "$dest" | cut -d' ' -f1)" == "$expected" ]]; then return; fi
  local temp="$WORK/$name.download"
  curl --fail --location --retry 3 --connect-timeout 15 --output "$temp" "$url"
  [[ "$(sha256sum "$temp" | cut -d' ' -f1)" == "$expected" ]] || {
    rm -f "$temp"; echo "SHA-256 mismatch: $name" >&2; exit 1;
  }
  install -m 0644 "$temp" "$dest"
  rm -f "$temp"
}
fetch_model movenet.onnx "$REMOLD_MOVENET_MODEL_URL" "$REMOLD_MOVENET_MODEL_SHA256"
fetch_model rtmpose-hand.onnx "$REMOLD_RTMPOSE_HAND_MODEL_URL" "$REMOLD_RTMPOSE_HAND_MODEL_SHA256"
fetch_model face-landmarks.onnx "$REMOLD_FACE_LANDMARKS_ONNX_URL" "$REMOLD_FACE_LANDMARKS_ONNX_SHA256"
install -m 0644 "$ROOT/third_party/models/face-blendshapes.onnx" "$OUTPUT/models/face-blendshapes.onnx"
install -m 0644 "$ROOT/third_party/models/README.md" "$OUTPUT/models/face-model-NOTICE.txt"
install -m 0644 "$ROOT/third_party/models/APACHE-2.0-LICENSE" "$OUTPUT/models/face-model-LICENSE.txt"
if [[ -n "${REMOLD_SYNSKELETON_MODEL:-}" ]]; then
  mkdir -p "$OUTPUT/models"
  install -m 0644 "$REMOLD_SYNSKELETON_MODEL" "$OUTPUT/models/SynSkeleton.onnx"
fi
echo "Qt frontend built: $OUTPUT/KinectRemoldQt"
