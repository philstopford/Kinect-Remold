#!/usr/bin/env bash
set -euo pipefail
[[ "$(uname -s)" == Darwin && "$(uname -m)" == arm64 ]] || { echo 'Apple Silicon macOS is required.' >&2; exit 2; }
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
BUILD="${REMOLD_WORK_ROOT:-${TMPDIR:-/tmp}/kinect-remold-build}/macos/qt"
OUTPUT="$ROOT/binaries/macos/applications"
command -v cmake >/dev/null 2>&1 || { echo 'CMake is required.' >&2; exit 2; }
command -v macdeployqt >/dev/null 2>&1 || { echo 'Qt 6 with macdeployqt is required.' >&2; exit 2; }
command -v brew >/dev/null 2>&1 || { echo 'Homebrew ONNX Runtime is required.' >&2; exit 2; }
ONNX_ROOT="$(brew --prefix onnxruntime)"
[[ -f "$ONNX_ROOT/lib/libonnxruntime.1.dylib" ]] || { echo 'Install onnxruntime with Homebrew.' >&2; exit 2; }
OPENSSL_ROOT="$(brew --prefix openssl@4)"
[[ -f "$OPENSSL_ROOT/lib/libssl.4.dylib" && -f "$OPENSSL_ROOT/lib/libcrypto.4.dylib" ]] || {
  echo 'Install openssl@4 for LAN mutual TLS.' >&2; exit 2;
}
mkdir -p "$BUILD" "$OUTPUT"
cmake -S "$ROOT/applications/qt" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64
# macdeployqt rewrites the linked binary inside the build bundle. Re-link it
# from clean objects so subsequent packaging runs see the original Qt paths.
cmake --build "$BUILD" --config Release --clean-first --parallel
APP="$BUILD/KinectRemoldQt.app"
[[ -d "$APP" ]] || { echo "Qt app bundle missing: $APP" >&2; exit 3; }
mkdir -p "$APP/Contents/Resources"
mkdir -p "$APP/Contents/Resources/licenses/libfreenect2"
cp "$ROOT/third_party/licenses/libfreenect2/"* "$APP/Contents/Resources/licenses/libfreenect2/"
cat > "$APP/Contents/Resources/qt.conf" <<'QTCONF'
[Paths]
Prefix = ..
Plugins = PlugIns
QTCONF
rm -rf "$APP/Contents/Frameworks" "$APP/Contents/PlugIns"
macdeployqt "$APP" -always-overwrite -no-plugins -no-strip -no-codesign
QT_PLUGINS="$(qtpaths --plugin-dir)"
mkdir -p "$APP/Contents/PlugIns/platforms" "$APP/Contents/PlugIns/imageformats" "$APP/Contents/PlugIns/tls"
cp "$QT_PLUGINS/platforms/libqcocoa.dylib" "$APP/Contents/PlugIns/platforms/"
cp "$QT_PLUGINS/platforms/libqoffscreen.dylib" "$APP/Contents/PlugIns/platforms/"
cp "$QT_PLUGINS/imageformats/libqjpeg.dylib" "$APP/Contents/PlugIns/imageformats/"
cp "$QT_PLUGINS/tls/libqsecuretransportbackend.dylib" "$APP/Contents/PlugIns/tls/"
cp "$QT_PLUGINS/tls/libqopensslbackend.dylib" "$APP/Contents/PlugIns/tls/"
mkdir -p "$APP/Contents/Frameworks"
cp -L "$ONNX_ROOT/lib/libonnxruntime.1.dylib" "$APP/Contents/Frameworks/libonnxruntime.1.dylib"
cp -L "$OPENSSL_ROOT/lib/libssl.4.dylib" "$APP/Contents/Frameworks/libssl.4.dylib"
cp -L "$OPENSSL_ROOT/lib/libcrypto.4.dylib" "$APP/Contents/Frameworks/libcrypto.4.dylib"
# Homebrew binaries may already carry signatures. Clear them before changing
# load paths; the completed bundle is signed after all edits.
for binary in "$APP/Contents/PlugIns/tls/libqopensslbackend.dylib" \
              "$APP/Contents/Frameworks/libssl.4.dylib" \
              "$APP/Contents/Frameworks/libcrypto.4.dylib" \
              "$APP/Contents/Frameworks/libonnxruntime.1.dylib"; do
  codesign --remove-signature "$binary" 2>/dev/null || true
done
install_name_tool -change "$OPENSSL_ROOT/lib/libssl.4.dylib" \
  @loader_path/../../Frameworks/libssl.4.dylib "$APP/Contents/PlugIns/tls/libqopensslbackend.dylib"
install_name_tool -change "$OPENSSL_ROOT/lib/libcrypto.4.dylib" \
  @loader_path/../../Frameworks/libcrypto.4.dylib "$APP/Contents/PlugIns/tls/libqopensslbackend.dylib"
install_name_tool -change "$(otool -L "$OPENSSL_ROOT/lib/libssl.4.dylib" | awk '/libcrypto\.4\.dylib/ {print $1; exit}')" \
  @loader_path/libcrypto.4.dylib "$APP/Contents/Frameworks/libssl.4.dylib"
install_name_tool -id @rpath/libssl.4.dylib "$APP/Contents/Frameworks/libssl.4.dylib"
install_name_tool -id @rpath/libcrypto.4.dylib "$APP/Contents/Frameworks/libcrypto.4.dylib"
install_name_tool -change "$ONNX_ROOT/lib/libonnxruntime.1.dylib" \
  @executable_path/../Frameworks/libonnxruntime.1.dylib "$APP/Contents/MacOS/KinectRemoldQt"
install_name_tool -id @rpath/libonnxruntime.1.dylib "$APP/Contents/Frameworks/libonnxruntime.1.dylib"
rm -rf "$OUTPUT/KinectRemoldQt.app"
cp -R "$APP" "$OUTPUT/KinectRemoldQt.app"
install -m 0755 "$ROOT/binaries/macos/drivers/native-usb/kinect-remold-native-usb" \
  "$OUTPUT/KinectRemoldQt.app/Contents/MacOS/kinect-remold-native-usb"
bash "$ROOT/scripts/macos/FETCH-MODELS.sh" "$OUTPUT/KinectRemoldQt.app/Contents/Resources/models"
if [[ -n "${REMOLD_SYNSKELETON_MODEL:-}" ]]; then
  install -m 0644 "$REMOLD_SYNSKELETON_MODEL" \
    "$OUTPUT/KinectRemoldQt.app/Contents/Resources/models/SynSkeleton.onnx"
fi
codesign --force --deep --sign - "$OUTPUT/KinectRemoldQt.app"
QT_QPA_PLATFORM=offscreen QT_PLUGIN_PATH="$OUTPUT/KinectRemoldQt.app/Contents/PlugIns" \
  "$OUTPUT/KinectRemoldQt.app/Contents/MacOS/KinectRemoldQt" --check-body-model
echo "Native Qt app built: $OUTPUT/KinectRemoldQt.app"
