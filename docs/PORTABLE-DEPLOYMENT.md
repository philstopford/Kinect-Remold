# Unpack-and-run Qt releases

These archives contain the native Qt frontend, tracking models and the Kinect v2
runtime. They do not contain the separate Processing Studio application or the
Xbox 360 system-camera/audio services. Existing Xbox 360 services can still be
used by Qt. No compiler, CMake, Python, ONNX installation or model download is
needed on the end-user machine.

## End-user instructions

| Platform | Start after extracting | One-time setup |
| --- | --- | --- |
| Linux AMD64 | Open `KinectRemoldQt/KinectRemoldQt.sh` (or run it in a terminal) | The GUI's **Sensor setup** dialog explains USB permissions. Reconnect USB after installing the rule. |
| Windows AMD64 | Open `KinectRemoldQt/KinectRemoldQt.exe` | Run `drivers/kinect-one-remold/SETUP.cmd` once and approve the administrator prompt to install the WinUSB binding and service. |
| macOS Apple Silicon | Open `KinectRemoldQt/KinectRemoldQt.app` | Allow USB accessories if macOS asks. Public releases should be Developer ID signed and notarized. |

Use external sensor power and a USB 3 data cable. **Sensor setup** is always
available in the main window. On Linux and Windows it opens once at startup if
no sensor is Ready. Linux checks USB enumeration and read/write permissions
separately, so a connected sensor with denied access gets specific instructions.
The app never elevates itself or installs USB rules silently.

Linux runs the bundled v2 helper as your user and stops it when the GUI exits.
It prefers a reachable installed service; an explicit `REMOLD_LINUX_RUNTIME_ONE`
setting takes precedence. Windows uses the installed service so the archive can
be moved without breaking that service. Installing Remold's Windows driver
changes the sensor binding; Microsoft's Kinect SDK applications may require
restoring their original driver. Removal is available through the installed
module's maintenance script.

Linux archives bundle Qt, its platform/image/TLS/multimedia plugins, ONNX Runtime,
libusb and their linked dependencies. They use the host's glibc, graphics drivers,
desktop and audio services. Build on the **oldest Linux distribution you intend
to support**: an archive made on rolling Arch is not a universal Ubuntu/Debian
binary. A distribution's file manager may require choosing “Run” for executable
shell scripts; the script can always be run from a terminal. Preserve executable
permissions when extracting the `.tar.gz` archive.

MP4 recording on Linux/Windows needs FFmpeg. Release builders can include it;
otherwise the app uses `ffmpeg` from PATH and reports if it is missing. macOS
uses AVFoundation. All three builds include the models for CPU inference;
optional CUDA requires the corresponding runtime/provider dependencies and is
not part of this archive contract.

## Release builders

Build on each target OS/architecture; these commands do not cross-compile.
Install the development dependencies described in the platform build guides.
Output archives and SHA-256 sidecars go under `binaries/releases/`.

```bash
# Linux AMD64; --ffmpeg also bundles the encoder and its dependencies.
python3 scripts/linux/PACKAGE-QT.py --ffmpeg
# Repackage already-built outputs:
python3 scripts/linux/PACKAGE-QT.py --no-build --ffmpeg
# If multiple Qt SDKs are installed, select the matching Qt 6 tool:
# python3 scripts/linux/PACKAGE-QT.py --no-build --qtpaths /path/to/qt6/bin/qtpaths

# Apple Silicon macOS:
bash scripts/macos/PACKAGE-QT.sh
# Repackage an existing bundle:
bash scripts/macos/PACKAGE-QT.sh --no-build
```

```powershell
# Windows AMD64; QtPrefix and ONNXRUNTIME_ROOT may be set as for Build-Qt.ps1.
powershell -NoProfile -File scripts/windows/Package-Qt.ps1
# Include a self-contained FFmpeg executable (or supply its dependent DLLs):
powershell -NoProfile -File scripts/windows/Package-Qt.ps1 -NoBuild -Ffmpeg C:\tools\ffmpeg.exe
```

The Windows builder includes Qt/VC runtime DLLs and the generated v2 driver
package. Public driver distribution requires an appropriately signed package;
the build's locally trusted development certificate is intended for development,
not a substitute for public driver signing. The service uses the static MSVC
runtime so its installed copy does not depend on DLLs in the extracted archive.

The macOS builder accepts `REMOLD_MACOS_SIGN_IDENTITY` (Developer ID Application)
and `REMOLD_MACOS_NOTARY_PROFILE` (a configured `notarytool` keychain profile).
When both are set, it signs a staged copy, submits the ZIP to Apple, staples the
app and recreates the final ZIP. Without them it preserves the locally signed
development bundle; Gatekeeper may block that archive on another Mac.

The Linux dependency inventory is recorded in `DEPENDENCIES.txt`. Arch license
files are collected for bundled library packages; on other build distributions,
include their corresponding dependency copyright/license material before
shipping. The model notices and repository license are included. Keep Qt and
other dependency redistribution notices with the archive.

## Release verification

Extract into a new directory containing spaces, outside the checkout. Launch
without build tools or a preinstalled ONNX/Qt SDK. Verify startup, sensor setup
without a device, USB permission guidance, RGB/depth/IR, unplug/reconnect, model
loading and app shutdown. On Windows verify first-time driver setup on a clean
machine; on macOS verify Gatekeeper on a different Mac. Check the SHA-256 sidecar
after transferring the archive.

For Linux model checks, invoke the extracted `KinectRemoldQt.sh` with
`--check-body-model`, `--check-hand-model` and `--check-face-model` separately.
`--check-live-preview` starts the bundled helper and checks RGB/depth/IR without
opening the setup dialog. These synthetic and short capture checks do not prove
tracking quality or long-duration reliability. Windows and macOS release
builders require validation on those hosts before publishing their archives.

The Linux archive was built and extracted into a directory containing spaces on
Arch AMD64. With build/runtime overrides cleared, body/hand/face model checks,
factory registration, video recording and live RGB/depth/IR acquisition passed.
The live check started and stopped the bundled helper automatically. The setup
dialog's connected/access-denied messages and copy-instructions action were
also exercised. This verifies relocation on the build host; cross-distribution
and clean-machine Windows/macOS checks remain release requirements.
