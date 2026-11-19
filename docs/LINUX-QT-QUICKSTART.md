# Kinect v2 with the native Qt frontend on Linux

The Qt frontend used on Apple Silicon also builds on Linux AMD64. It talks to
this repository's libusb Kinect v2 service; Microsoft Kinect SDK and libfreenect2
are not required. The root Linux launcher (`Kinect-Remold.sh`) still launches
the separate Processing Studio workflow. The commands below build the native Qt
frontend and v2 runtime directly, without installing system services or the
Xbox 360/V4L2 stack.

For end-user distribution without installing Qt/ONNX packages, see
[unpack-and-run releases](PORTABLE-DEPLOYMENT.md). The dependencies below are
for building from source.

## Arch Linux dependencies

```bash
sudo pacman -S --needed base-devel cmake pkgconf libusb qt6-base qt6-websockets qt6-multimedia onnxruntime-cpu curl ffmpeg usbutils acl
```

`onnxruntime-cpu` provides ONNX Runtime for body, hand and face inference. CUDA
is optional; the CPU package is sufficient for the default configuration.
Use matching ONNX headers and libraries. Qt configuration requires the development
headers as well as the shared library; a Python `pip install onnxruntime` alone
is insufficient. Tracking weights are separate from ONNX Runtime itself. `ffmpeg` enables MP4
recording; `usbutils` and `acl` provide the USB diagnostic commands below.

If your distribution does not provide development headers, extract an official
Linux x64 ONNX Runtime SDK release and supply
`-DONNXRUNTIME_ROOT=/absolute/path/to/onnxruntime-linux-x64-VERSION` to CMake.
That directory should contain `include/onnxruntime_cxx_api.h` and
`lib/libonnxruntime.so`. A package exporting a CMake config can instead be selected
with `-Donnxruntime_DIR=/path/to/cmake/onnxruntime`.

## USB access

Connect external power and a USB 3 data port/cable. Supported v2 USB IDs are
`045e:02c4` and `045e:02d8`. `lsusb -t` should show 5000M or faster.
Run from the repository root:

```bash
sudo install -m 0644 drivers/modules/kinect-one-remold/linux/source/70-kinect-one-remold.rules /etc/udev/rules.d/70-kinect-one-remold.rules
sudo udevadm control --reload-rules
```

Unplug and reconnect USB. The `uaccess` tag grants the active local desktop user
access. The rule must run **before** systemd's `73-seat-late.rules`; a rule named
`90-...` is too late. If you installed the earlier rule, first remove only that
file with `sudo rm -f /etc/udev/rules.d/90-kinect-one-remold.rules`.
For SSH/headless operation use the installed system service or an explicit local
USB group policy; logind desktop ACLs do not grant SSH-only sessions access.

Check the device path shown by `lsusb -d 045e:` with
`getfacl /dev/bus/usb/BUS/DEVICE`. Your desktop user should have read/write access.
Do not run the Qt GUI as root.

## Build and run

After the dependency and USB setup above, the simplest route is:

```bash
./Kinect-Remold-linux-qt.sh
# Build/package without opening the GUI:
# ./Kinect-Remold-linux-qt.sh --build-only
```

The launcher builds missing artifacts, downloads verified tracking models, and
starts a temporary user-owned Kinect v2 runtime. It stops that runtime when the
GUI exits. An installed service or explicit `REMOLD_LINUX_RUNTIME_ONE` setting
takes precedence. The Qt app still uses your system Qt libraries/plugins; this
is a package for this Linux installation, not a self-contained portable AppImage.
For SDKs outside system paths, export `ONNXRUNTIME_ROOT` before building.

For development or manual runtime control, use the following commands instead.
From the repository root:

```bash
cmake -S . -B /tmp/kinect-remold-qt -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/kinect-remold-qt --parallel
```

To build just the v2 runtime without Qt or ONNX Runtime, add
`-DREMOLD_BUILD_QT=OFF`. `-DREMOLD_BUILD_KINECT_ONE=OFF` excludes the v2 service.

For preview, no tracking weights are required. For body/hand/face tracking, the
packaging script downloads the SHA-256-verified weights and stages the Qt app,
ONNX library and models together:

```bash
bash scripts/linux/BUILD-QT.sh
# An SDK outside system paths can be selected here too:
# bash scripts/linux/BUILD-QT.sh -DONNXRUNTIME_ROOT=/absolute/path/to/sdk
```

In one terminal start a temporary, user-owned runtime:

```bash
export REMOLD_LINUX_RUNTIME_ONE="$(mktemp -d /tmp/remold-one.XXXXXX)"
printf 'Runtime directory: %s\n' "$REMOLD_LINUX_RUNTIME_ONE"
REMOLD_RUNTIME_DIR="$REMOLD_LINUX_RUNTIME_ONE" /tmp/kinect-remold-qt/drivers/modules/kinect-one-remold/linux/source/kinect-one-remold
```

In another terminal copy the printed runtime directory into the export below:

```bash
export REMOLD_LINUX_RUNTIME_ONE=/tmp/remold-one.REPLACE_ME
/tmp/kinect-remold-qt/applications/qt/KinectRemoldQt
# Or run the packaged app, with tracking models:
# binaries/linux/applications/KinectRemoldQt/KinectRemoldQt
```

Close the app, stop the runtime with Ctrl-C, then remove the empty temporary
directory. Avoid running two runtimes against the same physical sensor.
With an installed v2 service, omit `REMOLD_LINUX_RUNTIME_ONE`: the Qt app defaults
to `/run/kinect-one-remold/sdk.sock`. The equivalent Xbox 360 override is
`REMOLD_LINUX_RUNTIME360`; these Linux settings do not affect macOS.

## Verification and limits

With that runtime running and the same environment exported:

```bash
QT_QPA_PLATFORM=offscreen /tmp/kinect-remold-qt/applications/qt/KinectRemoldQt --check-live-preview
echo $?
```

Exit zero means RGB, depth and infrared preview acquisition passed. Nonzero means
failure; code 2 means no Ready device with a camera endpoint. `Unavailable` can
mean missing USB permission or a failed command probe; `USB3Required` means the
connection negotiated below SuperSpeed. This short preview check does not prove
sustained frame rate, depth accuracy or tracking quality.

Check packaged tracking model loading and inference without a sensor:

```bash
QT_QPA_PLATFORM=offscreen binaries/linux/applications/KinectRemoldQt/KinectRemoldQt --check-body-model
QT_QPA_PLATFORM=offscreen binaries/linux/applications/KinectRemoldQt/KinectRemoldQt --check-hand-model
QT_QPA_PLATFORM=offscreen binaries/linux/applications/KinectRemoldQt/KinectRemoldQt --check-face-model
```

All three packaged model checks, the launcher's offscreen startup/cleanup check,
and the complete `--build-only` packaging path passed on this machine. Model
checks use synthetic input and do not establish real-world tracking quality.

On Arch Linux AMD64 with Qt 6.12, libusb 1.0.30 and `onnxruntime-cpu` 1.29,
the full build and offscreen factory-registration/video-recorder checks passed.
The attached `045e:02d8` sensor negotiated 5 Gbit/s, reported Ready, and passed
the Qt RGB/depth/IR preview check. A 20-second simultaneous raw capture delivered
291 JPEG color frames, 300 depth frames and 299 IR frames (roughly 15 fps), with
nonzero depth and IR payloads. These are short hardware checks on this machine,
not evidence of 30 fps throughput or a long-duration reliability guarantee.
A depth-only session then delivered 75 frames in five seconds; after client
disconnect and resubscription, another ten-second simultaneous session delivered
140 color, 150 depth and 149 IR frames. The runtime survived both disconnects
and stopped cleanly on SIGTERM.
macOS Apple Silicon retains its IOUSBHost backend and
`Kinect-Remold-macos.sh` workflow. Windows AMD64 retains its WinUSB service and
`scripts/windows/Build-Qt.ps1` packaging; the root CMake project also exposes the
v2 service there. Windows builds and macOS regression checks require those hosts.
