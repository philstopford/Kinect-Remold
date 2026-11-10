# Kinect Remold Qt: user guide

This guide covers the native **KinectRemoldQt** application. The older Java
SynKinect Studio has separate behavior and documentation. Current physical
capture experience is strongest on an Apple Silicon Mac with a Kinect One/v2;
Windows and Linux builds exist, but the same end-to-end capture workflow has
not been established on every sensor and platform combination.

## Install and start

| Platform | Build | Start |
|---|---|---|
| macOS, Apple Silicon | `./Kinect-Remold-macos.sh --build-only` | Open `binaries/macos/applications/KinectRemoldQt.app` in Finder. The bundle starts its USB host. |
| Windows | `scripts/windows/Build-Qt.ps1` | Run `binaries/windows/applications/KinectRemoldQt/KinectRemoldQt.exe` with its packaged files. |
| Linux | `scripts/linux/BUILD-QT.sh` | Run `binaries/linux/applications/KinectRemoldQt/KinectRemoldQt` with its packaged files. |

Build prerequisites and model downloads are described in [the main README](../README.md)
and [the developer guide](QT-DEVELOPER-GUIDE.md). On Kinect One/v2, use the
sensor's external power supply and a USB 3 connection. A hub or adapter that
negotiates below SuperSpeed cannot deliver its camera streams. The macOS
Xbox 360 USB host can discover the device, but Xbox 360 streaming is still
pending there.

## Make a capture

1. Connect the sensor and open **Motion Capture**. Select the local sensor.
   Use **Refresh** if it was connected after opening the window.
2. Select **Body + hands**, **Face**, or both. The selection takes effect when
   tracking starts. Select Face alone when working close to the camera; use
   Body + hands for full-body framing.
3. Press **Start tracking**. The video stage and large quality marker let you
   check framing from a distance. If another window owns the same sensor,
   starting here hands it to this window.
4. Press **Run fit check** before a serious recording. It gives you time to
   get into position and then samples the selected tracking sources. Follow
   its recommendation if it asks for framing or calibration.
5. In **Calibrate**, run **Guide body calibration** and/or **Guide face
   calibration** for the sources you selected. The guide uses large distance
   and pose instructions. One beep begins a capture, two confirm a completed
   pose, and three indicate that it still needs a valid pose. Make sure system
   sound is audible for the eyes-closed face step.
6. When the quality marker is green and the fit check reports readiness,
   select **Record JSONL** for full observations or **Record BVH** for the
   derived humanoid skeleton. Stop recording in the same window. You can
   enable network outputs independently before capture.

### Framing and quality

For body capture, stand far enough back to keep the head, hands, knees and
feet visible, with some room around the image edges. Avoid one limb covering
another. For face capture, keep the full face in view and mostly toward the
sensor. If the face fills much of the image or the mesh stretches, step back
slightly. Strong profile turns can defeat the single-camera face model.

**GOOD** means the local solve has the required visible evidence. **MARGINAL**
warns of weak or edge-of-frame landmarks; adjust before losing the pose.
**UNUSABLE** or **FACE LOST** means the current frame should not be trusted.
In face mode, marginal frames dim the filled surface and unusable frames hide
it. The quality marker reflects the **local** sensor; a connected LAN peer
does not improve this score or the recorded solve yet.

The **Body stability** control trades responsiveness for less jitter. Move it
right if a held pose flutters; move it left if deliberate movement feels late.
It also smooths face landmarks, head orientation and expression weights in the
current implementation. Filtering cannot recover an occluded joint or a face
that the model fitted incorrectly. The point dots may still fluctuate more
than the filled surface because the camera annotation is drawn from the raw
tracker output.

### Calibration prompts

| Guide | Requested poses | What it changes |
|---|---|---|
| Body | Full body at rest, T pose, arms at sides | Measures observed limb proportions for the current capture session. |
| Face | Relaxed face, smile, mouth open, eyes closed, turn left, turn right | Uses the relaxed pose as neutral orientation and expression baseline; other poses check response. |

Calibration is session-local and does not train either ONNX model. It does not
measure a room floor, metric eye rotation, or a full 3D head translation.
**Capture selected pose** allows a manual retry, and **Reset calibration**
clears the current window's calibration.

## Review and output

The stage offers video overlay, a video/3D split and an orbitable 3D view.
**Filled face surface** toggles the shaded 852-triangle face mesh; landmark
dots remain visible. The 3D view is in camera coordinates. The timeline holds
about 60 seconds of poses; scrubbing it does **not** rewind the video. Use
**Go live** to resume following the sensor.

| Output | Contents and limits |
|---|---|
| JSONL / WebSocket | Full body and face observations, including 478 face landmarks and measured expression scores when available. |
| BVH | Retargeted 19-bone humanoid animation. Limb offsets are fixed from the first recorded pose. No measured floor or reliable foot-contact solve is guaranteed. |
| JSON UDP / OSC UDP | Live body observations and derived humanoid messages. JSON face landmarks use a separate compact packet. |
| VMC / Face Cap OSC | Live avatar and face coefficients when the relevant source is tracked; unusable face frames are withheld. |

The face model provides 51 measured expression coefficients; `tongueOut` is
unavailable. Eye gaze values are relative expression scores rather than
calibrated eye angles. Check the target application's coordinate convention,
avatar mapping and transport settings before relying on a live stream.

## LAN sensor sessions

The controller can listen for mutually authenticated WebSocket peers. Each
remote host connects with its own local sensor, certificate, key and trust
bundle; the controller can inspect its observations. Plan for **one sensor per
host** until simultaneous local USB capture has been validated. LAN peers are
currently **preview only**. Their poses are not fused into the local solve or
included in BVH/VMC export. See [the protocol and missing fusion work](MULTIHOST-MOCAP-PROTOCOL.md).

## If something goes wrong

| Symptom | First checks |
|---|---|
| No sensor | Confirm power, cable, USB 3 negotiation, native host and **Refresh**. On macOS, start the `.app` rather than a bare executable copied out of its bundle. |
| Stream rejected or busy | Press **Stop** in other capture windows, then start in the desired window. Check the native USB host log if streaming still fails. |
| No body or face overlay | Confirm the corresponding source box was selected **before** starting, verify model files are packaged, and move fully into the video view. |
| Red or yellow quality | Follow the large guidance; check camera distance, occlusion and image edges. A face turn can have high model confidence but poor geometry. |
| Jitter or flutter | Increase **Body stability** cautiously, improve light and framing, remove occlusions, and run the relevant calibration. Review the saved data before export. |
| Face mesh shifts or warps | Step back, face the sensor more squarely and check the quality marker. Close-range and profile geometry remain known limits. Hide **Filled face surface** to inspect raw dots. |
| Debug starts without a window | Use the packaged app for ordinary use. For CLion/LLDB troubleshooting, see [the Qt workspace note](QT-MOCAP-WORKSPACE.md#clion-debug-on-macos). |

For known gaps and how they could be addressed, see
[the developer guide](QT-DEVELOPER-GUIDE.md#improvement-plan).
