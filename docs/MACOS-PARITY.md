# macOS Kinect One/v2 parity

The Apple Silicon runtime uses IOUSBHost and a Qt 6 application. It has no
libusb or JDK dependency. This page distinguishes hardware transport parity
from SynKinect Studio feature parity.
Kinect One identities use the USB serial when available, so changing USB-C
ports does not change the device ID; port location is a fallback.

| Capability | Windows/Linux Kinect One | macOS Kinect One |
|---|---|---|
| SuperSpeed USB discovery and device identity | Yes | `02c4` (1520) and `02d8` (1565) share the native transport; both have delivered frames |
| RGB, metric depth, infrared | Yes | Yes; RGB/depth and IR/depth tested together |
| Concurrent RGB, depth, infrared | Yes | Yes; all three modes tested together |
| Factory depth and color calibration reply | Yes | Yes; extended reply tested |
| Device-scoped SDK `LIST`, `GET`, `CONTROL` | Yes | Yes |
| Legacy 16-byte and device-scoped 80-byte camera requests | Yes | Yes |
| Kinect v2 audio | Conditional on a four-channel OS input | No matching CoreAudio input has been validated |
| Processing Studio modules | Yes, with their models and dependencies | Qt has partial native Surveillance, Microphones, Acoustic Scanner, and Sensor Calibration windows |
| Body, hand, and face landmarks | Studio ONNX tracking with metric 3D output | Native ONNX MoveNet body and RTMPose Hand 2D previews, plus Apple Vision hand/face previews; Body + hands saves depth-backed camera-space JSON using the factory Kinect One color polynomial when available, but does not publish a Studio-compatible body session |
| Frame and depth-cloud export | Scanner uses calibrated depth | Single-frame PNG, metric-depth PLY and depth-surface OBJ export; saved per-device wall correction is applied |
| Manual video recording | Studio modules write module-specific recordings | Selected Qt preview records to MP4 on macOS |
| Scanner fusion, surveillance, interaction, motion capture | Studio-dependent | Qt has TSDF fusion, moving-camera ICP, a constrained turntable path, RGB vertex color on Kinect One, mesh preview and OBJ/PLY/STL export, multi-camera RGB/IR motion detection and pre-roll event video, plus Motion Capture JSON UDP, OSC UDP, WebSocket, VMC, JSONL and BVH. Loop closure, Refine HQ and interaction remain unported |
| Multiple simultaneous Kinect One sensors | Per-device sessions | Device-scoped session ownership and bounded concurrent clients implemented; simultaneous hardware streams not validated |

On the connected `045e:02d8` sensor, a Release build delivered 296 depth and
142 RGB frames over ten seconds. These are observed rates, not guaranteed frame
rates. The sensor negotiated 5 Gbit/s through the USB-C adapter. The model
1520 replacement enumerated as `045e:02c4` at 5 Gbit/s. A Mac freeze and
`AppleUSBXHCI` panic occurred while it was connected, but the report did not
identify a Kinect driver request as the cause. The model-specific block
introduced afterward has been removed. Model 1520 now follows the same
control, calibration and streaming path as model 1565 (`045e:02d8`). A fresh
end-to-end run on model 1520 is still needed to measure long-run reliability and
frame rates. A brief native service check on 1520 delivered 38 JPEG frames over
about three seconds and a separate metric depth frame at 512×424. One immediate
depth subscription following the RGB session returned a transient error; a
separate retry succeeded. Sustained and concurrent streaming remain unverified
on 1520.

The macOS camera driver is functional for imaging. Full Studio parity requires
native Qt implementations of the remaining Processing modules, plus their
model assets and dependencies. The Qt app currently provides discovery,
RGB/depth/IR previews, PNG frame export, corrected metric-depth PLY and
single-frame OBJ export, wall calibration with independent validation,
manual MP4 preview recording, multi-camera Surveillance, four-channel
Microphones and Acoustic Scanner where an audio endpoint is available, native
ONNX MoveNet body landmarks, and Apple Vision hand and face landmark overlays.
The Qt Body + hands preview also uses the same pinned MoveNet and RTMPose Hand
models downloaded by the reference builders. The separate trained
`SynSkeleton.onnx` depth model is not one of those downloaded assets.
The Qt Motion Capture window streams depth-backed observed joints in camera
space. Its WebSocket server binds to loopback and emits one JSON text frame per
pose. The Kinect One transport now provides its factory color polynomial to
Qt. A live model 1520 reply had valid coefficients, projected the depth center
at 2 m to RGB (974, 563), and produced tracked poses. Precise registration
error has not yet been measured against a target. Qt derives a 19-bone humanoid pose from observed joints
and offers VMC packets, Unity-space JSON, mirrored avatar output and BVH
recording. The native retargeter now uses shoulder and hip axes for torso twist,
limb bend planes and hand thumb landmarks for limb twist, learns observed
segment lengths, and smooths local rotations across frames. The camera pose
does not supply feet or a floor plane, so foot contact and full Java
retargeting fidelity remain unverified. Measured registration and retargeting
quality remain limits for every rig-based output.
The Qt app now runs a cross-platform ONNX 478-point face model and MediaPipe
blendshape regressor within Motion Capture. Full facial landmarks and 51
measured expression coefficients are recorded with pose timestamps, and Face
Cap/VMC blendshape packets are available. ARKit `tongueOut`, validated gaze,
and measured head-pose accuracy remain open.
Live Kinect One face-only checks observed 149/150 neutral frames and 96/150
frames during a strong left head turn after adding a reusable face crop. Median
estimated yaw changed from about -6 degrees to -62 degrees. The profile run
still had frequent short tracking gaps and occasional implausible yaw values;
profile continuity and head-pose quality need further work before production
facial capture. The diagnostic face tracks were deleted after analysis.

The newer Qt Motion Capture workspace adds a video/3D overlay stage, a
60-second in-memory pose timeline, a sensor table and mutual-TLS LAN observation
preview. Remote sensor poses are not fused. The LAN path still needs PTP,
cross-sensor room calibration and association before it can produce a
capture-grade multi-Kinect recording.
