# Kinect Remold Qt: developer guide

This guide describes the native Qt implementation as it exists now, with an
emphasis on capture correctness and the remaining work. The Java Studio is a
reference for workflow and features, not proof of Qt parity. The
[user guide](QT-USER-GUIDE.md) covers operation; the
[workspace note](QT-MOCAP-WORKSPACE.md) contains detailed control and output
behavior.

## Code and data flow

```text
platform USB host / driver endpoint
  -> local framed RGB, depth, IR and factory calibration
  -> StreamWorker (applications/qt/main.cpp)
  -> RGB body / hand / face ONNX inference + RGB/depth association
  -> local pose JSON -> MocapPoseSmoother -> MocapRig
  -> stage, timeline, JSONL, BVH, UDP/OSC/VMC/Face Cap, WebSocket

remote host -> mutual-TLS LanPoseLink -> controller peer preview
```

| Component | Owner | Current role |
|---|---|---|
| Device discovery and stream arbitration | `main.cpp`, native driver modules | Local camera selection and latest-window handoff. |
| Body and hand inference | `macos/OnnxBodyTracker.cpp`, `OnnxHandTracker.cpp` | RGB landmarks; depth association supplies metric joint positions. |
| Face inference | `OnnxFaceTracker.cpp` | Rotated face crop, 478 landmarks, 51 blendshape scores, orientation, quality and crop diagnostics. |
| Face surface | `FaceMeshTopology.hpp`, `main.cpp` | MediaPipe 852-triangle topology drawn over the matching video frame; poor face frames dim or suppress the fill. |
| Temporal filtering | `MocapPoseSmoother` in `main.cpp`, `MocapRig.cpp` | Confidence-aware body position filter, face landmark/expression/orientation filter, local bone rotation filter. |
| Recording and streams | `MocapBvh.cpp`, `MocapNetwork.cpp`, `MocapWebSocket.cpp` | JSONL, BVH and real-time transport. |
| LAN observation preview | `LanPoseLink.cpp` | Certificate-authenticated peer exchange and ordered remote observations; no fused output. |

The face crop retains its side length through short tracking failures. On
reacquisition, it expands if the landmark face oval occupies too much of the
crop. `cropOccupancy` above 0.72 marks the frame marginal; above 0.9 makes it
unusable. This addresses a live trace where the crop contracted to 283 pixels
and the face filled roughly 90% of it. A comparison run recovered face data
on 150/150 frames, versus 93/150 before that correction. That observation
does **not** establish geometric accuracy or general performance across
subjects and distances. The 15-second diagnostics use `REMOLD_SOLVE_CAPTURE`
and `--check-live-face-dedicated`; their temporary JSONL and PNG samples are
not part of the application package.

## Build and verification

| Platform | Primary command | Output and status |
|---|---|---|
| macOS Apple Silicon | `./Kinect-Remold-macos.sh --build-only` | Signed `.app` in `binaries/macos/applications/`. Native Kinect One/v2 live RGB/depth/IR and face/body checks were exercised on attached hardware. Xbox 360 macOS streaming remains pending. |
| Windows | `scripts/windows/Build-Qt.ps1` | Packaged `.exe` and dependencies under `binaries/windows/applications/`. Full current Qt mocap workflow still needs a physical sensor validation pass. |
| Linux | `scripts/linux/BUILD-QT.sh` | Executable, models and ONNX Runtime under `binaries/linux/applications/`. Full current Qt mocap workflow still needs a physical sensor validation pass. |

The app requires Qt 6 and ONNX Runtime at configure time. The macOS root
CMake project is CLion-friendly. Model URLs and expected SHA-256 hashes live
in `config/build.env`; macOS uses the persistent ignored
`binaries/model-cache`, while the platform packaging scripts stage models
beside the executable. Do not silently substitute an unverified model: tensor
shape, preprocessing, license and coefficient mapping are part of the runtime
contract. The optional `SynSkeleton.onnx` is not a generally available 58
landmark dependency and is not required for the RGB workflow.

Useful non-hardware checks are `--check-mocap-smoothing`,
`--check-mocap-stage`, `--check-mocap-rig`, `--check-mocap-guidance`,
`--check-face-model`, and `--check-factory-registration`. Hardware checks
include `--check-live-preview`, `--check-live-face-dedicated`,
`--check-live-body-only`, and `--check-live-mocap`. These switches run from
the Qt executable and may need a graphical desktop even when they do not
open a window. On macOS, use the bundle or a CLion Run/Debug configuration;
running the executable directly from a restricted shell can fail before the
check begins. Do not start a second hardware check while a GUI window owns
the sensor. `--check-lan-pose CERT KEY CA` exercises a local mutual-TLS link.

The latest macOS build and strict code-signature verification passed. The
body smoothing check passed after face temporal filtering was added. That
face filter has not yet been assessed with a controlled live before/after
capture. Windows and Linux package construction and end-to-end Qt captures
were not rerun in that validation pass.

## Improvement plan

### 1. Measure capture quality before tuning further

Record repeatable neutral, T pose, slow arm motion, eye blink, mouth motion,
head tilt, yaw and camera-distance sequences. Keep RGB/depth timestamps,
joint confidence, crop geometry, quality state and output pose in a replayable
fixture. Compare jitter during a still pose, lag during deliberate motion,
dropout duration and end-to-end latency. Include multiple subjects, lighting
conditions and glasses. A visual impression alone cannot establish whether a
filter improved the export.

### 2. Improve body stability and contact

The current body filter damps independent joints, followed by a rig rotation
filter. This can still produce limb-length changes, knee wobble and foot
lifting when depth samples jump. Add confidence-aware kinematic constraints
using measured segment lengths, joint limits, and a calibrated floor/contact
state. Prefer short prediction through occlusion over snapping to a new depth
sample. Test against held T poses, squats, turns and intentional foot lifts;
measure both stillness and response lag. Avoid locking feet without explicit
contact detection.

### 3. Improve face geometry and expression stability

The face filter now damps landmarks, coefficients and head quaternion, but
the raw camera annotation dots can still flutter. Reconcile preview and
export so both show the same filtered frame, while preserving raw data for
diagnostics. Validate roll, yaw and scale against labeled landmarks or a
second camera. For close/profile views, evaluate a licensed face detector or
relocalizer, perspective-aware crop fitting, geometric plausibility checks
and a short hold of the last good mesh. Presence confidence alone is not a
geometry score. Test every change with temporal image/pose comparisons; do
not make the yellow/red gate more permissive merely to show a mesh.

### 4. Finish capture-grade multi-sensor solving

The authenticated LAN path is a preview transport. Add measured clock offset
and uncertainty, synchronized capture windows, room extrinsic calibration
with revision IDs, subject association, confidence-weighted observation
selection and a fused articulated solve. Reject stale or incompatible frames.
Then validate occlusions, clock faults, reconnects and different sensor
placements with physical multi-host captures before including peers in the
quality marker, recordings or VMC. See [the protocol](MULTIHOST-MOCAP-PROTOCOL.md).

### 5. Establish cross-platform and export quality gates

Run the same sensor and output fixtures on Windows, Linux and macOS. Verify
packaged startup without a shell, model and plugin loading, sensor ownership,
RGB/depth timing, calibration, recording duration, BVH bone conventions,
Face Cap and VMC receiver interoperability. Add export replay and regression
checks before claiming parity with the Java reference. The current BVH rig
has no validated measured floor, robust foot contact or proven facial
receiver calibration.

## Contributions and diagnostics

Keep raw observations and filtered/retargeted poses distinguishable in logs
and test fixtures. Change one stage at a time, retain a baseline capture,
and compare the exported pose as well as the preview. Document model
provenance and hashes, coordinate conventions, timing, hardware model and
platform for every performance claim. The face topology carries upstream
Apache attribution in `FaceMeshTopology.hpp`; review third-party notices when
changing model or mesh sources.
