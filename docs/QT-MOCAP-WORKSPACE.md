# Qt Motion Capture workspace

## CLion Debug on macOS

Open the repository root as the CMake project and select the `KinectRemoldQt`
Debug configuration. macOS must allow CLion under **System Settings → Privacy &
Security → Developer Tools**. If LLDB leaves the app stopped before its window
appears, check `DevToolsSecurity -status`; enabling Developer Mode requires
administrator authorization. Restart macOS after changing these permissions,
then reopen CLion. A healthy Debug launch opens the GUI and responds to Pause
with a stack at `main()` in `applications/qt/main.cpp`.

CLion Run and the packaged `.app` do not require debugger authorization.
The macOS packaging script stores SHA-256-verified inference models in
`binaries/model-cache` so a reboot does not force another download. Set
`REMOLD_MODEL_CACHE` to use another persistent cache directory.

Open **Motion Capture** in the native Qt application, choose a ready camera,
select **Body + hands**, **Face**, or both, and press **Start tracking**.
Face-only mode uses RGB and records facial observations without a body solve;
body-only mode skips the face model. Restart tracking after changing the source
switches. The stage can show a video overlay, a
video/3D split view, or the orbitable 3D view. Body and hand points are drawn
only when the model and metric depth observed them. The 3D stage is in camera
metres; it is not a calibrated multi-sensor room view.
Starting a camera stream in another window hands that sensor to the latest
window automatically. A message in the previous window reports the handoff.
Motion Capture applies confidence-aware joint smoothing with a small hold zone
for stationary joints and limits single-frame position jumps before drawing,
retargeting, streaming, or recording a pose. Deliberate movement is followed
more closely than a still pose. Move the **Body stability** slider right for a steadier solve at
the cost of response speed. This reduces jitter but does not restore occluded joints or replace
accurate RGB/depth registration.
The RGB-D association keeps a recent person-depth reference and favors leg
samples near the torso depth. Brief gaps between RGB and depth frames retain
the last valid solve; a sustained depth loss still reports unusable tracking.
The retargeted head follows face landmark roll in combined mode and the ear
line in body-only mode. Missing ankles briefly retain their previous rig
positions instead of moving a foot to its rest offset.
The **Filled face surface** switch draws MediaPipe's 852-triangle face mesh
from the tracked 3D landmarks over the video in face-only or combined mode.
The tracked landmark dots remain in the preview; turn off the surface when you
need an unobstructed view of them. The face quality indicator dims the surface
on marginal frames and withholds it on unusable frames. Strong profile turns
can still cause the single-camera landmark model to lose the face.
The stability control also filters face landmarks, expression weights, and
head rotation over time. Larger movements receive a faster response, while
small frame-to-frame changes are damped.
The large **GOOD / MARGINAL / UNUSABLE** marker reports local body fit. Green
requires all major limb joints, adequate confidence, and room around the frame
edges. Yellow warns about a missing or weak limb or a limb near the edge; red
means too many joints are missing for a useful body solve. The default stage
view shows the full video overlay for framing. The quality state and reason
are also included in local pose JSON. LAN peers remain preview-only, so this
marker does not yet claim multi-sensor fusion quality.
In face-only mode the marker instead reports **FACE GOOD** or **FACE LOST**.
It also reports **FACE MARGINAL** when landmark confidence, head yaw, or roll
approaches the useful range of the single-camera face model. An extreme turn
or tilt is marked unusable: the filled overlay is hidden, the face is flagged
untracked, and Face Cap/VMC face packets are withheld. A high face-presence
score alone does not
prove that the eye and mouth landmarks fit a profile view. Keep the face
mostly toward the camera for expression capture.

The **pose timeline** holds the latest 60 seconds (up to 1,800 poses). Drag the
slider to inspect a pose, press **Play** to replay buffered poses, or **Go live**
to follow capture again. The timeline buffers poses in memory; video is live
and is not synchronized to a scrubbed pose. Use **Record JSONL** to save pose
samples or **Record BVH** for the derived 19-bone humanoid animation. The
selected **Unity JSON** mode applies to JSON UDP, WebSocket and JSONL; plain
OSC retains its camera joints and adds humanoid bone messages. **Mirror avatar**
applies to the derived humanoid outputs.
The same session also estimates 478 face landmarks and 51 measured expression
coefficients when a face is visible. The **Calibrate** side tab has separate
**Guide body calibration** and **Guide face calibration** runs. Each starts
tracking if necessary, checks the user's size and framing in the camera, and
shows large **MOVE CLOSER**, **MOVE FARTHER**, **CENTER**, or **HOLD POSITION**
directions. After the framing check passes, the large banner names the next
pose and gives five seconds to prepare. Sampling begins with one system beep;
two beeps confirm success and automatically advance to the next pose. Three
beeps warn that the pose is not yet detected. The guide keeps waiting, so the
eyes-closed step can be completed without looking at the screen. The pose list
shows progress; **Capture selected pose** remains available for a manual retry.

Body calibration checks standing fully in view, a T pose, and arms at sides
while the retargeter learns observed segment lengths. Face calibration checks
a relaxed face, smile, open mouth, closed eyes, and left/right turns. The
relaxed-face step collects 50 confident frames, subtracts neutral expression
levels and makes the neutral head orientation the relative zero for JSON,
Face Cap, VMC, WebSocket and recordings. The other face steps verify response;
they do not train the ONNX model. Calibration lasts for the current Motion
Capture window and does not calibrate metric head translation or eye angles.

**Run fit check** gives eight seconds to get into position, then evaluates 60
frames for the selected capture sources. Its large result recommends framing
changes or the Body or Face guide when needed, and reports **READY TO RECORD**
after fit and guided calibration checks pass. JSONL and BVH recording use one
beep on start and two on stop. System sound must be audible.
**Face Cap OSC** sends `/W` coefficients
on its configured port; **VMC** sends body bones and face blendshapes on the
VMC port. JSONL and WebSocket retain the full landmark array. JSON UDP sends
a body pose and a separate `face-landmarks` packet with base64 encoded
`u16le-uvz` coordinates. `u` and `v` use the full unsigned 16-bit range for
0–1 image coordinates; `z` uses the range -1–1. The source MediaPipe model
does not infer `tongueOut`, so the Face Cap slot for it is zero and should be
treated as unavailable. Face head orientation is estimated from eye and
vertical facial axes; metric head translation is included only when RGB and
depth are paired. `eyeGaze` contains differences between MediaPipe's eye-look
scores in image-right and image-up directions; these are relative expression
scores, not calibrated eye rotation angles. Face Cap `/HR`, `/HT`, `/ELR` and
`/ERR` are not emitted until the corresponding pose conventions are validated.
The Qt rig uses observed hip and shoulder side axes for torso twist, limb bend
planes and hand thumb landmarks for limb twist, observed segment lengths for
the exported local offsets, and temporal smoothing of local rotations. The
camera pose does not provide foot points or a measured floor plane; foot
contact and full Java retargeting fidelity remain unverified.
BVH freezes the learned offsets from the first recorded pose in its hierarchy,
as required by the BVH format.

## LAN preview

The sensor table distinguishes the local USB sensor from authenticated LAN
peers. The controller enters a PEM certificate, its PEM private key and a PEM
trust bundle containing the sensor-host CA, then presses **Listen as controller**.
The listener requires client certificates and serves `wss://<controller>:39550/`.
Each sensor host selects its local Kinect, enters its own certificate, private
key, trust bundle and controller URL, then presses **Join controller** and
**Start tracking**. The URL host must match the controller certificate. The
controller accepts several hosts. Click a LAN
sensor row to inspect its received 3D observations. Sensor identity combines
the claimed host, the authenticated certificate fingerprint and device ID.
The Qt tracking worker opens one selected sensor per window. The device-scoped
drivers permit more than one device on a host, but simultaneous local capture
has not been validated; one sensor per host is the supported planning model.

LAN messages use the `body-observation` v1 envelope described in
[multi-host protocol](MULTIHOST-MOCAP-PROTOCOL.md). They contain observed joint
positions and confidence, not raw RGB/depth. The sender currently declares
1-second clock uncertainty and an unconfigured room calibration revision;
therefore remote poses are **preview-only**. They are never merged into local
tracking, BVH or VMC. Capture-grade fusion still requires PTP clock discipline,
room extrinsics, calibration revision selection, subject association and
cross-sensor validation. Using two local sensor rows or connected peers does
not imply those conditions are met.
The receiver preserves source sequence, monotonic capture time, clock uncertainty
and calibration revision and discards out-of-order observations.
The transport supports both controller-to-host and host-to-controller delivery;
the latter is the recommended session topology.

The test switches `--check-mocap-stage`, `--check-mocap-window` and
`--check-mocap-rig` verify stage painting, layout and rig/file format without
hardware. `--check-live-mocap` validates the attached RGB-D transport,
timeline, overlays and outputs. `--check-lan-pose CERT KEY CA` runs a local
mutual-TLS sender/receiver check with a test identity whose certificate covers
`localhost`.

On Kinect One, the Qt app requests the full factory color calibration reply.
`--check-factory-registration` tests the polynomial and depth-dependent color
shift with synthetic parameters; `--check-color-calibration` checks the live
reply and a plausible center projection. A measured color/depth target is
still needed to establish absolute registration error.
