# Multi-person motion capture in Qt

## One person from several sensors

Choose **One person · combine sensors** from Layout at the top of the capture window and point two or more local sensors at the same person. Enable Body + hands, Face, or both, then start tracking. The body solve uses fresh torso-aligned observations to fill missing joints or select a stronger joint observation. The face solve combines expression scores from fresh confident views and uses the clearest view for the landmark mesh and head observation. `face.sources` lists the expression contributors; `face.sourceSensor` identifies the mesh's camera. The mesh remains in that camera's image coordinates. The capture readout shows contributor counts and the mesh source. Select that sensor's row to inspect its image.

This is observation-level fusion after each sensor's ONNX inference. It does not merge RGB images or triangulate face landmarks into a shared 3D face mesh.

## Several people from several sensors

Choose **Several people · one sensor per person** from Layout at the top of the capture window, then select the first sensor and start tracking. Each ready local sensor runs its own body, hand, and face models. The capture produces one synchronized clip. `person-1` belongs to the selected sensor; subsequent IDs follow the sensor list shown when tracking starts. The current ONNX body and face models track one person per sensor.

Two Kinect v2 sensors can enumerate as Ready while failing to stream simultaneously through one USB 3 hub/controller. Put them on separate USB 3 controllers, with adequate power, before starting a combined capture. The Sensors & LAN panel reports each stream's startup or transport error.

For a shared 3D stage, align the sensors before recording:

- **Align from shared pose:** Have one person stand still and visible to all sensors, then press the button. The app maps each sensor's shoulder and hip frame into the selected sensor's space.
- **Set measured sensor placement:** Enter each other sensor's position in metres and yaw in degrees relative to the selected sensor.

Each recorded frame retains a `tracks` array with `id`, `sensor`, `space`, `tracked`, `observed`, `bodies`, and `face` for each person. A track's `space` is `room` after alignment and `camera` otherwise. Face landmarks remain normalized to their own sensor image. Aligned face heads include `pRoom` and `qRoom` alongside camera-relative values. The top-level pose remains the selected sensor's pose for existing single-person consumers.

The 3D stage draws all room-aligned bodies. Select a sensor row to inspect that sensor's video and observations. JSON UDP sends separate body and packed face packets for each person with a `person` ID. WebSocket and recorded clips retain the combined frame. The existing OSC, VMC, Face Cap, and BVH live outputs represent the selected sensor's person.

## Audio with a motion clip

The capture window lists four-channel Kinect microphone inputs that macOS exposes through CoreAudio and labels them with the matching sensor ID when their USB serials agree. For one-sensor capture, that sensor's microphone is checked by default. For local fusion or multi-person capture, all matched Kinect microphones are checked by default. Uncheck any input to omit it, or uncheck all inputs for a silent clip. **Record clip** starts the checked microphones alongside the motion recording and writes one `.audio-N.wav` per input beside the clip. Each WAV has a `.json` sidecar with its sensor ID, CoreAudio input ID, first audio receipt time, format and byte count. The list updates when inputs appear or disappear; **Refresh audio** also rescans it. macOS asks for microphone access when needed.

If the list says no four-channel Kinect input is available, the app cannot record the array from that sensor. An attached USB Audio interface by itself does not prove that CoreAudio has an active four-channel stream. The motion clip still records normally, and the UI reports that audio is unavailable.

Audio inputs in this panel belong to the machine running that capture window. In a controller layout, remote sensor-host audio is not sent over the LAN pose link. Record audio on a sensor host only if it has a CoreAudio input and use the WAV sidecar to align it later; remote audio transport and cross-machine clock synchronization remain separate work.

In **Process motion capture**, choose all people, person 1, person 2, or enter a comma-separated list such as `person-2, person-3`. Noise reduction applies to the selected tracks. Loop processing uses one shared seam and frame count so the people stay synchronized. Other tracks follow the same trim and loop timing.

## Capture across machines

On the controller machine, open **Motion Capture → Sensors & LAN**, enter its TLS certificate, private key, and trusted CA, and select **Listen as controller**. Choose **Controller · several remote people** for one sensor per person, or **Controller · combine remote sensors** when all sensors see the same person. Enable Body + hands, Face, or both, and start tracking. The controller can wait for hosts to join; its sensor table shows each incoming LAN sensor.

On each sensor host, choose a local layout, select its sensor, enable the same capture channels needed at the controller, enter that host's TLS certificate and private key plus the trusted CA, and set the controller URL (`wss://controller-host:39550/`). Select **Join controller**, then **Start tracking**. A host with several working local sensors sends an observation from each sensor. The controller records and processes the body and face observations without opening a capture window for every sensor on the controller.

For remote multi-person capture, the first observed sensor becomes `person-1`; other sensor tracks are assigned in first-observation order. Each sensor should stay assigned to one person for that take. Align a shared calibration pose, or enter measured placement in **Sensors & LAN**, for room-space body tracks. Remote fusion combines fresh body joints and expression scores using the same rules as local fusion. These are observation-level solves; the current trackers detect one person per sensor, and facial landmarks remain in the source camera's image coordinates. Controller frames use receipt time because clock synchronization between hosts is not yet available; keep LAN latency low for interaction capture.

## Linux local-fusion verification

The one-person local fusion path uses the same Qt/ONNX code on Linux, Windows
and macOS. In **Motion Capture**, choose **One person · combine local sensors**,
keep both cameras aimed at the same person, enable the capture sources and press
**Start tracking**. The sensor table should show observations from both cameras.
The “body N sensor(s)” count increases only when another camera contributes a
joint, rather than merely being connected. The solve maps joints through the
observed shoulder/hip torso frames; this is not a measured, fixed extrinsic
calibration or synchronized multi-view triangulation. Observations older than
200 ms by local receipt time are excluded from local fusion.

A diagnostic for the actual local body-fusion path is available in current
builds:

```bash
# Run with one person visible to both cameras. After up to 60 seconds of
# tracker warmup, the check measures 30 seconds of capture.
# From an extracted archive:
./KinectRemoldQt.sh --check-live-fusion-solve
# Or from a source build, with its runtime configured:
binaries/linux/applications/KinectRemoldQt/KinectRemoldQt --check-live-fusion-solve
```

Exit zero requires at least ten primary output frames and five frames with body
joints contributed by multiple cameras. The report also includes maximum face
contributors and joint counts. Missing or poorly framed people can fail this
check even when both USB streams work. The test does not require both faces to
be detected, and does not establish motion accuracy or long-duration stability.

On Arch Linux AMD64, the attached `02c4` and `02d8` sensors both streamed RGB,
depth and IR concurrently. With one person framed in both views, the warmed
30-second local-fusion check passed: 137 output poses, six poses with body
contributions from both sensors, 153 accepted secondary joints, and up to two
face contributors. No torso-alignment mismatches were reported. Earlier cropped
views produced too few usable body observations to pass. The measured result
confirms execution of the shared fusion path on Linux, not continuous two-view
contributions or calibrated motion accuracy. The solver retains stronger
existing joints instead of forcing an average of both cameras.

### CPU use during body fusion

Linux and Windows now use two inference threads per body/hand model session,
instead of allowing every sensor's sessions to allocate machine-sized thread
pools. Their worker pools also stop spinning while waiting. The face model's
explicit thread settings and expression-fusion algorithm are unchanged, as are
the existing macOS inference settings. Restart the application to use the new
settings; stopping and starting tracking in an already running older executable
does not update that executable.

The body/hands stream also skips a superseded RGB frame when a complete newer
RGB packet is already buffered. It still reads depth packets in sequence, and
never drops the last complete RGB frame merely because a newer packet is
partial. This prevents processing all buffered video after an inference delay.
Face-only capture and the macOS frame-consumption path retain their behavior.

On this 16-thread Linux host, a repeated two-camera synthetic inference workload
(20 total camera frames, each with body and two hand inferences) changed from
1.24 seconds elapsed / 17.4 CPU seconds to 0.75 seconds elapsed / 2.48 CPU seconds.
These figures exclude facial inference and USB decoding. A subsequent warmed
30-second live check passed with 170 output poses, 41 poses receiving body joints
from both cameras, and 563 accepted secondary joints. The GUI process averaged
4.1 CPU cores during that run; the separate USB runtime is additional. Framing
varied between live runs, so the joint counts are not an accuracy comparison.

To try a smaller or larger body/hand inference budget on Linux/Windows, set
`REMOLD_ONNX_THREADS` to an integer from 1 to 16 before launching; the default is
2 per session, not a global process CPU limit. This setting does not override
the face model's explicit two-thread configuration. `REMOLD_PROFILE_MOCAP=1`
prints body/hand/face/lifting timings; the GUI's Timing option shows them too.
CPU tuning does not remove the need for usable depth and visible torso joints
in both views, and does not add hardware capture synchronization.
