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
