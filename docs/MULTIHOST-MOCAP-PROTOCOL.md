# Multi-host motion-capture protocol

## Scope and trust boundary

One **coordinator** owns person association, calibration revision selection and
the fused articulated solve. The coordinator listens for mutually authenticated
WebSocket connections. Each **sensor host** selects a local Kinect, connects to
the coordinator and sends its observations. A host never commands another host's
USB device. Discovery is explicit rather than broadcast on untrusted networks.
The transport also permits the coordinator to dial a listening sensor host,
but the host-to-coordinator connection is the main workflow.

## Clock and calibration handshake

Before a host is admitted, it reports `hostId`, software/protocol version,
monotonic-clock sample, estimated clock offset/drift/uncertainty, and every
sensor's calibration revision. PTP is required for capture; NTP is preview-only.
The coordinator rejects an observation when its uncertainty exceeds the session
limit or when its calibration revision differs from the active room calibration.

Each sensor transform maps local camera metres into the named room frame:
`p_room = R_camera_to_room * p_camera + t_camera_to_room`. The transform, RMS
error, calibration timestamp and revision travel with the session metadata—not
with each frame.

## Observation envelope (protocol v1)

```
{
  "v": 1, "kind": "body-observation", "host": "capture-a",
  "sensor": "kinect-xbox-360-remold:serial", "seq": 821,
  "tMonoNs": 1234567890123, "clockUncertaintyNs": 900000,
  "calibrationRevision": "room-2026-11-08-1", "subject": "local-track-4",
  "joints": [{"name":"ElbowLeft","p":[x,y,z],"c":0.91,"state":"tracked"}]
}
```

Frames are ordered per sensor but may arrive out of order across hosts. The
coordinator aligns them in a bounded fusion window, associates subjects by
room-space pelvis/head prediction, then uses robust confidence-weighted residual
rejection per joint. It selects a valid unobscured observation rather than
averaging incompatible limbs. The resulting skeleton is passed to the existing
temporal IK, joint-limit, finger and foot-contact solve.

## Implementation status and required work

The Qt application implements a mutually authenticated controller/host
WebSocket link and publishes ordered local observations for remote preview.
It also has local ONNX face landmarks and blendshapes, Face Cap/VMC outputs,
JSONL and BVH recording. These local features do not imply multi-sensor
fusion. Remote observations are not merged into the local solve or exports.

Capture-grade multi-host support still requires measured clock discipline,
room extrinsic calibration and revision management, subject association,
bounded temporal fusion, and a fused articulated solve. It then needs replay
tests and synchronized physical captures with occlusion, clock fault and
receiver interoperability checks. See the [Qt developer guide](QT-DEVELOPER-GUIDE.md#4-finish-capture-grade-multi-sensor-solving)
for the implementation sequence and [Qt user guide](QT-USER-GUIDE.md#lan-sensor-sessions)
for current operation.
