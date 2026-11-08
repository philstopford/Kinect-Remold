# Multi-host motion-capture protocol

## Scope and trust boundary

One **coordinator** owns person association, calibration revision selection and
the fused articulated solve. Each **sensor host** owns one or more local Kinect
streams and sends calibrated observations. A host never commands another host's
USB device. Control and data connections are mutually authenticated and
encrypted; discovery is explicit rather than broadcast on untrusted networks.

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

## Required implementation work

1. Implement authenticated coordinator/host transport, clock discipline and
   calibration workflow.
2. Publish local tracked observations from `BodyTrackingSession`, then implement
   fusion, association and replay tests.
3. Add a face ONNX model with redistribution licence, SHA-256, strict tensor
   validation, RGB crop/landmark inference, ARKit-52 regression and head/eye
   pose.
4. Implement Face Cap and VMC Face sinks plus the JSON rig-profile loader and
   recorder.
5. Validate with synchronized physical multi-Kinect captures, ground truth,
   occlusion scenarios, clock fault injection and receiver interoperability.
