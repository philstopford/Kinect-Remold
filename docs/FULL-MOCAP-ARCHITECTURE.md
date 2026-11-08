# Full motion and facial capture architecture

## Capture contract

`StudioFaceFrame` is the canonical face snapshot. It carries measured head pose,
per-eye gaze, ARKit-52-compatible expression coefficients and a confidence for
every coefficient. Facial-bone rigs are derived from that canonical expression
space through versioned avatar profiles; they are never used as the capture
format. This permits one performer stream to target ARKit/VMC receivers and
different VRM, Unity, Unreal or custom facial rigs consistently.

The host must not advertise face capture unless its installed model passes the
declared input/output contract and its model licence permits redistribution.
Missing, incompatible, or low-confidence face measurements are reported as
missing; coefficients must not be invented from body landmarks.

## Face Cap / Faceit compatibility

The Faceit Blender extension installed on the development machine was used as a
protocol reference only. It is GPL-3.0, so its implementation and bundled rig
assets are not copied into Kinect Remold. Its use confirms the public ARKit-52
ordering and the established Face Cap OSC interchange shape:

- `/W`: `(index, weight)` for indexes 0 through 51 in `StudioFaceBlendshape.ARKIT_52` order;
- `/HR`: head rotation (receiver-specific Euler convention must be declared);
- `/HT`: head translation;
- `/ELR` and `/ERR`: left and right eye rotations.

The Kinect Remold sender will expose this as an explicit **Face Cap-compatible**
output profile alongside a VMC Face profile. Coordinates, rotation order and
units are profile metadata, not assumptions hidden in an avatar rig. A facial
bone profile maps named ARKit coefficients to rig controls and is authored per
avatar; it never changes the captured source coefficients.

## Multi-host body fusion

Each sensor host publishes only timestamped calibrated observations. A dedicated
coordinator performs subject association and the solve. Hosts and coordinator
use a monotonic clock disciplined by PTP where available (NTP is accepted only
for preview), and retain clock-offset uncertainty with every observation.

1. Calibrate each sensor's intrinsics and rigid transform into one room frame.
2. Continuously estimate clock offset/drift and reject frames outside the fusion
   window or with excessive uncertainty.
3. Associate observations using room-space pelvis/head position and appearance
   identity; do not merge bodies merely by arrival order.
4. Fuse only contemporaneous, confidence-weighted joints. Robust residual
   rejection chooses an unobscured view rather than averaging an occluded limb.
5. Solve articulated body and finger bones with joint limits, foot contacts and
   temporal filtering, then publish the same standard body outputs as local
   capture. Preserve source IDs, calibration revision and confidence for replay.

Network transport needs authenticated, encrypted control and data channels. Raw
RGB/depth is opt-in for diagnostics; normal operation sends calibrated landmark
observations and timing metadata to reduce LAN bandwidth and privacy exposure.

## Delivery gates

Before enabling the feature by default, validate latency/jitter, clock drift,
extrinsic-calibration error, occlusion recovery, body/hand angular error, face
pose/gaze error, and ARKit-coefficient repeatability on actual Kinect hardware.
BVH remains body-only; face recording requires a timestamped coefficient track.
