package org.synkinect.studio.api;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Immutable, timestamped face capture snapshot in the ARKit-52 vocabulary. */
public final class StudioFaceFrame {
    private final String deviceId;
    private final long timestampMs, frameNumber, trackingId;
    private final boolean tracked;
    private final float confidence;
    private final float[] headPosition, headRotation, leftGaze, rightGaze;
    private final List<StudioFaceBlendshape> blendshapes;

    public StudioFaceFrame(String deviceId, long timestampMs, long frameNumber, long trackingId,
                           boolean tracked, float confidence, float[] headPosition, float[] headRotation,
                           float[] leftGaze, float[] rightGaze, List<StudioFaceBlendshape> blendshapes) {
        this.deviceId = deviceId == null ? "" : deviceId;
        this.timestampMs = timestampMs;
        this.frameNumber = frameNumber;
        this.trackingId = trackingId;
        this.tracked = tracked;
        this.confidence = clamp(confidence);
        this.headPosition = vector(headPosition, 3);
        this.headRotation = vector(headRotation, 4);
        this.leftGaze = vector(leftGaze, 3);
        this.rightGaze = vector(rightGaze, 3);
        this.blendshapes = blendshapes == null ? Collections.<StudioFaceBlendshape>emptyList()
                : Collections.unmodifiableList(new ArrayList<StudioFaceBlendshape>(blendshapes));
    }

    public String deviceId() { return deviceId; }
    /** Host-monotonic timestamp in milliseconds. */
    public long timestampMs() { return timestampMs; }
    public long frameNumber() { return frameNumber; }
    public long trackingId() { return trackingId; }
    public boolean tracked() { return tracked; }
    public float confidence() { return confidence; }
    /** Camera-space metres, or null when unavailable. */
    public float[] headPosition() { return copy(headPosition); }
    /** Camera-space quaternion x,y,z,w, or null when unavailable. */
    public float[] headRotation() { return copy(headRotation); }
    /** Normalized camera-space gaze direction, or null when unavailable. */
    public float[] leftGaze() { return copy(leftGaze); }
    public float[] rightGaze() { return copy(rightGaze); }
    public List<StudioFaceBlendshape> blendshapes() { return blendshapes; }
    public StudioFaceBlendshape blendshape(String name) {
        if (name == null) return null;
        for (StudioFaceBlendshape value : blendshapes) if (name.equals(value.name())) return value;
        return null;
    }

    private static float clamp(float value) { return !Float.isFinite(value) ? 0 : Math.max(0, Math.min(1, value)); }
    private static float[] vector(float[] value, int size) {
        return value == null || value.length != size ? null : value.clone();
    }
    private static float[] copy(float[] value) { return value == null ? null : value.clone(); }
}
