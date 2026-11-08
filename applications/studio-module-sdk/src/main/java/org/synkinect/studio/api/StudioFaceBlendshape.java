package org.synkinect.studio.api;

/**
 * One normalized facial-expression coefficient. Names use the ARKit 52
 * blendshape vocabulary so receivers do not need a product-specific mapping.
 */
public final class StudioFaceBlendshape {
    /** Canonical ARKit-compatible coefficient order used by Face Cap and Live Link Face receivers. */
    public static final String[] ARKIT_52 = {
            "eyeBlinkLeft", "eyeLookDownLeft", "eyeLookInLeft", "eyeLookOutLeft", "eyeLookUpLeft", "eyeSquintLeft", "eyeWideLeft",
            "eyeBlinkRight", "eyeLookDownRight", "eyeLookInRight", "eyeLookOutRight", "eyeLookUpRight", "eyeSquintRight", "eyeWideRight",
            "jawForward", "jawLeft", "jawRight", "jawOpen", "mouthClose", "mouthFunnel", "mouthPucker", "mouthRight", "mouthLeft",
            "mouthSmileLeft", "mouthSmileRight", "mouthFrownRight", "mouthFrownLeft", "mouthDimpleLeft", "mouthDimpleRight",
            "mouthStretchLeft", "mouthStretchRight", "mouthRollLower", "mouthRollUpper", "mouthShrugLower", "mouthShrugUpper",
            "mouthPressLeft", "mouthPressRight", "mouthLowerDownLeft", "mouthLowerDownRight", "mouthUpperUpLeft", "mouthUpperUpRight",
            "browDownLeft", "browDownRight", "browInnerUp", "browOuterUpLeft", "browOuterUpRight", "cheekPuff", "cheekSquintLeft",
            "cheekSquintRight", "noseSneerLeft", "noseSneerRight", "tongueOut"
    };
    private final String name;
    private final float weight, confidence;

    public StudioFaceBlendshape(String name, float weight, float confidence) {
        this.name = name == null ? "" : name;
        this.weight = clamp(weight);
        this.confidence = clamp(confidence);
    }

    public String name() { return name; }
    /** Expression magnitude in [0, 1]. */
    public float weight() { return weight; }
    /** Per-coefficient measurement confidence in [0, 1]. */
    public float confidence() { return confidence; }

    private static float clamp(float value) {
        if (!Float.isFinite(value)) return 0;
        return Math.max(0, Math.min(1, value));
    }
}
