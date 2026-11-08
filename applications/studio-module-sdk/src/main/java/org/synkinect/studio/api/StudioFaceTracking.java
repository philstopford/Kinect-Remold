package org.synkinect.studio.api;

import java.util.function.Consumer;

/** Host-owned face-capture session for one selected RGB-D device. */
public interface StudioFaceTracking extends AutoCloseable {
    String deviceId();
    boolean live();
    /** Delivers latest-only immutable frames; slow consumers never block capture. */
    AutoCloseable subscribe(Consumer<StudioFaceFrame> consumer);
    @Override void close();
}
