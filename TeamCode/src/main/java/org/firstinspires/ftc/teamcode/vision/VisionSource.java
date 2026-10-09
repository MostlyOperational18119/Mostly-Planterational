package org.firstinspires.ftc.teamcode.vision;

/**
 * Where {@link org.firstinspires.ftc.teamcode.subsystems.Vision} gets its samples — the
 * Limelight on the robot ({@link LimelightSource}), a fake in tests, or {@link #NONE} when no
 * camera is configured. Same role as {@code HubManager}: it confines the untestable class.
 *
 * <p>Both methods are called on the control thread every cycle, so they must not block.
 */
public interface VisionSource {

    /** The newest result, or {@code null} if there has never been one. Never blocks. */
    VisionSample latest();

    /** The camera has answered recently. Never blocks. */
    boolean online();

    /** OpMode exit. */
    default void close() {}

    /** No camera: always offline. */
    VisionSource NONE = new VisionSource() {
        @Override public VisionSample latest() { return null; }
        @Override public boolean online() { return false; }
    };
}
