package org.firstinspires.ftc.teamcode.vision;

/**
 * One Limelight result reduced to plain values — the boundary between the SDK and our logic.
 *
 * <p>{@code LLResult} is built on Android's {@code org.json}, which is only a stub on the JVM,
 * so it cannot exist in a unit test. Everything past {@link LimelightSource} works on this
 * instead. No SDK imports.
 *
 * <p>One entry per detected tag: its centre in the <b>robot frame</b>, inches — +x forward,
 * +y left, +z up, origin on the floor under the robot's centre (so z is height above the
 * TILES). Parallel arrays rather than objects, so a sample is five allocations however many
 * tags it holds. Do not mutate the arrays.
 */
public final class VisionSample {

    /** The pipeline reported a target ({@code LLResult.isValid()}). */
    public final boolean valid;

    /** {@code System.currentTimeMillis()} when the driver parsed the reply — wall clock. */
    public final long parsedAtMs;

    /** Capture + targeting latency on the camera, ms. The frame is this much older than the
     *  reply. */
    public final double pipelineLatencyMs;

    public final int[] tagIds;
    public final double[] tagX, tagY, tagZ;

    public VisionSample(boolean valid, long parsedAtMs, double pipelineLatencyMs,
                        int[] tagIds, double[] tagX, double[] tagY, double[] tagZ) {
        this.valid = valid;
        this.parsedAtMs = parsedAtMs;
        this.pipelineLatencyMs = pipelineLatencyMs;
        this.tagIds = tagIds;
        this.tagX = tagX;
        this.tagY = tagY;
        this.tagZ = tagZ;
    }

    public int tagCount() {
        return tagIds.length;
    }

    /** When the shutter fired, in the wall-clock domain (architecture doc §7.4). */
    public long captureTimeMs() {
        return parsedAtMs - Math.round(pipelineLatencyMs);
    }
}
