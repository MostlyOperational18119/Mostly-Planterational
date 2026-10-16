package org.firstinspires.ftc.teamcode.util;

/**
 * Recent robot headings, keyed by wall-clock time, so a vision measurement can be brought
 * forward from the moment its frame was captured to now (architecture doc §7.4).
 *
 * <p>A frame is ~30–50 ms old by the time it is used. If the robot turned through Δ since the
 * shutter fired, the target's bearing now is the measured bearing minus Δ — and without that
 * correction a fast aim overshoots, because it keeps turning on an error that is already gone.
 *
 * <p>Keyed on {@code System.currentTimeMillis()}, <b>not</b> {@code nanoTime()}: the Limelight's
 * timestamps are wall clock, and mixing the domains gives an offset of the device's uptime.
 * Heading only, because heading is all the aim needs: vision measures the CELLS relative to the
 * robot, never the robot's field position.
 *
 * <p>Fixed size and allocation-free; 256 entries is ~1.5 s at 6 ms per loop, far beyond
 * {@code Vision.MAX_AGE_MS}. No SDK imports.
 */
public final class HeadingHistory {

    private static final int CAPACITY = 256;

    private final long[] times = new long[CAPACITY];
    private final double[] headings = new double[CAPACITY];
    private int count;
    private int newest = -1;

    /** Call once per loop, after {@code follower.update()}, with times that never decrease. */
    public void record(long timeMs, double heading) {
        newest = (newest + 1) % CAPACITY;
        times[newest] = timeMs;
        headings[newest] = heading;
        if (count < CAPACITY) count++;
    }

    /**
     * The heading at {@code timeMs}, interpolated between the two records around it. NaN if
     * {@code timeMs} is older than everything held — the caller should then use the raw
     * measurement rather than a guess. After the newest record, the newest heading.
     */
    public double headingAt(long timeMs) {
        if (count == 0) return Double.NaN;
        if (timeMs >= times[newest]) return headings[newest];
        // Walk back from the newest: the lookups are always for the last few tens of ms.
        int later = newest;
        for (int n = 1; n < count; n++) {
            int earlier = (later - 1 + CAPACITY) % CAPACITY;
            if (times[earlier] <= timeMs) {
                long span = times[later] - times[earlier];
                if (span == 0) return headings[later];
                double f = (double) (timeMs - times[earlier]) / span;
                return headings[earlier] + f * wrap(headings[later] - headings[earlier]);
            }
            later = earlier;
        }
        return Double.NaN;
    }

    /** {@code angle} wrapped to (−π, π]. */
    public static double wrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
