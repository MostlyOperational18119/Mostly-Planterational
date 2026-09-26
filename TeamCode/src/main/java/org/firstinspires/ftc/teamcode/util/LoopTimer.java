package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;
import java.util.function.LongSupplier;

/**
 * A rolling window of loop periods, reported as median / p95 / max.
 *
 * <p>Mean is deliberately not offered: it hides exactly the periodic spikes (GC, a stray
 * un-cached read, a telemetry flush) that break aiming (architecture doc §9). The budget is
 * <b>&le; 6 ms median</b>; when {@link #maxMs()} climbs, the near-universal causes in order of
 * likelihood are a sensor read that escaped {@code read()}, an un-gated write, a telemetry call
 * outside the throttle, or a path being built mid-routine.
 *
 * <p>No SDK imports, and the clock is injectable — this class is unit-testable as-is.
 */
public final class LoopTimer {

    private static final int WINDOW = 256;
    private static final double NANOS_PER_MS = 1e6;

    private final LongSupplier clock;
    private final long[] periods = new long[WINDOW];

    private int count;
    private int index;
    private long lastTick = Long.MIN_VALUE;
    private long peak;

    public LoopTimer() {
        this(System::nanoTime);
    }

    /** @param clock nanosecond clock; injectable so tests can drive time by hand. */
    public LoopTimer(LongSupplier clock) {
        this.clock = clock;
    }

    /** Records the period since the previous call. Call once per loop, at the same point. */
    public void tick() {
        long now = clock.getAsLong();
        if (lastTick != Long.MIN_VALUE) {
            long period = now - lastTick;
            periods[index] = period;
            index = (index + 1) % WINDOW;
            if (count < WINDOW) count++;
            if (period > peak) peak = period;
        }
        lastTick = now;
    }

    public int samples() {
        return count;
    }

    public double medianMs() {
        return percentileMs(0.50);
    }

    public double p95Ms() {
        return percentileMs(0.95);
    }

    /** Max over the rolling window. */
    public double maxMs() {
        return percentileMs(1.0);
    }

    /** Max over the whole OpMode — the one number that catches a single catastrophic stall. */
    public double peakMs() {
        return peak / NANOS_PER_MS;
    }

    public double hz() {
        double median = medianMs();
        return median > 0 ? 1000.0 / median : 0.0;
    }

    /** Allocates and sorts a copy — call at telemetry rate (~10 Hz), never every loop. */
    private double percentileMs(double fraction) {
        if (count == 0) return 0.0;
        long[] sorted = Arrays.copyOf(periods, count);
        Arrays.sort(sorted);
        int i = (int) Math.round(fraction * (count - 1));
        return sorted[i] / NANOS_PER_MS;
    }

    /** {@code loop: med 5.2ms p95 7.8ms max 14.1ms peak 21.0ms (192Hz)} */
    public String summary() {
        return String.format("loop: med %.1fms p95 %.1fms max %.1fms peak %.1fms (%.0fHz)",
                medianMs(), p95Ms(), maxMs(), peakMs(), hz());
    }
}
