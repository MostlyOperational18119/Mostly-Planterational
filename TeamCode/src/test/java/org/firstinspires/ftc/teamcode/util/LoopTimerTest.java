package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class LoopTimerTest {

    private long now;
    private final LoopTimer timer = new LoopTimer(() -> now);

    private void tickAfterMs(double ms) {
        now += (long) (ms * 1e6);
        timer.tick();
    }

    @Test
    public void firstTickRecordsNoPeriod() {
        timer.tick();
        assertEquals(0, timer.samples());
        assertEquals(0.0, timer.medianMs(), 0.0);
    }

    @Test
    public void reportsMedianP95AndMax() {
        timer.tick();
        for (int i = 1; i <= 100; i++) tickAfterMs(i);
        assertEquals(100, timer.samples());
        assertEquals(51.0, timer.medianMs(), 1e-9);   // index round(0.5 * 99) = 50
        assertEquals(95.0, timer.p95Ms(), 1e-9);      // index round(0.95 * 99) = 94
        assertEquals(100.0, timer.maxMs(), 1e-9);
    }

    @Test
    public void windowRollsButPeakIsKept() {
        timer.tick();
        tickAfterMs(50);
        for (int i = 0; i < 256; i++) tickAfterMs(5);
        assertEquals(256, timer.samples());
        assertEquals(5.0, timer.maxMs(), 1e-9);       // the 50 ms spike has left the window
        assertEquals(50.0, timer.peakMs(), 1e-9);     // but not the OpMode-long peak
        assertEquals(200.0, timer.hz(), 1e-9);
    }
}
