package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class HeadingHistoryTest {

    private static final double EPS = 1e-9;

    @Test
    public void emptyKnowsNothing() {
        assertTrue(Double.isNaN(new HeadingHistory().headingAt(1000)));
    }

    @Test
    public void interpolatesBetweenRecords() {
        HeadingHistory h = new HeadingHistory();
        h.record(1000, 0.0);
        h.record(1010, 0.1);
        h.record(1020, 0.3);
        assertEquals(0.0, h.headingAt(1000), EPS);
        assertEquals(0.05, h.headingAt(1005), EPS);
        assertEquals(0.2, h.headingAt(1015), EPS);
        assertEquals("after the newest: the newest", 0.3, h.headingAt(5000), EPS);
    }

    @Test
    public void tooOldIsNaNRatherThanAGuess() {
        HeadingHistory h = new HeadingHistory();
        h.record(1000, 0.0);
        h.record(1010, 0.1);
        assertTrue(Double.isNaN(h.headingAt(999)));
    }

    @Test
    public void interpolatesTheShortWayAcrossPi() {
        HeadingHistory h = new HeadingHistory();
        h.record(0, Math.PI - 0.1);
        h.record(10, -Math.PI + 0.1);       // turned 0.2 rad CCW through ±π
        assertEquals(Math.PI, Math.abs(HeadingHistory.wrap(h.headingAt(5))), EPS);
    }

    @Test
    public void wrapsAroundItsCapacity() {
        HeadingHistory h = new HeadingHistory();
        for (int i = 0; i < 1000; i++) h.record(i, i * 0.001);
        assertEquals(0.9, h.headingAt(900), EPS);
        assertTrue("fell off the end", Double.isNaN(h.headingAt(100)));
    }
}
