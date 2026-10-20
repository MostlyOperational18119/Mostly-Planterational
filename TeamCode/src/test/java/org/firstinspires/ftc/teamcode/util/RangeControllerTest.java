package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class RangeControllerTest {

    @Before
    public void setUp() {
        RangeController.SHOT_DISTANCE_IN = 50;
        RangeController.TOLERANCE_IN = 4;
        RangeController.KP = 0.03;
        RangeController.MAX_DRIVE = 0.4;
    }

    @After
    public void restore() {
        setUp();
    }

    @Test
    public void tooFarDrivesForwardTooCloseBacksOff() {
        assertEquals(0.03 * 5, RangeController.forward(55, 0), 1e-9);
        assertEquals(-0.03 * 5, RangeController.forward(45, 0), 1e-9);
    }

    @Test
    public void cappedFarAway() {
        assertEquals(0.4, RangeController.forward(120, 0), 0.0);
        assertEquals(-0.4, RangeController.forward(10, 0), 0.0);
    }

    @Test
    public void stopsInsideHalfTheTolerance() {
        assertEquals(0, RangeController.forward(51.5, 0), 0.0);
        assertEquals(0, RangeController.forward(48.5, 0), 0.0);
    }

    @Test
    public void fadesWithTheAimErrorAndNeverDrivesWhenFacingAway() {
        assertEquals(0.15 * Math.cos(0.5), RangeController.forward(55, 0.5), 1e-9);
        assertEquals(0, RangeController.forward(55, Math.toRadians(120)), 0.0);
        assertEquals("no aim error: treated as aimed", 0.15,
                RangeController.forward(55, Double.NaN), 1e-9);
    }

    @Test
    public void noTargetIsNaNSoTheDriverKeepsControl() {
        assertTrue(Double.isNaN(RangeController.forward(Double.NaN, 0)));
    }

    @Test
    public void inRange() {
        assertTrue(RangeController.inRange(53));
        assertTrue(RangeController.inRange(47));
        assertFalse(RangeController.inRange(55));
        assertFalse(RangeController.inRange(Double.NaN));
    }
}
