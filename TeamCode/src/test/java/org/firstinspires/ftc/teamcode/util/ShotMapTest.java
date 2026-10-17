package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotMapTest {

    private final ShotMap map = new ShotMap(new double[][] {
            {40, 2800},
            {60, 3200},
            {80, 3800},
    });

    @Test
    public void measuredPointsAreExact() {
        assertEquals(2800, map.rpmAt(40), 0.0);
        assertEquals(3200, map.rpmAt(60), 0.0);
        assertEquals(3800, map.rpmAt(80), 0.0);
    }

    @Test
    public void betweenPointsItInterpolates() {
        assertEquals(3000, map.rpmAt(50), 1e-9);
        assertEquals(3350, map.rpmAt(65), 1e-9);
    }

    @Test
    public void outsideTheRangeItHoldsTheEnds() {
        assertEquals(2800, map.rpmAt(10), 0.0);
        assertEquals(3800, map.rpmAt(200), 0.0);
    }

    @Test
    public void unknownDistanceOrEmptyTableIsNaN() {
        assertTrue(Double.isNaN(map.rpmAt(Double.NaN)));
        ShotMap empty = new ShotMap(new double[][] {});
        assertTrue(empty.isEmpty());
        assertFalse(map.isEmpty());
        assertTrue(Double.isNaN(empty.rpmAt(50)));
    }

    @Test
    public void onePointIsUsedEverywhere() {
        ShotMap one = new ShotMap(new double[][] {{55, 3100}});
        assertEquals(3100, one.rpmAt(20), 0.0);
        assertEquals(3100, one.rpmAt(90), 0.0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void unsortedDistancesAreRejected() {
        new ShotMap(new double[][] {{60, 3200}, {40, 2800}});
    }

    @Test(expected = IllegalArgumentException.class)
    public void aRepeatedDistanceIsRejected() {
        new ShotMap(new double[][] {{60, 3200}, {60, 3300}});
    }

    @Test(expected = IllegalArgumentException.class)
    public void aPointMustBeAPair() {
        new ShotMap(new double[][] {{60}});
    }
}
