package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ClusterFitTest {

    private static final double[] OFFSETS = {-6.5, -2.75, 2.75, 6.5};
    private final double[] out = new double[3];

    /** Tags of a cluster centred at c, row along unit u, spacing scaled by {@code scale}. */
    private boolean fit(double[] c, double[] u, double scale, int... indices) {
        int n = indices.length;
        double[] o = new double[n], x = new double[n], y = new double[n], z = new double[n];
        for (int k = 0; k < n; k++) {
            o[k] = OFFSETS[indices[k]];
            x[k] = c[0] + scale * o[k] * u[0];
            y[k] = c[1] + scale * o[k] * u[1];
            z[k] = c[2] + scale * o[k] * u[2];
        }
        return ClusterFit.center(n, o, x, y, z, out);
    }

    @Test
    public void allFourTagsRecoverTheCentre() {
        double[] c = {60, -8, 48};
        assertTrue(fit(c, new double[] {0, 1, 0}, 1, 0, 1, 2, 3));
        assertArrayEquals(c, out, 1e-9);
    }

    @Test
    public void twoTagsOnOneSideStillFindTheCentre() {
        // Only the two leftmost tags in view: their mean is 4.6 in off-centre; the fit is not.
        double[] c = {40, 10, 35};
        double s = Math.sqrt(0.5);
        assertTrue(fit(c, new double[] {s, s, 0}, 1, 0, 1));
        assertArrayEquals(c, out, 1e-9);
    }

    @Test
    public void aTiltedRowIsHandled() {
        // The HIVE tips the row's plane, and a robot viewing it obliquely sees the row at an
        // angle in all three axes.
        double[] c = {55, 20, 49};
        double[] u = {0.48, 0.80, 0.36};
        double len = Math.sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        for (int i = 0; i < 3; i++) u[i] /= len;
        assertTrue(fit(c, u, 1, 1, 2, 3));
        assertArrayEquals(c, out, 1e-9);
    }

    @Test
    public void wrongSpacingIsRejected() {
        // E.g. the pipeline configured for the wrong tag size: everything scales by the ratio.
        assertFalse(fit(new double[] {60, 0, 48}, new double[] {0, 1, 0}, 1.5, 0, 1, 2, 3));
        assertFalse(fit(new double[] {60, 0, 48}, new double[] {0, 1, 0}, 0.5, 0, 1, 2, 3));
    }

    @Test
    public void oneTagIsNotEnough() {
        assertFalse(fit(new double[] {60, 0, 48}, new double[] {0, 1, 0}, 1, 2));
    }
}
