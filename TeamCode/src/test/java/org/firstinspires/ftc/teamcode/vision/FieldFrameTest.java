package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;

import com.pedropathing.math.Pose;

import org.junit.Test;

public class FieldFrameTest {

    private static final double EPS = 1e-9;

    @Test
    public void fieldCentreIsPedroCentre() {
        Pose p = FieldFrame.toPedro(0, 0, 90);
        assertEquals(72, p.x(), EPS);
        assertEquals(72, p.y(), EPS);
        assertEquals(0, p.heading(), EPS);
    }

    @Test
    public void metresBecomeInchesAndAxesRotate() {
        // Pedro's convertToPedro: rotate −90°, so FTC +y → Pedro +x and FTC +x → Pedro −y.
        Pose p = FieldFrame.toPedro(0.0254, 0.0254 * 2, 0);
        assertEquals(72 + 2, p.x(), EPS);
        assertEquals(72 - 1, p.y(), EPS);
        assertEquals(3 * Math.PI / 2, p.heading(), EPS);    // −90° wrapped to [0, 2π)
    }

    @Test
    public void roundTripsInBothDirections() {
        double[] ftc = {1.2, -0.7, 33};
        Pose pedro = FieldFrame.toPedro(ftc[0], ftc[1], ftc[2]);
        double[] back = FieldFrame.toFtc(pedro);
        assertEquals(ftc[0], back[0], EPS);
        assertEquals(ftc[1], back[1], EPS);
        assertEquals(ftc[2], back[2], EPS);

        Pose again = FieldFrame.toPedro(back[0], back[1], back[2]);
        assertEquals(pedro.x(), again.x(), EPS);
        assertEquals(pedro.y(), again.y(), EPS);
        assertEquals(pedro.heading(), again.heading(), EPS);
    }

    @Test
    public void fieldCornersStayOnThe144InchField() {
        double half = 72 * 0.0254;
        for (double x : new double[] {-half, half}) {
            for (double y : new double[] {-half, half}) {
                Pose p = FieldFrame.toPedro(x, y, 0);
                assertEquals(0, Math.min(Math.abs(p.x()), Math.abs(p.x() - 144)), 1e-6);
                assertEquals(0, Math.min(Math.abs(p.y()), Math.abs(p.y() - 144)), 1e-6);
            }
        }
    }
}
