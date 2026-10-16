package org.firstinspires.ftc.teamcode.vision;

/**
 * Finds a cluster's centre from the tags of it that were seen.
 *
 * <p>The four tags sit on a straight line at known offsets {@code o_i} from the centre, so each
 * detected centre is {@code p_i = c + o_i·u} for the cluster centre {@code c} and the row's unit
 * direction {@code u}. A least-squares fit over two to four tags recovers both, using tag
 * <em>positions</em> only — far steadier than a single tag's orientation, which flips between
 * two solutions at range.
 *
 * <p>The fitted slope's length is the measured tag spacing divided by the true one; it should
 * be ~1. Far off means a wrong tag size on the camera, a misread ID, or a wild detection, so
 * the fit is rejected rather than trusted.
 *
 * <p>No allocation: results land in the caller's {@code out}. No SDK imports.
 */
public final class ClusterFit {

    /** Accepted range of measured/true spacing. */
    static final double MIN_SCALE = 0.75, MAX_SCALE = 1.25;

    private ClusterFit() {}

    /**
     * @param n   tags seen, ≥ 2, with distinct offsets
     * @param out receives {@code {x, y, z}} of the centre
     * @return false if the fit is rejected
     */
    public static boolean center(int n, double[] offset, double[] x, double[] y, double[] z,
                                 double[] out) {
        if (n < 2) return false;
        double om = 0, xm = 0, ym = 0, zm = 0;
        for (int i = 0; i < n; i++) {
            om += offset[i];
            xm += x[i];
            ym += y[i];
            zm += z[i];
        }
        om /= n;
        xm /= n;
        ym /= n;
        zm /= n;

        double soo = 0, sx = 0, sy = 0, sz = 0;
        for (int i = 0; i < n; i++) {
            double d = offset[i] - om;
            soo += d * d;
            sx += d * (x[i] - xm);
            sy += d * (y[i] - ym);
            sz += d * (z[i] - zm);
        }
        if (soo == 0) return false;
        double ux = sx / soo, uy = sy / soo, uz = sz / soo;
        double scale = Math.sqrt(ux * ux + uy * uy + uz * uz);
        if (scale < MIN_SCALE || scale > MAX_SCALE) return false;

        // The row is known to be unit-spaced; normalise so measurement noise in the spacing
        // does not leak into the centre.
        out[0] = xm - om * ux / scale;
        out[1] = ym - om * uy / scale;
        out[2] = zm - om * uz / scale;
        return true;
    }
}
