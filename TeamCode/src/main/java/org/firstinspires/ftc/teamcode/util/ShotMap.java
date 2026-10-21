package org.firstinspires.ftc.teamcode.util;

/**
 * Launch speed by distance: a measured table, linearly interpolated. The hood is at a fixed
 * angle, so speed is the only thing that sets where the pollen lands.
 *
 * <p>Measured rather than modelled: each point is a distance and the speed that scored from
 * there, so hood angle, compression, spin and wheel slip are all in the numbers without being
 * modelled. {@code ShotCalibration} records points in this format.
 *
 * <p>Outside the measured range the nearest end is used, not extrapolated — a straight line
 * past the last point is a guess, and a wrong guess at full speed is a wild shot. Pure math,
 * no SDK.
 */
public final class ShotMap {

    private final double[] distances, rpms;

    /**
     * @param points {@code {distanceIn, rpm}} pairs, in increasing distance. May be empty:
     *               then {@link #rpmAt} is always NaN.
     * @throws IllegalArgumentException if a point is not a pair, or distances do not increase
     */
    public ShotMap(double[][] points) {
        distances = new double[points.length];
        rpms = new double[points.length];
        for (int i = 0; i < points.length; i++) {
            if (points[i].length != 2) {
                throw new IllegalArgumentException("point " + i + " is not {distanceIn, rpm}");
            }
            if (i > 0 && !(points[i][0] > points[i - 1][0])) {
                throw new IllegalArgumentException("distances must increase, at point " + i);
            }
            distances[i] = points[i][0];
            rpms[i] = points[i][1];
        }
    }

    /** No points measured yet. */
    public boolean isEmpty() {
        return distances.length == 0;
    }

    /**
     * The launch speed for a target {@code distanceIn} away. NaN if the table is empty or the
     * distance is NaN (unknown) — the caller decides the fallback.
     */
    public double rpmAt(double distanceIn) {
        int n = distances.length;
        if (n == 0 || Double.isNaN(distanceIn)) return Double.NaN;
        if (distanceIn <= distances[0]) return rpms[0];
        if (distanceIn >= distances[n - 1]) return rpms[n - 1];
        int i = 1;
        while (distances[i] < distanceIn) i++;
        double f = (distanceIn - distances[i - 1]) / (distances[i] - distances[i - 1]);
        return rpms[i - 1] + f * (rpms[i] - rpms[i - 1]);
    }
}
