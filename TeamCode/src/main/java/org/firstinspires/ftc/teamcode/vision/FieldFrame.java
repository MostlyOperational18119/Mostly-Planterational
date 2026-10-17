package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;

/**
 * Converts between the FTC field frame the Limelight reports in and Pedro's frame.
 *
 * <ul>
 *   <li><b>FTC</b>: origin at field centre, metres, heading in degrees.</li>
 *   <li><b>Pedro</b>: origin at a field corner (field is 0–144 in), inches, radians.</li>
 * </ul>
 *
 * <p>The mapping is the one Pedro 2's {@code FTCCoordinates.convertToPedro} applied — rotate
 * −90°, then shift the origin by (72, 72). It was reimplemented here because Pedro 2.1.2's
 * {@code convertFromPedro} also rotated by −90° (so the pair was not inverse); Pedro 3 has no
 * FTC-frame conversion at all.
 *
 * <p><b>Unverified on a field.</b> Before trusting it: park the robot at a measured spot facing
 * a known direction and compare {@code vision.fieldPose()} with the tape measure. If the axes
 * come out swapped or mirrored, fix it here — this is the only place the convention lives.
 */
public final class FieldFrame {

    public static final double METERS_TO_INCHES = 1 / 0.0254;

    private static final double HALF_FIELD_IN = 72;

    private FieldFrame() {}

    public static Pose toPedro(double xMeters, double yMeters, double yawDegrees) {
        double x = xMeters * METERS_TO_INCHES;
        double y = yMeters * METERS_TO_INCHES;
        // Pose normalises the heading to [0, 2π) itself.
        return new Pose(y + HALF_FIELD_IN, -x + HALF_FIELD_IN, Math.toRadians(yawDegrees) - Math.PI / 2);
    }

    /** Inverse of {@link #toPedro}: {@code {xMeters, yMeters, yawDegrees}}. */
    public static double[] toFtc(Pose pedro) {
        double xIn = HALF_FIELD_IN - pedro.y();
        double yIn = pedro.x() - HALF_FIELD_IN;
        return new double[] {
                xIn / METERS_TO_INCHES,
                yIn / METERS_TO_INCHES,
                Math.toDegrees(wrap(pedro.heading() + Math.PI / 2))
        };
    }

    /** To [0, 2π), Pedro's heading range. */
    private static double wrap(double radians) {
        double r = radians % (2 * Math.PI);
        return r < 0 ? r + 2 * Math.PI : r;
    }
}
