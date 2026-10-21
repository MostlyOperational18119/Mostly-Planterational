package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Drives the robot to the distance it scores from. The flywheel runs at one fixed power, so
 * there is one range the shot goes in from; the robot closes on it instead of the flywheel
 * changing speed.
 *
 * <p>Works on the vision distance to the up CELL, so like {@link AimController} it needs no
 * odometry or field coordinates. It drives robot-forward, which is toward the target once
 * aimed (the launcher fires forward), and fades out with the aim error so a robot still turning
 * onto the target does not drive off sideways. The control law is P, capped, zero inside half
 * the tolerance.
 *
 * <p>Fields are {@code public static} non-final so Panels can tune them live; the values are
 * starting points, not tuned. Pure math otherwise — no SDK.
 */
@Configurable
public final class RangeController {

    /**
     * Distance the fixed-power shot scores from: floor distance to the up CELL's tag cluster,
     * inches, as {@code Vision.targetDistance} measures it. Found on the robot by parking where
     * shots went in at {@code Outtake.TARGET_RPM}; retune the two together.
     */
    public static double SHOT_DISTANCE_IN = 42;

    /** Within this of {@link #SHOT_DISTANCE_IN} the robot counts as in range. */
    public static double TOLERANCE_IN = 4;

    /** Forward power per inch of range error. */
    public static double KP = 0.03;

    /** Cap, low because the vision distance is ~30–50 ms old. */
    public static double MAX_DRIVE = 0.4;

    private RangeController() {}

    /**
     * Forward power (Pedro's TeleOp units, + toward the target when aimed) to bring a target
     * {@code distanceIn} away to {@link #SHOT_DISTANCE_IN}. NaN when the distance is NaN (no
     * target), so the caller can fall back to the driver.
     *
     * @param aimError radians still to turn; forward power is scaled by its cosine, and zero
     *                 when facing more than 90° away. NaN is treated as aimed.
     */
    public static double forward(double distanceIn, double aimError) {
        if (Double.isNaN(distanceIn)) return Double.NaN;
        double error = distanceIn - SHOT_DISTANCE_IN;
        if (Math.abs(error) < TOLERANCE_IN / 2) return 0;
        double f = Math.max(-MAX_DRIVE, Math.min(MAX_DRIVE, KP * error));
        return Double.isNaN(aimError) ? f : f * Math.max(0, Math.cos(aimError));
    }

    public static boolean inRange(double distanceIn) {
        return Math.abs(distanceIn - SHOT_DISTANCE_IN) < TOLERANCE_IN;     // false for NaN
    }
}
