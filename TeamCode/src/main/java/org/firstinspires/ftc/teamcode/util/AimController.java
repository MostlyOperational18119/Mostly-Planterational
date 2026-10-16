package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Turns the whole robot to point the launcher at a target. There is no turret: the flywheel is
 * fixed to the chassis, so aiming <em>is</em> a drivetrain heading problem.
 *
 * <p>Works on a target position relative to the robot (from {@code Vision}), so it needs no
 * odometry. The control law is P plus a static-friction kick, capped:
 * {@code turn = KP·error + KS·sign(error)}, zero inside {@link #TOLERANCE}.
 *
 * <p>Vision is ~30–50 ms behind, so while the robot spins the error it sees lags reality by
 * ω × latency. Too much {@code KP} or {@code MAX_TURN} shows up as overshoot and hunting
 * around the target. {@code Robot.aimError} subtracts the heading change since the frame's
 * capture time (architecture doc §7.4); once the Pinpoint is in and that correction is live,
 * the gains can go up.
 *
 * <p>Fields are {@code public static} non-final so Panels can tune them live; the values are
 * starting points, not tuned. Pure math otherwise — no SDK.
 */
@Configurable
public final class AimController {

    /** Turn power per radian of error. */
    public static double KP = 1.0;

    /** Minimum turn power that actually moves the robot. */
    public static double KS = 0.06;

    /** Cap, well below 1, because of the vision lag above. */
    public static double MAX_TURN = 0.5;

    /** Inside this the robot counts as aimed and stops turning. */
    public static double TOLERANCE = Math.toRadians(1.5);

    /** Where the launcher points relative to robot forward, radians, CCW positive.
     *  0 = shoots forward, π = shoots backward. CAD: forward. */
    public static double LAUNCH_DIRECTION = 0;

    /** How far left of the robot centre the launcher's line of fire runs, inches (right is
     *  negative). CAD: centred. */
    public static double LAUNCHER_LEFT_OFFSET_IN = 0;

    private AimController() {}

    /**
     * How far the robot must turn (radians, CCW positive — Pedro's turn direction) so the
     * launcher's line of fire passes through a target at {@code (x, y)} inches in the robot
     * frame (+x forward, +y left). NaN if the target is inside the launcher offset.
     */
    public static double error(double x, double y) {
        double r = Math.hypot(x, y);
        double e = Math.atan2(y, x) - LAUNCH_DIRECTION;
        // An off-centre launcher fires along a line offset from the robot centre, so the
        // centre must point slightly past the target: asin(offset / range).
        e -= Math.asin(LAUNCHER_LEFT_OFFSET_IN / r);
        return Math.atan2(Math.sin(e), Math.cos(e));
    }

    /** Turn power for {@code error}, in Pedro's [−1, 1] TeleOp turn units. NaN in, NaN out. */
    public static double turn(double error) {
        if (Double.isNaN(error)) return Double.NaN;
        if (Math.abs(error) < TOLERANCE) return 0;
        double t = KP * error + Math.signum(error) * KS;
        return Math.max(-MAX_TURN, Math.min(MAX_TURN, t));
    }

    public static boolean onTarget(double error) {
        return Math.abs(error) < TOLERANCE;
    }
}
