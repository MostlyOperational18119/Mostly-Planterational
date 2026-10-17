package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.HashMap;
import java.util.Map;

/**
 * A holonomic point robot standing in for Pedro's {@code Mecanum}. It keeps the last
 * robot-frame command (forward, strafe-left, CCW turn — fractions of full power, normalised
 * the way {@code Mecanum} normalises its wheel powers) and {@link SimLocalizer} integrates it.
 * No inertia: the robot moves at the commanded fraction of {@link #MAX_SPEED} immediately —
 * enough to check that routines terminate and commands hand the drivetrain over correctly,
 * not to tune Foresight.
 */
public final class SimDrivetrain implements Drivetrain {

    /** in/s at full power, forward or sideways. */
    public static final double MAX_SPEED = 60;

    /** rad/s at full power. */
    public static final double MAX_TURN_RATE = 2 * Math.PI;

    private double forward, strafe, turn;
    private boolean manual;

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        this.manual = manual;
        // Mecanum wheel powers are forward ± strafe ± turn; it scales them down together when
        // any exceeds 1, which caps |forward| + |strafe| + |turn| at 1.
        double sum = Math.abs(powers.forward()) + Math.abs(powers.strafe()) + Math.abs(powers.turn());
        double scale = sum > 1 ? 1 / sum : 1;
        forward = powers.forward() * scale;
        strafe = powers.strafe() * scale;
        turn = powers.turn() * scale;
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        // Same rule as Mecanum: the largest λ keeping every wheel of current + λ·delta in [-1, 1].
        double[] a = wheels(current), b = wheels(delta);
        double lambda = 1.0;
        for (int i = 0; i < 4; i++) {
            if (Math.abs(b[i]) < 1e-9) continue;
            double t1 = (1.0 - a[i]) / b[i];
            double t2 = (-1.0 - a[i]) / b[i];
            if (t1 >= 0 && t1 < lambda) lambda = t1;
            if (t2 >= 0 && t2 < lambda) lambda = t2;
        }
        return Math.max(0, Math.min(1, lambda));
    }

    private static double[] wheels(DrivePowers p) {
        double f = p.forward(), s = p.strafe(), t = p.turn();
        return new double[] {f - s - t, f + s + t, f + s - t, f - s + t};
    }

    @Override
    public void stop() {
        forward = strafe = turn = 0;
    }

    @Override
    public void stop(boolean brake) {
        stop();
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return 1.0 / (Math.abs(Math.cos(theta)) / xRadius + Math.abs(Math.sin(theta)) / yRadius);
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new HashMap<>();
        map.put("forward", forward);
        map.put("strafe", strafe);
        map.put("turn", turn);
        return map;
    }

    // --- for SimLocalizer and assertions; robot frame ---

    /** Forward speed, in/s. */
    public double vx() { return forward * MAX_SPEED; }

    /** Leftward speed, in/s. */
    public double vy() { return strafe * MAX_SPEED; }

    /** Turn rate, rad/s, CCW positive. */
    public double omega() { return turn * MAX_TURN_RATE; }

    /** Whether the last command came from manual (TeleOp) driving. */
    public boolean manual() { return manual; }

    public boolean isCommandingMotion() {
        return forward != 0 || strafe != 0 || turn != 0;
    }
}
