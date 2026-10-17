package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/**
 * Perfect odometry for {@link SimDrivetrain}: integrates its last robot-frame command over
 * <em>real</em> elapsed time, rotated into the field frame by the simulated heading. Real time
 * because Pedro's follower measures its own loop time with {@code System.nanoTime()}, so
 * simulated and controller time must agree.
 */
public final class SimLocalizer implements Localizer {

    /** Caps one integration step, so a GC pause does not teleport the robot. */
    private static final long MAX_STEP_NANOS = 50_000_000L;

    private final SimDrivetrain drivetrain;

    private double x, y, heading;
    private double vx, vy, omega;          // field frame
    private long lastNanos = Long.MIN_VALUE;
    private boolean nan;

    public SimLocalizer(SimDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        if (lastNanos != Long.MIN_VALUE) {
            double dt = Math.min(now - lastNanos, MAX_STEP_NANOS) / 1e9;
            double cos = Math.cos(heading), sin = Math.sin(heading);
            vx = drivetrain.vx() * cos - drivetrain.vy() * sin;
            vy = drivetrain.vx() * sin + drivetrain.vy() * cos;
            omega = drivetrain.omega();
            x += vx * dt;
            y += vy * dt;
            heading += omega * dt;
        }
        lastNanos = now;
    }

    @Override
    public MotionState state() {
        Pose pose = nan ? new Pose(Double.NaN, Double.NaN, Double.NaN) : new Pose(x, y, heading);
        return MotionState.ofVelocity(pose, new Velocity(vx, vy, omega));
    }

    @Override
    public void setPose(Pose pose) {
        x = pose.x();
        y = pose.y();
        heading = pose.heading();
    }

    @Override
    public void reset() {
        x = y = heading = 0;
    }

    /** Simulates losing localization (e.g. an odometry pod unplugged). */
    public void loseLocalization() {
        nan = true;
    }
}
