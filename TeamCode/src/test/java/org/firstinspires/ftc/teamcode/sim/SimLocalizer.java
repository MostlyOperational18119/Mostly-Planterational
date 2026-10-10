package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

/**
 * Perfect odometry for {@link SimDrivetrain}: integrates its last command over <em>real</em>
 * elapsed time. Real time because Pedro's controllers read the wall clock for their
 * derivative terms, so simulated and controller time must agree.
 */
public final class SimLocalizer implements Localizer {

    /** Caps one integration step, so a GC pause does not teleport the robot. */
    private static final long MAX_STEP_NANOS = 50_000_000L;

    private final SimDrivetrain drivetrain;

    private double x, y, heading, totalHeading;
    private double vx, vy, omega;
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
            vx = drivetrain.vx();
            vy = drivetrain.vy();
            omega = drivetrain.omega();
            x += vx * dt;
            y += vy * dt;
            heading += omega * dt;
            totalHeading += omega * dt;
        }
        lastNanos = now;
    }

    @Override
    public Pose getPose() {
        return nan ? new Pose(Double.NaN, Double.NaN, Double.NaN) : new Pose(x, y, heading);
    }

    @Override
    public Pose getVelocity() {
        return new Pose(vx, vy, omega);
    }

    @Override
    public Vector getVelocityVector() {
        Vector v = new Vector();
        v.setOrthogonalComponents(vx, vy);
        return v;
    }

    @Override
    public void setStartPose(Pose pose) {
        setPose(pose);
    }

    @Override
    public void setPose(Pose pose) {
        x = pose.getX();
        y = pose.getY();
        heading = pose.getHeading();
        totalHeading = heading;
    }

    @Override public double getTotalHeading() { return totalHeading; }
    @Override public double getForwardMultiplier() { return 1; }
    @Override public double getLateralMultiplier() { return 1; }
    @Override public double getTurningMultiplier() { return 1; }
    @Override public void resetIMU() {}
    @Override public double getIMUHeading() { return heading; }
    @Override public boolean isNAN() { return nan; }

    /** Simulates losing localization (e.g. an odometry pod unplugged). */
    public void loseLocalization() {
        nan = true;
    }
}
