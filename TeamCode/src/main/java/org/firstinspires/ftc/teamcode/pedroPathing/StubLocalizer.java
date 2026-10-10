package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

/**
 * A localizer for a chassis with no odometry: the pose never moves from where it was set, and
 * velocity is always zero. Enough for robot-centric TeleOp, where Pedro passes stick values
 * straight through to the wheels (the heading it rotates by cancels out).
 *
 * <p>Nothing that follows a path or holds a pose works on top of this — the robot never
 * "arrives", so a path drives at full power until a timeout. {@link Constants#LOCALIZER_INSTALLED}
 * is what keeps those OpModes from starting.
 */
public final class StubLocalizer implements Localizer {

    private Pose pose = new Pose();

    @Override public Pose getPose() { return pose; }
    @Override public Pose getVelocity() { return new Pose(); }
    @Override public Vector getVelocityVector() { return new Vector(); }
    @Override public void setStartPose(Pose setStart) { pose = setStart; }
    @Override public void setPose(Pose setPose) { pose = setPose; }
    @Override public void update() {}
    @Override public double getTotalHeading() { return pose.getHeading(); }
    @Override public double getForwardMultiplier() { return 1; }
    @Override public double getLateralMultiplier() { return 1; }
    @Override public double getTurningMultiplier() { return 1; }
    @Override public void resetIMU() {}
    @Override public double getIMUHeading() { return pose.getHeading(); }
    @Override public boolean isNAN() { return false; }
}
