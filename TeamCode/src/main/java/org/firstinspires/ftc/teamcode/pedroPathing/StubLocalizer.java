package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/**
 * A localizer for a chassis with no odometry: the pose never moves from where it was set, and
 * velocity is always zero. Enough for robot-centric TeleOp, where Pedro passes stick values
 * straight through to the wheels.
 *
 * <p>Nothing that follows a path or holds a pose works on top of this — the robot never
 * "arrives". {@link Constants#LOCALIZER_INSTALLED} is what keeps those OpModes from starting.
 */
public final class StubLocalizer implements Localizer {

    private MotionState state = MotionState.ofVelocity(Pose.zero(), Velocity.zero());

    @Override public void setPose(Pose pose) { state = MotionState.ofVelocity(pose, Velocity.zero()); }
    @Override public MotionState state() { return state; }
    @Override public void update() {}
    @Override public void reset() {}
}
