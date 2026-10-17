package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.PathTracker;

import java.util.Collections;
import java.util.Map;

/**
 * The follower's algorithm before Foresight is tuned — the same role {@link StubLocalizer}
 * plays for a chassis without odometry. Foresight cannot be constructed without AutoTune's
 * measurements, and Pedro 3's {@code Follower} needs an algorithm, so this lets the follower
 * exist for manual driving, which never consults it.
 *
 * <p>Following or holding commands <b>zero power</b> and never reports arrival: a path or
 * {@code turnTo} that slips past the {@link Constants#FORESIGHT_TUNED} guards sits still until
 * its timeout, rather than driving on guessed constants.
 */
public final class UntunedAlgorithm implements Algorithm {

    @Override
    public DrivePowers calculatePath(Drivetrain drivetrain, PathTracker pathTracker,
                                     MotionState state, double deltaTime) {
        return DrivePowers.zero();
    }

    @Override
    public DrivePowers calculateHold(Drivetrain drivetrain, Pose target, MotionState state,
                                     boolean useScaling, double deltaTime) {
        return DrivePowers.zero();
    }

    @Override public double completion() { return 0; }
    @Override public Pose closestPose() { return Pose.zero(); }
    @Override public Vector2D closestTangent() { return Vector2D.zero(); }
    @Override public Vector2D closestNormal() { return Vector2D.zero(); }
    @Override public double curvature() { return 0; }
    @Override public double remainingDistance() { return 0; }
    @Override public boolean atParametricEnd() { return false; }
    @Override public void reset() {}
    @Override public boolean isBusy() { return true; }

    @Override
    public Map<String, Object> debug() {
        return Collections.singletonMap("algorithm", "untuned — run AutoTune");
    }
}
