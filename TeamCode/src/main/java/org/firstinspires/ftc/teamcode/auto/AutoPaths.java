package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Every path the autos drive. One set serves both alliances: the moves are relative to the
 * start, and red's and blue's starts see their HIVES the same way, so the same path works
 * from either.
 *
 * <p>Relative to the start — the robot begins at the origin facing +x — because this season's
 * field coordinates are not in yet.
 *
 * <p>Built before {@code waitForStart()}, never mid-routine (architecture doc §8.1); the
 * fields are read then, so set them in Panels before pressing INIT.
 */
@Configurable
public final class AutoPaths {

    /** How far to drive out of the corner, along the robot's start-forward, inches. */
    public static double LEAVE_FORWARD_IN = 24;

    /** How far to drive out of the corner, toward the robot's start-left, inches. */
    public static double LEAVE_LEFT_IN = 0;

    /** Heading to end on, degrees CCW from the start heading: point the camera at the HIVE. */
    public static double LEAVE_HEADING_DEG = 90;

    public static final Pose START = new Pose(0, 0, 0);

    /** Where {@link #leave} ends: out of the corner, with the HIVE's tags in the camera's view. */
    public final Pose leaveEnd = new Pose(LEAVE_FORWARD_IN, LEAVE_LEFT_IN,
            Math.toRadians(LEAVE_HEADING_DEG));
    public final Path leave = Paths.line(START, leaveEnd).linear(START, leaveEnd);
}
