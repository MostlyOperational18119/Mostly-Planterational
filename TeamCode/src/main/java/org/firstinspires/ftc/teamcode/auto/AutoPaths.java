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

    /**
     * How far to drive out of the corner, toward the robot's start-left, inches. Keep it at
     * least {@link #CLEAR_LEFT_IN}: the turn happens on the way there, and ending closer to the
     * right-hand wall brings a swinging corner back into it.
     */
    public static double LEAVE_LEFT_IN = 6;

    /** Heading to end on, degrees CCW from the start heading: point the camera at the HIVE. */
    public static double LEAVE_HEADING_DEG = 90;

    /**
     * The start is in a corner, a wall behind the robot and one on its right, and turning left
     * swings the back corners into both. So {@link #leave} first slides this far forward and
     * this far left with the heading held, and only then turns. Turning sweeps a corner out
     * to half the robot's diagonal — 12.7 in for the 18 in limit — against the 9 in it starts
     * from each wall, so each must be over 3.7 in; 6 leaves margin. Inches.
     */
    public static double CLEAR_FORWARD_IN = 6;
    /** See {@link #CLEAR_FORWARD_IN}. */
    public static double CLEAR_LEFT_IN = 6;

    public static final Pose START = new Pose(0, 0, 0);

    /** Where {@link #leave} stops holding the start heading: off both walls. */
    public final Pose clear = new Pose(CLEAR_FORWARD_IN, CLEAR_LEFT_IN, START.heading());

    /** Where {@link #leave} ends: out of the corner, with the HIVE's tags in the camera's view. */
    public final Pose leaveEnd = new Pose(LEAVE_FORWARD_IN, LEAVE_LEFT_IN,
            Math.toRadians(LEAVE_HEADING_DEG));

    /** Off the walls without turning, then on to {@link #leaveEnd}, turning as it goes. */
    public final Path leave = Paths.path(
            Paths.line(START, clear).constant(START),
            Paths.line(clear, leaveEnd).linear(clear, leaveEnd));
}
