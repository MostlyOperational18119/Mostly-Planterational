package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Aim geometry in the field frame: where a point is relative to a robot pose. For aiming from
 * odometry; without a field position, {@code Vision}'s robot-relative
 * {@code CellSighting} already gives bearing and distance directly.
 */
public final class Aim {

    private Aim() {}

    /** Field-frame direction from the robot to {@code (goalX, goalY)}, radians. */
    public static double bearing(Pose robot, double goalX, double goalY) {
        return Math.atan2(goalY - robot.y(), goalX - robot.x());
    }

    /**
     * How far the robot must turn to face {@code (goalX, goalY)}, radians in (−π, π].
     * Positive is counter-clockwise, Pedro's turn direction.
     */
    public static double headingError(Pose robot, double goalX, double goalY) {
        double e = bearing(robot, goalX, goalY) - robot.heading();
        return Math.atan2(Math.sin(e), Math.cos(e));
    }

    /** Straight-line distance to {@code (goalX, goalY)}, inches. */
    public static double distance(Pose robot, double goalX, double goalY) {
        return Math.hypot(goalX - robot.x(), goalY - robot.y());
    }
}
