package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import com.pedropathing.geometry.Pose;

import org.junit.Test;

public class AimTest {

    private static final double EPS = 1e-9;

    @Test
    public void goalStraightAheadNeedsNoTurn() {
        Pose robot = new Pose(10, 10, 0);
        assertEquals(0, Aim.headingError(robot, 50, 10), EPS);
        assertEquals(40, Aim.distance(robot, 50, 10), EPS);
    }

    @Test
    public void goalToTheLeftIsAPositiveTurn() {
        Pose robot = new Pose(0, 0, 0);
        assertEquals(Math.PI / 2, Aim.headingError(robot, 0, 30), EPS);
        assertEquals(-Math.PI / 2, Aim.headingError(robot, 0, -30), EPS);
    }

    @Test
    public void errorTakesTheShortWayRound() {
        // Facing 350°, goal at bearing 10°: turn +20°, not −340°.
        Pose robot = new Pose(0, 0, Math.toRadians(350));
        double goalAngle = Math.toRadians(10);
        double e = Aim.headingError(robot, Math.cos(goalAngle) * 50, Math.sin(goalAngle) * 50);
        assertEquals(Math.toRadians(20), e, 1e-9);
    }
}
