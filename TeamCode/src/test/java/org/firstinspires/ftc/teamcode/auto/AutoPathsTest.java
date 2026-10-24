package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.After;
import org.junit.Test;

/** The paths take their Panels settings when built, heading converted from degrees. */
public class AutoPathsTest {

    @After
    public void restore() {
        AutoPaths.LEAVE_FORWARD_IN = 24;
        AutoPaths.LEAVE_LEFT_IN = 6;
        AutoPaths.LEAVE_HEADING_DEG = 90;
        AutoPaths.CLEAR_FORWARD_IN = 6;
        AutoPaths.CLEAR_LEFT_IN = 6;
    }

    /** The FTC size limit; the robot's real footprint is not recorded anywhere. */
    private static final double ROBOT_IN = 18;

    @Test
    public void leaveNeverSwingsACornerIntoEitherStartWall() {
        // Starting square in the corner: the back wall at x = -9, the right-hand one at y = -9.
        double wall = -ROBOT_IN / 2;
        AutoPaths paths = new AutoPaths();
        for (int i = 0; i <= 1000; i++) {
            Pose p = paths.leave.get(i / 1000.0);
            for (int sx = -1; sx <= 1; sx += 2) {
                for (int sy = -1; sy <= 1; sy += 2) {
                    double cx = sx * ROBOT_IN / 2, cy = sy * ROBOT_IN / 2;
                    double h = p.heading();
                    double x = p.x() + cx * Math.cos(h) - cy * Math.sin(h);
                    double y = p.y() + cx * Math.sin(h) + cy * Math.cos(h);
                    String at = String.format("at t=%.3f, pose (%.1f, %.1f, %.0f°)",
                            i / 1000.0, p.x(), p.y(), Math.toDegrees(h));
                    assertTrue("corner into the back wall " + at, x >= wall - 1e-6);
                    assertTrue("corner into the right-hand wall " + at, y >= wall - 1e-6);
                }
            }
        }
    }

    @Test
    public void leaveStartsAndEndsWhereTheSettingsSay() {
        AutoPaths paths = new AutoPaths();
        Pose start = paths.leave.get(0), end = paths.leave.get(1);
        assertEquals(0, start.x(), 1e-6);
        assertEquals(0, start.y(), 1e-6);
        assertEquals(AutoPaths.LEAVE_FORWARD_IN, end.x(), 1e-6);
        assertEquals(AutoPaths.LEAVE_LEFT_IN, end.y(), 1e-6);
        assertEquals(Math.toRadians(AutoPaths.LEAVE_HEADING_DEG), end.heading(), 1e-6);
    }

    @Test
    public void leaveEndsWhereTheSettingsSay() {
        AutoPaths.LEAVE_FORWARD_IN = 20;
        AutoPaths.LEAVE_LEFT_IN = 10;
        AutoPaths.LEAVE_HEADING_DEG = 30;
        AutoPaths paths = new AutoPaths();

        assertEquals(20, paths.leaveEnd.x(), 1e-9);
        assertEquals(10, paths.leaveEnd.y(), 1e-9);
        assertEquals("heading must be converted to radians",
                Math.toRadians(30), paths.leaveEnd.heading(), 1e-9);
    }

    @Test
    public void settingsChangedAfterOneBuildApplyToTheNext() {
        // Panels edits between OpModes must take effect without restarting the app.
        new AutoPaths();
        AutoPaths.LEAVE_FORWARD_IN = 30;
        assertEquals(30, new AutoPaths().leaveEnd.x(), 1e-9);
    }
}
