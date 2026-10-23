package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Test;

/** The paths take their Panels settings when built, heading converted from degrees. */
public class AutoPathsTest {

    @After
    public void restore() {
        AutoPaths.LEAVE_FORWARD_IN = 24;
        AutoPaths.LEAVE_LEFT_IN = 0;
        AutoPaths.LEAVE_HEADING_DEG = 90;
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
