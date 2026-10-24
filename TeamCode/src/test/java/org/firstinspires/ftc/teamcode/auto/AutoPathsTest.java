package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * The park route against the field as the manual draws it (Fig. 9-2, 9-3, 11-1), in
 * distances from the back wall and the alliance wall — the same for either alliance.
 */
public class AutoPathsTest {

    // The settings as the code ships them, so every test checks those, not stand-ins.
    private double startBack, startSide, approachBack, approachSide, parkBack, parkSide;

    @Before
    public void save() {
        startBack = AutoPaths.START_FROM_BACK_WALL_IN;
        startSide = AutoPaths.START_FROM_ALLIANCE_WALL_IN;
        approachBack = AutoPaths.APPROACH_FROM_BACK_WALL_IN;
        approachSide = AutoPaths.APPROACH_FROM_ALLIANCE_WALL_IN;
        parkBack = AutoPaths.PARK_FROM_BACK_WALL_IN;
        parkSide = AutoPaths.PARK_FROM_ALLIANCE_WALL_IN;
    }

    @After
    public void restore() {
        AutoPaths.START_FROM_BACK_WALL_IN = startBack;
        AutoPaths.START_FROM_ALLIANCE_WALL_IN = startSide;
        AutoPaths.APPROACH_FROM_BACK_WALL_IN = approachBack;
        AutoPaths.APPROACH_FROM_ALLIANCE_WALL_IN = approachSide;
        AutoPaths.PARK_FROM_BACK_WALL_IN = parkBack;
        AutoPaths.PARK_FROM_ALLIANCE_WALL_IN = parkSide;
    }

    /** The FTC size limit; the robot's real footprint is not recorded anywhere. */
    private static final double HALF_ROBOT_IN = 9;

    // The HIVE's footprint on this alliance's side (Fig. 11-1).
    private static final double HIVE_FROM_BACK_WALL_IN = 52, HIVE_TO_BACK_WALL_IN = 97;
    private static final double HIVE_FROM_ALLIANCE_WALL_IN = 47;

    // The LOADING ZONE (Fig. 9-2, 9-3).
    private static final double ZONE_FROM_BACK_WALL_IN = 96, ZONE_TO_BACK_WALL_IN = 120;
    private static final double ZONE_DEPTH_IN = 11;

    /** Start-frame pose back to distance from the back wall. */
    private static double fromBackWall(Pose p) {
        return p.x() + AutoPaths.START_FROM_BACK_WALL_IN;
    }

    /** Start-frame pose back to distance from the alliance wall. */
    private static double fromAllianceWall(Pose p) {
        return AutoPaths.START_FROM_ALLIANCE_WALL_IN - p.y();
    }

    @Test
    public void theStartFrameIsTheRobotAtTheStart() {
        Pose start = AutoPaths.fromWalls(AutoPaths.START_FROM_BACK_WALL_IN,
                AutoPaths.START_FROM_ALLIANCE_WALL_IN);
        assertEquals(0, start.x(), 1e-9);
        assertEquals(0, start.y(), 1e-9);
        // Toward the alliance wall is the robot's left (+y).
        assertTrue(AutoPaths.fromWalls(9, 30).y() > 0);
    }

    @Test
    public void theRouteToTheZoneNeverTouchesTheHive() {
        // Heading is held at the start's along the route, so the robot stays axis-aligned.
        AutoPaths paths = new AutoPaths();
        for (int i = 0; i <= 1000; i++) {
            Pose p = paths.toPark.get(i / 1000.0);
            double back = fromBackWall(p), side = fromAllianceWall(p);
            boolean besideTheHive = back + HALF_ROBOT_IN > HIVE_FROM_BACK_WALL_IN
                    && back - HALF_ROBOT_IN < HIVE_TO_BACK_WALL_IN;
            if (besideTheHive) {
                assertTrue(String.format("into the HIVE at t=%.3f: %.1f in from the alliance wall",
                        i / 1000.0, side), side + HALF_ROBOT_IN < HIVE_FROM_ALLIANCE_WALL_IN);
            }
        }
        assertTrue("approach is not short of the HIVE",
                fromBackWall(paths.approach) + HALF_ROBOT_IN < HIVE_FROM_BACK_WALL_IN);
    }

    @Test
    public void itParksPartlyInTheLoadingZoneAndOffTheWall() {
        AutoPaths paths = new AutoPaths();
        Pose end = paths.toPark.get(1);
        double back = fromBackWall(end), side = fromAllianceWall(end);
        // PARK needs the robot only partly in the zone (§10.5.4): its footprint must overlap.
        assertTrue("not level with the zone", back + HALF_ROBOT_IN > ZONE_FROM_BACK_WALL_IN
                && back - HALF_ROBOT_IN < ZONE_TO_BACK_WALL_IN);
        assertTrue("not in the zone", side - HALF_ROBOT_IN < ZONE_DEPTH_IN);
        assertTrue("into the wall", side - HALF_ROBOT_IN > 0);
        assertEquals("the path does not end at the park pose", paths.park.x(), end.x(), 1e-6);
        assertEquals(paths.park.y(), end.y(), 1e-6);
    }

    @Test
    public void settingsChangedAfterOneBuildApplyToTheNext() {
        // Panels edits between OpModes must take effect without restarting the app.
        new AutoPaths();
        AutoPaths.PARK_FROM_BACK_WALL_IN = 100;
        assertEquals(100 - AutoPaths.START_FROM_BACK_WALL_IN, new AutoPaths().park.x(), 1e-9);
    }
}
