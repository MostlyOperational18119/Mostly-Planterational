package org.firstinspires.ftc.teamcode.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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
        startBack = AutoPaths.AUDIENCE_START_FROM_BACK_WALL_IN;
        startSide = AutoPaths.AUDIENCE_START_FROM_ALLIANCE_WALL_IN;
        approachBack = AutoPaths.AUDIENCE_APPROACH_PARK_BACK_WALL_IN;
        approachSide = AutoPaths.AUDIENCE_APPROACH_PARK_ALLIANCE_WALL_IN;
        parkBack = AutoPaths.AUDIENCE_PARK_FROM_BACK_WALL_IN;
        parkSide = AutoPaths.AUDIENCE_PARK_FROM_ALLIANCE_WALL_IN;
    }

    @After
    public void restore() {
        AutoPaths.AUDIENCE_START_FROM_BACK_WALL_IN = startBack;
        AutoPaths.AUDIENCE_START_FROM_ALLIANCE_WALL_IN = startSide;
        AutoPaths.AUDIENCE_APPROACH_PARK_BACK_WALL_IN = approachBack;
        AutoPaths.AUDIENCE_APPROACH_PARK_ALLIANCE_WALL_IN = approachSide;
        AutoPaths.AUDIENCE_PARK_FROM_BACK_WALL_IN = parkBack;
        AutoPaths.AUDIENCE_PARK_FROM_ALLIANCE_WALL_IN = parkSide;
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
        return p.x() + AutoPaths.AUDIENCE_START_FROM_BACK_WALL_IN;
    }

    /** Start-frame pose back to distance from the alliance wall. */
    private static double fromAllianceWall(Pose p) {
        return AutoPaths.AUDIENCE_START_FROM_ALLIANCE_WALL_IN - p.y();
    }

    @Test
    public void theStartFrameIsTheRobotAtTheStart() {
        Pose start = AutoPaths.fromWalls(9, 60, 9, 60);
        assertEquals(0, start.x(), 1e-9);
        assertEquals(0, start.y(), 1e-9);
        // Toward the alliance wall is the robot's left (+y).
        assertTrue(AutoPaths.fromWalls(9, 30, 9, 60).y() > 0);
    }

    /**
     * The far edge of the HIVE's footprint from the alliance wall. Not in the manual's drawings
     * as used here; the field centre line is the most it can be, so this errs toward a collision.
     */
    private static final double HIVE_TO_ALLIANCE_WALL_IN = 72;
    private static final double FIELD_IN = 144;

    /**
     * The robot's footprint at {@code t} along {@code path}, turned to the path's heading
     * there, as its bounding box in wall distances: {back min, back max, side min, side max}.
     * The box of a turned square is wider than the square, so this errs toward a collision.
     */
    private static double[] footprint(com.pedropathing.paths.Path path, double t,
                                      double startBack, double startSide) {
        return footprint(path.get(t), path.heading(t), startBack, startSide);
    }

    /** {@link #footprint(com.pedropathing.paths.Path, double, double, double)} at a pose. */
    private static double[] footprint(Pose p, double h, double startBack, double startSide) {
        // Half-extent of a turned square along either axis.
        double half = HALF_ROBOT_IN * (Math.abs(Math.cos(h)) + Math.abs(Math.sin(h)));
        double back = p.x() + startBack, side = startSide - p.y();
        return new double[] {back - half, back + half, side - half, side + half};
    }

    private static boolean overlapsTheHive(double[] box) {
        return box[1] > HIVE_FROM_BACK_WALL_IN && box[0] < HIVE_TO_BACK_WALL_IN
                && box[3] > HIVE_FROM_ALLIANCE_WALL_IN && box[2] < HIVE_TO_ALLIANCE_WALL_IN;
    }

    /** True if {@code heading} is the start's, i.e. the robot is square to the field. */
    private static boolean square(double heading) {
        return Math.abs(Math.atan2(Math.sin(heading - AutoPaths.START.heading()),
                Math.cos(heading - AutoPaths.START.heading()))) < 1e-6;
    }

    /** Distance of {@code p} from the line from {@code approachHive} to {@code hive}. */
    private static double offTheConfirmedLine(Pose p, AutoPaths paths) {
        double ax = paths.approachHive.x(), ay = paths.approachHive.y();
        double dx = paths.hive.x() - ax, dy = paths.hive.y() - ay;
        return Math.abs((p.x() - ax) * dy - (p.y() - ay) * dx) / Math.hypot(dx, dy);
    }

    @Test
    public void eachRouteIsOneSmoothPathRoundItsCorner() {
        // No stop at the waypoint: the route is a single path, its direction continuous, and it
        // cuts the corner rather than driving to it.
        for (AutoPaths.Side s : AutoPaths.Side.values()) {
            AutoPaths paths = new AutoPaths(s);
            com.pedropathing.paths.Path[] routes = s == AutoPaths.Side.AUDIENCE
                    ? new com.pedropathing.paths.Path[] {paths.toPark}
                    : new com.pedropathing.paths.Path[] {paths.toHive, paths.toPark};
            Pose[] corners = s == AutoPaths.Side.AUDIENCE
                    ? new Pose[] {paths.approachPark}
                    : new Pose[] {paths.approachHive, paths.approachPark};
            for (int k = 0; k < routes.length; k++) {
                double closest = Double.MAX_VALUE, worstBend = 0;
                Pose prev = routes[k].get(0), prevStep = null;
                for (int i = 1; i <= 2000; i++) {
                    Pose p = routes[k].get(i / 2000.0);
                    closest = Math.min(closest, p.distance(corners[k]));
                    Pose step = new Pose(p.x() - prev.x(), p.y() - prev.y(), 0);
                    if (prevStep != null && Math.hypot(step.x(), step.y()) > 1e-9
                            && Math.hypot(prevStep.x(), prevStep.y()) > 1e-9) {
                        double bend = Math.abs(Math.atan2(
                                prevStep.x() * step.y() - prevStep.y() * step.x(),
                                prevStep.x() * step.x() + prevStep.y() * step.y()));
                        worstBend = Math.max(worstBend, bend);
                    }
                    prev = p;
                    prevStep = step;
                }
                String which = s + " route " + k;
                assertTrue(which + " drives to its corner instead of rounding it", closest > 2);
                assertTrue(String.format("%s has a sharp turn of %.1f° in one step", which,
                        Math.toDegrees(worstBend)), worstBend < Math.toRadians(5));
            }
        }
    }

    @Test
    public void theAudienceSideNeverTouchesTheHiveAtAnyHeading() {
        AutoPaths paths = new AutoPaths(AutoPaths.Side.AUDIENCE);
        com.pedropathing.paths.Path[] legs = {paths.toPark, paths.toFieldEnd,
                paths.toFieldStart};
        String[] names = {"toPark", "toFieldEnd", "toFieldStart"};
        for (int k = 0; k < legs.length; k++) {
            for (int i = 0; i <= 1000; i++) {
                double[] box = footprint(legs[k], i / 1000.0,
                        AutoPaths.AUDIENCE_START_FROM_BACK_WALL_IN,
                        AutoPaths.AUDIENCE_START_FROM_ALLIANCE_WALL_IN);
                assertFalse(String.format("%s into the HIVE at t=%.3f", names[k], i / 1000.0),
                        overlapsTheHive(box));
            }
        }
    }

    @Test
    public void theScoringSideIsSquareWheneverItIsUnderTheHive() {
        // The team confirmed the robot fits under the HIVE square; turned, its corners swing
        // out and hit it. Every leg near the HIVE must hold the start's heading.
        AutoPaths paths = new AutoPaths(AutoPaths.Side.SCORING);
        com.pedropathing.paths.Path[] legs = {paths.toHive, paths.toPark};
        String[] names = {"toHive", "toPark"};
        int under = 0;
        for (int k = 0; k < legs.length; k++) {
            for (int i = 0; i <= 1000; i++) {
                double t = i / 1000.0;
                double[] box = footprint(legs[k], t, AutoPaths.SCORING_START_FROM_BACK_WALL_IN,
                        AutoPaths.SCORING_START_FROM_ALLIANCE_WALL_IN);
                if (overlapsTheHive(box)) {
                    under++;
                    assertTrue(String.format("%s turned under the HIVE at t=%.3f", names[k], t),
                            square(legs[k].heading(t)));
                    // And on the line the team drove, not still rounding a corner: the curve
                    // must be over before the robot reaches the HIVE.
                    double off = offTheConfirmedLine(legs[k].get(t), paths);
                    assertTrue(String.format("%s %.2f in off the confirmed line under the HIVE"
                            + " at t=%.3f", names[k], off, t), off < 1e-3);
                }
            }
        }
        assertTrue("never went under the HIVE, so this proves nothing", under > 0);
        // The way back out is the way in, reversed: the one route known to clear the HIVE.
        assertEquals(paths.approachHive.x(), paths.approachPark.x(), 1e-9);
        assertEquals(paths.approachHive.y(), paths.approachPark.y(), 1e-9);
        // And the robot squares up where it shoots, clear of the HIVE, before it sets off.
        assertEquals(paths.hive.x(), paths.shootFrom.x(), 1e-9);
        assertEquals(paths.hive.y(), paths.shootFrom.y(), 1e-9);
        assertTrue(square(paths.shootFrom.heading()));
        assertFalse("turning at the shooting spot would swing into the HIVE", overlapsTheHive(
                footprint(paths.shootFrom, Math.PI / 4, AutoPaths.SCORING_START_FROM_BACK_WALL_IN,
                        AutoPaths.SCORING_START_FROM_ALLIANCE_WALL_IN)));
    }

    @Test
    public void neitherSideDrivesIntoAWall() {
        // The collecting legs are left out: they drive into the back wall on purpose.
        for (AutoPaths.Side s : AutoPaths.Side.values()) {
            AutoPaths paths = new AutoPaths(s);
            boolean audience = s == AutoPaths.Side.AUDIENCE;
            double startBack = audience ? AutoPaths.AUDIENCE_START_FROM_BACK_WALL_IN
                    : AutoPaths.SCORING_START_FROM_BACK_WALL_IN;
            double startSide = audience ? AutoPaths.AUDIENCE_START_FROM_ALLIANCE_WALL_IN
                    : AutoPaths.SCORING_START_FROM_ALLIANCE_WALL_IN;
            com.pedropathing.paths.Path[] legs = audience
                    ? new com.pedropathing.paths.Path[] {paths.toPark}
                    : new com.pedropathing.paths.Path[] {paths.toHive, paths.toPark};
            for (com.pedropathing.paths.Path leg : legs) {
                for (int i = 0; i <= 1000; i++) {
                    double[] box = footprint(leg, i / 1000.0, startBack, startSide);
                    // Starting against a wall is touching it, not driving into it.
                    String where = String.format("%s side at t=%.3f", s, i / 1000.0);
                    assertTrue("into the back wall, " + where, box[0] > -1e-6);
                    assertTrue("into the far wall, " + where, box[1] < FIELD_IN + 1e-6);
                    assertTrue("into the alliance wall, " + where, box[2] > -1e-6);
                    assertTrue("into the other side wall, " + where, box[3] < FIELD_IN + 1e-6);
                }
            }
        }
    }

    @Test
    public void itParksPartlyInTheLoadingZoneAndOffTheWall() {
        AutoPaths paths = new AutoPaths(AutoPaths.Side.AUDIENCE);
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
    public void theScoringSideDrivesToTheHiveThroughItsApproach() {
        AutoPaths paths = new AutoPaths(AutoPaths.Side.SCORING);
        assertEquals(0, paths.toHive.get(0).x(), 1e-6);
        assertEquals(0, paths.toHive.get(0).y(), 1e-6);
        assertEquals(paths.hive.x(), paths.toHive.get(1).x(), 1e-6);
        assertEquals(paths.hive.y(), paths.toHive.get(1).y(), 1e-6);
        // As distances from the walls: x grows away from the back wall, y toward the alliance wall.
        assertEquals(AutoPaths.SCORING_HIVE_BACK_WALL_IN - AutoPaths.SCORING_START_FROM_BACK_WALL_IN,
                paths.hive.x(), 1e-9);
        assertEquals(AutoPaths.SCORING_START_FROM_ALLIANCE_WALL_IN - AutoPaths.SCORING_HIVE_ALLIANCE_WALL_IN,
                paths.hive.y(), 1e-9);
    }

    @Test
    public void theFieldPathsRunBetweenTheTurnedStartAndTheEndPose() {
        double endBack = AutoPaths.PRELOAD_AND_FIELD_BACK_WALL_END;
        double endSide = AutoPaths.PRELOAD_AND_FIELD_AUDIENCE_WALL_END;
        try {
            AutoPaths.PRELOAD_AND_FIELD_BACK_WALL_END = 21;
            AutoPaths.PRELOAD_AND_FIELD_AUDIENCE_WALL_END = 20;
            AutoPaths paths = new AutoPaths(AutoPaths.Side.AUDIENCE);

            assertEquals(Math.PI / 2, paths.fieldTurned.heading(), 1e-9);
            assertEquals(12, paths.fieldEnd.x(), 1e-9);
            assertEquals(40, paths.fieldEnd.y(), 1e-9);
            assertEquals(Math.PI / 2, paths.fieldEnd.heading(), 1e-9);
            assertEquals(0, paths.toFieldEnd.get(0).x(), 1e-6);
            assertEquals(paths.fieldEnd.y(), paths.toFieldEnd.get(1).y(), 1e-6);
            assertEquals(paths.fieldEnd.y(), paths.toFieldStart.get(0).y(), 1e-6);
            assertEquals(0, paths.toFieldStart.get(1).y(), 1e-6);
        } finally {
            AutoPaths.PRELOAD_AND_FIELD_BACK_WALL_END = endBack;
            AutoPaths.PRELOAD_AND_FIELD_AUDIENCE_WALL_END = endSide;
        }
    }

    @Test
    public void onlyTheDriveToTheEndPoseIsSpeedLimited() {
        AutoPaths paths = new AutoPaths(AutoPaths.Side.AUDIENCE);
        assertEquals(1, paths.toFieldEnd.modifiers.size());
        assertTrue(paths.toFieldStart.modifiers.isEmpty());
        assertTrue(paths.toPark.modifiers.isEmpty());

        double before = Constants.foresightConfig.maxVelocityConstraint.get();
        paths.toFieldEnd.modifiers.get(0).apply();
        assertEquals(AutoPaths.PRELOAD_AND_FIELD_COLLECT_SPEED_IN_PER_S,
                Constants.foresightConfig.maxVelocityConstraint.get(), 0.0);
        paths.toFieldEnd.modifiers.get(0).revert();
        assertEquals(before, Constants.foresightConfig.maxVelocityConstraint.get(), 0.0);
    }

    @Test
    public void settingsChangedAfterOneBuildApplyToTheNext() {
        // Panels edits between OpModes must take effect without restarting the app.
        new AutoPaths(AutoPaths.Side.AUDIENCE);
        AutoPaths.AUDIENCE_PARK_FROM_BACK_WALL_IN = 100;
        assertEquals(100 - AutoPaths.AUDIENCE_START_FROM_BACK_WALL_IN, new AutoPaths(AutoPaths.Side.AUDIENCE).park.x(), 1e-9);
    }

    @Test
    public void eachSideUsesOnlyItsOwnSettings() {
        double[] scoring = {AutoPaths.SCORING_START_FROM_BACK_WALL_IN,
                AutoPaths.SCORING_START_FROM_ALLIANCE_WALL_IN,
                AutoPaths.SCORING_APPROACH_PARK_BACK_WALL_IN,
                AutoPaths.SCORING_APPROACH_PARK_ALLIANCE_WALL_IN,
                AutoPaths.SCORING_PARK_FROM_BACK_WALL_IN,
                AutoPaths.SCORING_PARK_FROM_ALLIANCE_WALL_IN};
        Pose audiencePark = new AutoPaths(AutoPaths.Side.AUDIENCE).park;
        try {
            AutoPaths.SCORING_START_FROM_BACK_WALL_IN = 10;
            AutoPaths.SCORING_START_FROM_ALLIANCE_WALL_IN = 40;
            AutoPaths.SCORING_APPROACH_PARK_BACK_WALL_IN = 50;
            AutoPaths.SCORING_APPROACH_PARK_ALLIANCE_WALL_IN = 25;
            AutoPaths.SCORING_PARK_FROM_BACK_WALL_IN = 100;
            AutoPaths.SCORING_PARK_FROM_ALLIANCE_WALL_IN = 15;

            AutoPaths paths = new AutoPaths(AutoPaths.Side.SCORING);
            assertEquals(40, paths.approachPark.x(), 1e-9);
            assertEquals(15, paths.approachPark.y(), 1e-9);
            assertEquals(90, paths.park.x(), 1e-9);
            assertEquals(25, paths.park.y(), 1e-9);
            assertEquals(90, paths.toPark.get(1).x(), 1e-6);

            Pose unchanged = new AutoPaths(AutoPaths.Side.AUDIENCE).park;
            assertEquals(audiencePark.x(), unchanged.x(), 1e-9);
            assertEquals(audiencePark.y(), unchanged.y(), 1e-9);
        } finally {
            AutoPaths.SCORING_START_FROM_BACK_WALL_IN = scoring[0];
            AutoPaths.SCORING_START_FROM_ALLIANCE_WALL_IN = scoring[1];
            AutoPaths.SCORING_APPROACH_PARK_BACK_WALL_IN = scoring[2];
            AutoPaths.SCORING_APPROACH_PARK_ALLIANCE_WALL_IN = scoring[3];
            AutoPaths.SCORING_PARK_FROM_BACK_WALL_IN = scoring[4];
            AutoPaths.SCORING_PARK_FROM_ALLIANCE_WALL_IN = scoring[5];
        }
    }
}
