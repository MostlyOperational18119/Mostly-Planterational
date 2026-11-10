package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Every path the autos drive. One set serves both alliances, because the field is
 * rotationally symmetric: blue's start, HIVE and LOADING ZONE are red's turned 180° about the
 * field centre (game manual Fig. 9-2), so from the robot's own point of view they are the same.
 *
 * <p><b>The start</b> is the red rectangular robot of the manual's Fig. 11-1: back against the
 * wall behind it (red: the audience wall; blue: the rear wall), centred on the third TILE from
 * its own alliance's wall, facing straight at its HIVE. From there the camera already sees the
 * HIVE's near CELL, so there is no move before shooting. The robot begins at the origin facing
 * +x; +y is its left, toward its alliance's wall.
 *
 * <p>Places on the field are given here as two distances, which read the same for either
 * alliance: from the <em>back wall</em> (the one behind the robot at the start) and from the
 * <em>alliance wall</em> (the alliance's own side wall, where its LOADING ZONE is). Measured
 * from the manual (field 144 in square, TILES 24 in) — check them on a real field.
 *
 * <p>The audience-side and scoring-side autos each share one start, approach and park; pick
 * which with {@link Side}. A side's poses are in the frame of its own start ({@link #START}).
 *
 * <p><b>The scoring side</b> starts where the HIVE's near CELL cannot be seen, so it drives
 * first, by {@link #toHive}, to {@link #hive}, the place it shoots from. From there it parks
 * like the audience side, by {@link #toPark}.
 *
 * <p><b>One move per route.</b> Each route is a corner between two straight legs — start,
 * waypoint ({@link #approachHive}, {@link #approachPark}), end — built as one path that rounds
 * the corner with a Bezier curve ({@link #rounded}), so the robot keeps its speed through it
 * instead of stopping at the waypoint. The waypoint itself is not driven to: it only fixes
 * the two legs. The curve stays inside the corner (a Bezier lies within its control points),
 * so it never strays outside the two legs it replaces. How far before and after the waypoint
 * the curve begins and ends is a {@code *_CORNER_*} setting: longer saves more time; under the
 * HIVE it is kept short, so the robot crosses on the straight line the team confirmed.
 *
 * <p><b>Heading.</b> Every path holds the start's heading wherever the robot is near the HIVE,
 * because the robot's corners swing out when it turns. Aiming turns it, so after shooting the
 * routines square it up in place before driving on ({@code ShootingRoutines.shootIfUpThenPark}).
 *
 * <p>Built before {@code waitForStart()}, never mid-routine (architecture doc §8.1); the
 * fields are read then, so set them in Panels before pressing INIT.
 */
@Configurable
public final class AutoPaths {

    /** Which set of poses an auto uses. */
    public enum Side { AUDIENCE, SCORING }

    // START FOR AUDIENCE SIDE
    /** The robot's centre at the start: half its length off the back wall, inches. */
    public static double AUDIENCE_START_FROM_BACK_WALL_IN = 9;
    /**
     * The robot's centre at the start: the middle of the third TILE from the alliance wall —
     * in line with the HIVE, as in Fig. 11-1 — inches.
     */
    public static double AUDIENCE_START_FROM_ALLIANCE_WALL_IN = 60;

    /**
     * Where the robot heads after shooting, to get round the HIVE: short of its near end and
     * outside it on the alliance-wall side. The HIVE covers 52–97 in from the back wall and
     * starts 47 in from the alliance wall (Fig. 11-1), so from here an 18 in robot clears it by
     * 7 in or more all the way up to the LOADING ZONE. Inches.
     */
    public static double AUDIENCE_APPROACH_PARK_BACK_WALL_IN = 36;
    /** See {@link #AUDIENCE_APPROACH_PARK_BACK_WALL_IN}. */
    public static double AUDIENCE_APPROACH_PARK_ALLIANCE_WALL_IN = 30;

    /**
     * Where the robot parks: in the alliance's LOADING ZONE, which runs 96–120 in from the back
     * wall and 11 in out from the alliance wall (Fig. 9-2, 9-3). PARK needs the robot only
     * partly in it (§10.5.4); centred 12 in off the wall, an 18 in robot spans 3–21 in, 8 in
     * of it inside the zone and clear of the wall. Inches.
     */
    public static double AUDIENCE_PARK_FROM_BACK_WALL_IN = 92;
    /** See {@link #AUDIENCE_PARK_FROM_BACK_WALL_IN}. */
    public static double AUDIENCE_PARK_FROM_ALLIANCE_WALL_IN = 15;

    /**
     * How much of each leg either side of {@link #AUDIENCE_APPROACH_PARK_BACK_WALL_IN}'s
     * waypoint the corner curve takes on the way to park, inches. Kept clear of the HIVE by
     * {@code AutoPathsTest}, which checks the robot's footprint along the whole curve.
     */
    public static double AUDIENCE_CORNER_IN = 25;

    // SCORING SIDE
    // Same meaning as above: inches from the back wall and from the alliance wall. Not yet
    // driven on a field.

    //Start pose
    public static double SCORING_START_FROM_BACK_WALL_IN = 120;
    public static double SCORING_START_FROM_ALLIANCE_WALL_IN = 9;

    //Start - Hive prep pose: straight out from the start position, preparing to move underneath the hive to the scoring location
    public static double SCORING_APPROACH_HIVE_BACK_WALL_IN = 120;
    public static double SCORING_APPROACH_HIVE_ALLIANCE_WALL_IN = 68;

    //Hive pose: launching location
    public static double SCORING_HIVE_BACK_WALL_IN = 12;
    public static double SCORING_HIVE_ALLIANCE_WALL_IN = 64;

    //Hive - Park prep pose: same as above
    public static double SCORING_APPROACH_PARK_BACK_WALL_IN = 120;
    public static double SCORING_APPROACH_PARK_ALLIANCE_WALL_IN = 68;

    /**
     * How much of the leg that crosses under the HIVE the corner curve may take, inches back
     * from the waypoint. The HIVE ends 97 in from the back wall and the robot reaches 9 in
     * past its centre, so with the waypoint at 120 the robot is clear of the HIVE until 14 in
     * along that leg; 11 leaves 3 in. Longer and the robot would still be curving, not on the
     * confirmed straight line, as it reached the HIVE.
     */
    public static double SCORING_CORNER_HIVE_LEG_IN = 11;

    /** How much of the open leg, along the alliance wall's side, the corner curve may take. */
    public static double SCORING_CORNER_OPEN_LEG_IN = 40;

    //Park pose: same as above
    public static double SCORING_PARK_FROM_BACK_WALL_IN = 120;
    public static double SCORING_PARK_FROM_ALLIANCE_WALL_IN = 10;

    // PRELOAD + FIELD (audience side)
    // Inches from the back wall and, in the *_AUDIENCE_WALL_* fields, from the alliance wall, as
    // in fromWalls(). Not yet driven on a field.

    //Start pose
    public static double PRELOAD_AND_FIELD_BACK_WALL_START = 9;
    public static double PRELOAD_AND_FIELD_AUDIENCE_WALL_START = 60;

    //End pose: where the robot has finished collecting.
    public static double PRELOAD_AND_FIELD_AUDIENCE_WALL_END = 13;
    public static double PRELOAD_AND_FIELD_BACK_WALL_END = 7.5;
    // Really going to the edge of the wall to intake (may need to intake at an angle if this doesn't work);



    /** Heading while collecting, relative to the start; + is CCW, toward the alliance wall. */
    public static double PRELOAD_AND_FIELD_COLLECT_HEADING_DEG = 90;

    /**
     * Speed limit on the drive to the end pose, in/s, so the intake has time to take the pollen
     * instead of the robot ramming them. Only that path; top speed is about 80. <b>Placeholder.</b>
     */
    public static double PRELOAD_AND_FIELD_COLLECT_SPEED_IN_PER_S = 20;

    public static final Pose START = new Pose(0, 0, 0);

    /** Audience side only, else null: at the start, turned to collect. */
    public final Pose fieldTurned;

    /** Audience side only, else null: where collecting ends, heading as {@link #fieldTurned}. */
    public final Pose fieldEnd;

    /** Audience side only, else null: from {@link #fieldTurned} to {@link #fieldEnd}. */
    public final Path toFieldEnd;

    /** Audience side only, else null: from {@link #fieldEnd} back to {@link #fieldTurned}. */
    public final Path toFieldStart;

    /**
     * Scoring side only, else null: the corner of the route to {@link #hive} — out from the
     * start to here, then straight under the HIVE. Rounded off, not driven to.
     */
    public final Pose approachHive;

    /** Scoring side only, else null: where the robot shoots from, facing as at the start. */
    public final Pose hive;

    /**
     * Scoring side only, else null: from the start to {@link #hive}, round
     * {@link #approachHive}. Crosses the HIVE's footprint on purpose, on the straight line
     * from {@link #approachHive}: the team confirmed the robot can pass there.
     */
    public final Path toHive;

    /**
     * Where the robot shoots from, facing as at the start: {@link #START} on the audience side,
     * {@link #hive} on the scoring side. After shooting, the robot squares up and returns here
     * before {@link #toPark}, because aiming turns it and ranging moves it.
     */
    public final Pose shootFrom;

    /** The corner of the route to park: round the HIVE, or out from under it. Not driven to. */
    public final Pose approachPark;

    /** In the LOADING ZONE, facing as at the start. */
    public final Pose park;

    /**
     * From {@link #shootFrom} into the LOADING ZONE, round {@link #approachPark}, heading held
     * at the start's. On the scoring side this retraces {@link #toHive}'s line under the HIVE,
     * the route the team confirmed the robot fits through square — but not turned, when its
     * corners swing wider (an 18 in square spans 25 in at 45°). So this is a path built at
     * init with a constant heading, not a {@code Drive.hold}, which would turn the robot back
     * from its aim while it crossed.
     */
    public final Path toPark;

    public AutoPaths(Side side) {
        if (side == Side.AUDIENCE) {
            approachPark = fromWalls(AUDIENCE_APPROACH_PARK_BACK_WALL_IN,
                    AUDIENCE_APPROACH_PARK_ALLIANCE_WALL_IN,
                    AUDIENCE_START_FROM_BACK_WALL_IN, AUDIENCE_START_FROM_ALLIANCE_WALL_IN);
            park = fromWalls(AUDIENCE_PARK_FROM_BACK_WALL_IN, AUDIENCE_PARK_FROM_ALLIANCE_WALL_IN,
                    AUDIENCE_START_FROM_BACK_WALL_IN, AUDIENCE_START_FROM_ALLIANCE_WALL_IN);
            approachHive = null;
            hive = null;
            toHive = null;
            fieldTurned = START.withHeading(Math.toRadians(PRELOAD_AND_FIELD_COLLECT_HEADING_DEG));
            fieldEnd = fromWalls(PRELOAD_AND_FIELD_BACK_WALL_END,
                    PRELOAD_AND_FIELD_AUDIENCE_WALL_END,
                    PRELOAD_AND_FIELD_BACK_WALL_START, PRELOAD_AND_FIELD_AUDIENCE_WALL_START)
                    .withHeading(fieldTurned.heading());
            toFieldEnd = Paths.line(fieldTurned, fieldEnd).constant(fieldTurned)
                    // In in/s; maxPathSpeed, by contrast, is a fraction of top speed.
                    .with(Constants.foresightConfig.maxVelocityConstraint
                            .at(PRELOAD_AND_FIELD_COLLECT_SPEED_IN_PER_S));
            toFieldStart = Paths.line(fieldEnd, fieldTurned).constant(fieldTurned);
        } else {
            fieldTurned = null;
            fieldEnd = null;
            toFieldEnd = null;
            toFieldStart = null;
            approachPark = fromWalls(SCORING_APPROACH_PARK_BACK_WALL_IN,
                    SCORING_APPROACH_PARK_ALLIANCE_WALL_IN,
                    SCORING_START_FROM_BACK_WALL_IN, SCORING_START_FROM_ALLIANCE_WALL_IN);
            park = fromWalls(SCORING_PARK_FROM_BACK_WALL_IN, SCORING_PARK_FROM_ALLIANCE_WALL_IN,
                    SCORING_START_FROM_BACK_WALL_IN, SCORING_START_FROM_ALLIANCE_WALL_IN);
            approachHive = fromWalls(SCORING_APPROACH_HIVE_BACK_WALL_IN,
                    SCORING_APPROACH_HIVE_ALLIANCE_WALL_IN,
                    SCORING_START_FROM_BACK_WALL_IN, SCORING_START_FROM_ALLIANCE_WALL_IN);
            hive = fromWalls(SCORING_HIVE_BACK_WALL_IN, SCORING_HIVE_ALLIANCE_WALL_IN,
                    SCORING_START_FROM_BACK_WALL_IN, SCORING_START_FROM_ALLIANCE_WALL_IN);
            toHive = rounded(START, approachHive, hive,
                    SCORING_CORNER_OPEN_LEG_IN, SCORING_CORNER_HIVE_LEG_IN);
        }
        shootFrom = side == Side.AUDIENCE ? START : hive;
        toPark = side == Side.AUDIENCE
                ? rounded(shootFrom, approachPark, park, AUDIENCE_CORNER_IN, AUDIENCE_CORNER_IN)
                : rounded(shootFrom, approachPark, park,
                        SCORING_CORNER_HIVE_LEG_IN, SCORING_CORNER_OPEN_LEG_IN);
    }

    /**
     * One path from {@code from} to {@code to}, by the legs through {@code corner}, with the
     * corner rounded: straight along the first leg until {@code inIn} short of the corner, a
     * quadratic Bezier with the corner as its control point to {@code outIn} along the second
     * leg, then straight on. The curve is tangent to both legs, so the follower carries its
     * speed through, and lies inside the triangle of its three points, so it never leaves the
     * corner's inside. Each length is clamped to its leg; heading held at the start's.
     */
    public static Path rounded(Pose from, Pose corner, Pose to, double inIn, double outIn) {
        Pose curveFrom = along(corner, from, inIn);
        Pose curveTo = along(corner, to, outIn);
        java.util.List<Path> pieces = new java.util.ArrayList<>();
        if (from.distance(curveFrom) > 1e-6) pieces.add(Paths.line(from, curveFrom));
        if (curveFrom.distance(corner) > 1e-6 && curveTo.distance(corner) > 1e-6) {
            pieces.add(Paths.curve(curveFrom, corner, curveTo));
        } else if (curveFrom.distance(curveTo) > 1e-6) {
            pieces.add(Paths.line(curveFrom, curveTo));    // a corner rounded by nothing
        }
        if (curveTo.distance(to) > 1e-6) pieces.add(Paths.line(curveTo, to));
        Path path = pieces.size() == 1 ? pieces.get(0)
                : Paths.path(pieces.toArray(new Path[0]));
        return path.constant(START);
    }

    /** The point {@code lengthIn} from {@code a} toward {@code b}, at most {@code b}. */
    private static Pose along(Pose a, Pose b, double lengthIn) {
        double d = a.distance(b);
        double f = d < 1e-9 ? 0 : Math.max(0, Math.min(1, lengthIn / d));
        return new Pose(a.x() + (b.x() - a.x()) * f, a.y() + (b.y() - a.y()) * f, START.heading());
    }

    /** A field position, as distances from the back and alliance walls, in a start's frame. */
    static Pose fromWalls(double fromBackWallIn, double fromAllianceWallIn,
                          double startFromBackWallIn, double startFromAllianceWallIn) {
        return new Pose(fromBackWallIn - startFromBackWallIn,
                startFromAllianceWallIn - fromAllianceWallIn, START.heading());
    }
}
