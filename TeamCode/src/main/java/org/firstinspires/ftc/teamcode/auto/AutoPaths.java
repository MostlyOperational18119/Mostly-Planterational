package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

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
    public static double AUDIENCE_PARK_FROM_ALLIANCE_WALL_IN = 10;

    // START FOR SCORING SIDE
    // PLACEHOLDERS: copies of the audience side's values until the real poses are measured.
    // Same meaning as above: inches from the back wall and from the alliance wall.
    public static double SCORING_START_FROM_BACK_WALL_IN = 120;
    public static double SCORING_START_FROM_ALLIANCE_WALL_IN = 10;
    public static double SCORING_APPROACH_PARK_BACK_WALL_IN = 36;
    public static double SCORING_APPROACH_PARK_ALLIANCE_WALL_IN = 30;
    public static double SCORING_PARK_FROM_BACK_WALL_IN = 120;
    public static double SCORING_PARK_FROM_ALLIANCE_WALL_IN = 10;

    public static final Pose START = new Pose(0, 0, 0);

    /**
     * Beside the HIVE, facing as at the start. Reached with {@code Drive.hold} from wherever
     * shooting left the robot, so it needs no path built mid-routine.
     */
    public final Pose approachPark;

    /** In the LOADING ZONE, facing as at the start. */
    public final Pose park;

    /** From {@link #approachPark} up the alliance-wall side of the HIVE into the LOADING ZONE. */
    public final Path toPark;

    public AutoPaths(Side side) {
        if (side == Side.AUDIENCE) {
            approachPark = fromWalls(AUDIENCE_APPROACH_PARK_BACK_WALL_IN,
                    AUDIENCE_APPROACH_PARK_ALLIANCE_WALL_IN,
                    AUDIENCE_START_FROM_BACK_WALL_IN, AUDIENCE_START_FROM_ALLIANCE_WALL_IN);
            park = fromWalls(AUDIENCE_PARK_FROM_BACK_WALL_IN, AUDIENCE_PARK_FROM_ALLIANCE_WALL_IN,
                    AUDIENCE_START_FROM_BACK_WALL_IN, AUDIENCE_START_FROM_ALLIANCE_WALL_IN);
        } else {
            approachPark = fromWalls(SCORING_APPROACH_PARK_BACK_WALL_IN,
                    SCORING_APPROACH_PARK_ALLIANCE_WALL_IN,
                    SCORING_START_FROM_BACK_WALL_IN, SCORING_START_FROM_ALLIANCE_WALL_IN);
            park = fromWalls(SCORING_PARK_FROM_BACK_WALL_IN, SCORING_PARK_FROM_ALLIANCE_WALL_IN,
                    SCORING_START_FROM_BACK_WALL_IN, SCORING_START_FROM_ALLIANCE_WALL_IN);
        }
        toPark = Paths.line(approachPark, park).constant(START);
    }

    /** A field position, as distances from the back and alliance walls, in a start's frame. */
    static Pose fromWalls(double fromBackWallIn, double fromAllianceWallIn,
                          double startFromBackWallIn, double startFromAllianceWallIn) {
        return new Pose(fromBackWallIn - startFromBackWallIn,
                startFromAllianceWallIn - fromAllianceWallIn, START.heading());
    }
}
