package org.firstinspires.ftc.teamcode.field;

/**
 * The four CELLS of the HIVE Structure and the AprilTag cluster under each — game manual
 * (TU04) §9.6 and §9.9.
 *
 * <p>Each alliance's HIVE is a rigid seesaw with a CELL at each end; it rests tipped 30° with
 * one CELL facing up and flips when enough is LAUNCHED into that CELL. The up CELL is the
 * target, and it changes during the match, so nothing here pins a goal to a field position.
 *
 * <p>Each CELL carries one cluster of four 36h11 tags, 3.25 in square, in a row on its bottom
 * face, facing the TILES with the bottom edge toward the field centre. Because both CELLS of a
 * HIVE hang from the same rigid beam, both clusters face downward in either state — the up
 * CELL's simply sits higher.
 */
public enum Cell {

    RED_FAR(Alliance.RED, Side.FAR, 30),
    RED_AUDIENCE(Alliance.RED, Side.AUDIENCE, 34),
    BLUE_AUDIENCE(Alliance.BLUE, Side.AUDIENCE, 38),
    BLUE_FAR(Alliance.BLUE, Side.FAR, 42);

    public enum Side { AUDIENCE, FAR }

    /** Tag centres along the cluster, in tag-ID order, from the cluster centre (Fig. 9-15). */
    private static final double[] TAG_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};

    public static final double TAG_SIZE_IN = 3.25;

    public final Alliance alliance;
    public final Side side;
    public final int firstTagId;

    Cell(Alliance alliance, Side side, int firstTagId) {
        this.alliance = alliance;
        this.side = side;
        this.firstTagId = firstTagId;
    }

    /** Which tag of this cluster {@code id} is, 0–3, or −1 if it is not one of ours. */
    public int tagIndex(int id) {
        int i = id - firstTagId;
        return i >= 0 && i < TAG_OFFSETS_IN.length ? i : -1;
    }

    /** Distance of tag {@code index} from the cluster centre along the row, inches. */
    public static double tagOffset(int index) {
        return TAG_OFFSETS_IN[index];
    }

    /** The other CELL on the same HIVE. */
    public Cell partner() {
        return of(alliance, side == Side.FAR ? Side.AUDIENCE : Side.FAR);
    }

    public static Cell of(Alliance alliance, Side side) {
        for (Cell c : values()) if (c.alliance == alliance && c.side == side) return c;
        throw new AssertionError();
    }

    /** The CELL whose cluster includes tag {@code id}, or {@code null}. */
    public static Cell forTag(int id) {
        for (Cell c : values()) if (c.tagIndex(id) >= 0) return c;
        return null;
    }
}
