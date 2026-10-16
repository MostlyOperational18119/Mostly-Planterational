package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.field.Cell;

/**
 * Where a CELL's tag cluster is relative to the robot this cycle: the fitted centre, robot
 * frame, inches (+x forward, +y left, +z up from the floor). Mutable and reused by
 * {@link org.firstinspires.ftc.teamcode.subsystems.Vision} — read it, don't keep it.
 */
public final class CellSighting {

    public final Cell cell;
    public int tagCount;
    public double x, y, z;

    public CellSighting(Cell cell) {
        this.cell = cell;
    }

    /** Direction to the cluster, radians, positive to the left — Pedro's turn direction. */
    public double bearing() {
        return Math.atan2(y, x);
    }

    /** Distance to the point on the floor under the cluster, inches. */
    public double floorDistance() {
        return Math.hypot(x, y);
    }
}
