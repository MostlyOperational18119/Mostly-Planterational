package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.util.AimController;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.vision.CellSighting;
import org.firstinspires.ftc.teamcode.vision.ClusterFit;
import org.firstinspires.ftc.teamcode.vision.LimelightSource;
import org.firstinspires.ftc.teamcode.vision.VisionSample;
import org.firstinspires.ftc.teamcode.vision.VisionSource;

import java.util.function.LongSupplier;

/**
 * Where each HIVE CELL is relative to the robot, and which CELL of each HIVE is facing up.
 *
 * <p>Robot-relative on purpose: the CELLS tip during the match and every tag is on a CELL, so
 * there is no fixed landmark to localize against (see {@link LimelightSource}). Aiming needs
 * the target relative to the robot anyway, and this measures exactly that — with or without
 * odometry.
 *
 * <p>Samples once in {@link #read()}; everything it exposes is then stable for the cycle. It
 * writes nothing, requires nothing and never schedules (architecture doc §7.5).
 */
public class Vision implements Subsystem {

    public enum Status { OFFLINE, NO_TARGET, TOO_FEW_TAGS, STALE, FIX }

    /** A cluster centre needs two tags of it; one tag leaves the row's direction unknown. */
    public static final int MIN_TAGS_PER_CELL = 2;

    /** Shutter-to-now. Pipeline latency is typically 20–40 ms, the driver poll adds ≤10. */
    public static final long MAX_AGE_MS = 100;

    /**
     * A cluster higher than this is on the up CELL. Midway between the two states' cluster
     * heights, from the manual's HIVE drawings (§9.6, Fig. 9-9/9-10): pivot 43.95 in up, 30°
     * tip, the down CELL's lowest corner 30.6 in up ⇒ CELL bottoms ~3 in below the pivot axis
     * ⇒ clusters at ~49 in (up) and ~34 in (down). The ±7.5 in margin covers drawing error,
     * but measure both states on a real HIVE and recentre this.
     */
    public static final double UP_CELL_MIN_HEIGHT_IN = 41.3;

    private static final Cell[] CELLS = Cell.values();

    private final VisionSource source;
    private final LongSupplier wallClockMs;

    // --- refreshed in read(), stable for the cycle ---
    private Status status = Status.OFFLINE;
    private VisionSample sample;
    private VisionSample processedSample;
    private final CellSighting[] sightings = new CellSighting[CELLS.length];
    private final boolean[] seen = new boolean[CELLS.length];
    // why a cell has no fix, for telemetry: how many of its tags matched, and the fit's spacing
    private final int[] matched = new int[CELLS.length];
    private final double[] spacing = new double[CELLS.length];

    // scratch for the fit, sized for one cluster
    private final double[] offset = new double[4], fx = new double[4], fy = new double[4],
            fz = new double[4], center = new double[4];

    public static Vision fromHardwareMap(HardwareMap hardwareMap) {
        return new Vision(LimelightSource.fromHardwareMap(hardwareMap));
    }

    public Vision(VisionSource source) {
        // currentTimeMillis, not nanoTime: the Limelight's timestamps are wall clock (§7.4).
        this(source, System::currentTimeMillis);
    }

    /** @param wallClockMs injectable so tests can age a sample by hand. */
    public Vision(VisionSource source, LongSupplier wallClockMs) {
        this.source = source;
        this.wallClockMs = wallClockMs;
        for (int i = 0; i < CELLS.length; i++) sightings[i] = new CellSighting(CELLS[i]);
    }

    @Override
    public void read() {
        sample = source.latest();
        if (sample != processedSample) {
            fitClusters(sample);
            processedSample = sample;
        }
        status = evaluate(source.online(), wallClockMs.getAsLong());
    }

    private void fitClusters(VisionSample s) {
        for (int c = 0; c < CELLS.length; c++) {
            seen[c] = false;
            matched[c] = 0;
            spacing[c] = Double.NaN;
            if (s == null || !s.valid) continue;
            Cell cell = CELLS[c];
            int n = 0;
            for (int t = 0; t < s.tagCount() && n < 4; t++) {
                int index = cell.tagIndex(s.tagIds[t]);
                if (index < 0) continue;
                offset[n] = Cell.tagOffset(index);
                fx[n] = s.tagX[t];
                fy[n] = s.tagY[t];
                fz[n] = s.tagZ[t];
                n++;
            }
            matched[c] = n;
            if (n < MIN_TAGS_PER_CELL) continue;
            center[3] = Double.NaN;
            boolean fitted = ClusterFit.center(n, offset, fx, fy, fz, center);
            spacing[c] = center[3];
            if (!fitted) continue;
            CellSighting sighting = sightings[c];
            sighting.tagCount = n;
            sighting.x = center[0];
            sighting.y = center[1];
            sighting.z = center[2];
            seen[c] = true;
        }
    }

    private Status evaluate(boolean online, long nowMs) {
        if (!online) return Status.OFFLINE;
        if (sample == null || !sample.valid || sample.tagCount() == 0) return Status.NO_TARGET;
        if (nowMs - sample.captureTimeMs() > MAX_AGE_MS) return Status.STALE;
        for (boolean s : seen) if (s) return Status.FIX;
        return Status.TOO_FEW_TAGS;
    }

    @Override
    public void write() {}

    @Override
    public void telemetry(TelemetryManager t) {
        if (status != Status.FIX) {
            t.addLine("vision: " + status
                    + (sample != null ? " (" + sample.tagCount() + " tags seen)" : ""));
            if (status == Status.TOO_FEW_TAGS) t.addLine(fitDiagnosis());
            return;
        }
        StringBuilder line = new StringBuilder(String.format("vision: FIX age %dms", ageMs()));
        for (Alliance a : Alliance.values()) {
            Cell up = upCell(a);
            line.append(String.format(" | %s up: %s", a, up == null ? "?" : up.side));
        }
        t.addLine(line.toString());
        for (Cell cell : CELLS) {
            CellSighting s = sighting(cell);
            if (s == null) continue;
            // spacing: measured/true tag spacing, ~1.00 when the pipeline's tag size and the
            // tag layout (Cell offsets) both match the field
            t.addLine(String.format("  %s %dt fwd %.1f left %.1f up %.1f → %.1fin %+.1f° spacing x%.2f",
                    cell, s.tagCount, s.x, s.y, s.z, s.floorDistance(),
                    Math.toDegrees(s.bearing()), spacing[cell.ordinal()]));
        }
    }

    /**
     * Why no CELL has a fix although tags are in view: the IDs seen (ours are 30–45), and for
     * each CELL with tags, how many matched and the fit's measured/true tag spacing. A spacing
     * outside 0.75–1.25 is rejected — almost always the pipeline's tag size.
     */
    private String fitDiagnosis() {
        StringBuilder line = new StringBuilder("  tag IDs:");
        for (int t = 0; t < sample.tagCount(); t++) line.append(' ').append(sample.tagIds[t]);
        for (int c = 0; c < CELLS.length; c++) {
            if (matched[c] == 0) continue;
            line.append(" | ").append(CELLS[c]).append(' ').append(matched[c]).append('t');
            if (matched[c] < MIN_TAGS_PER_CELL) line.append(" (need 2)");
            else if (!Double.isNaN(spacing[c])) {
                line.append(String.format(" spacing x%.2f%s", spacing[c], seen[c] ? ""
                        : " REJECTED (need 0.75–1.25: check pipeline tag size)"));
            }
        }
        return line.toString();
    }

    @Override
    public void stop() {
        source.close();
    }

    // --- queries: safe in phase 4 ---

    public Status status() {
        return status;
    }

    public boolean hasFix() {
        return status == Status.FIX;
    }

    /** This cycle's fix on {@code cell}'s cluster, or {@code null}. Do not keep it. */
    public CellSighting sighting(Cell cell) {
        return hasFix() && seen[cell.ordinal()] ? sightings[cell.ordinal()] : null;
    }

    /**
     * Which of {@code alliance}'s CELLS is facing up — the one to launch into — or
     * {@code null} if neither is in view. With both in view the higher wins; with one, its
     * height against {@link #UP_CELL_MIN_HEIGHT_IN} decides.
     */
    public Cell upCell(Alliance alliance) {
        CellSighting far = sighting(Cell.of(alliance, Cell.Side.FAR));
        CellSighting audience = sighting(Cell.of(alliance, Cell.Side.AUDIENCE));
        if (far != null && audience != null) return far.z >= audience.z ? far.cell : audience.cell;
        CellSighting one = far != null ? far : audience;
        if (one == null) return null;
        return one.z >= UP_CELL_MIN_HEIGHT_IN ? one.cell : one.cell.partner();
    }

    /**
     * How far the robot must turn to aim the launcher at {@code alliance}'s up CELL, radians,
     * CCW positive; NaN when that CELL is not in view. Aims at the cluster centre under the
     * CELL — the opening is offset a few inches from it.
     */
    public double aimError(Alliance alliance) {
        Cell up = upCell(alliance);
        CellSighting target = up == null ? null : sighting(up);
        return target == null ? Double.NaN : AimController.error(target.x, target.y);
    }

    /**
     * Floor distance to {@code alliance}'s up CELL's tag cluster, inches; NaN when that CELL is
     * not in view. What the launch speed is chosen by ({@code Outtake.SHOT_POINTS}).
     */
    public double targetDistance(Alliance alliance) {
        Cell up = upCell(alliance);
        CellSighting target = up == null ? null : sighting(up);
        return target == null ? Double.NaN : target.floorDistance();
    }

    /** Wall-clock ms when the frame behind this cycle's sightings was captured (§7.4). */
    public long captureTimeMs() {
        return hasFix() ? sample.captureTimeMs() : Long.MIN_VALUE;
    }

    public long ageMs() {
        return sample == null ? Long.MAX_VALUE : wallClockMs.getAsLong() - sample.captureTimeMs();
    }

    public boolean online() {
        return status != Status.OFFLINE;
    }
}
