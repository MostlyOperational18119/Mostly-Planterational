package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.sim.FakeVisionSource;
import org.firstinspires.ftc.teamcode.sim.FakeVisionSource.Frame;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Status;
import org.firstinspires.ftc.teamcode.vision.CellSighting;
import org.junit.Before;
import org.junit.Test;

public class VisionTest {

    private static final double UP = 49, DOWN = 34;    // cluster heights in each HIVE state

    private final FakeVisionSource camera = new FakeVisionSource();
    private long now = 1_000_000;
    private Vision vision;

    @Before
    public void setUp() {
        vision = new Vision(camera, () -> now);
        camera.online = true;
    }

    /** A frame captured 30 ms ago. */
    private Frame fresh() {
        return new Frame(now - 5, 25);
    }

    @Test
    public void aFullClusterGivesItsCentreRelativeToTheRobot() {
        camera.sample = fresh().cluster(Cell.RED_FAR, 60, 12, UP).build();
        vision.read();
        assertEquals(Status.FIX, vision.status());
        CellSighting s = vision.sighting(Cell.RED_FAR);
        assertEquals(4, s.tagCount);
        assertEquals(60, s.x, 1e-9);
        assertEquals(12, s.y, 1e-9);
        assertEquals(UP, s.z, 1e-9);
        assertEquals(Math.atan2(12, 60), s.bearing(), 1e-9);
        assertNull(vision.sighting(Cell.RED_AUDIENCE));
        assertEquals(30, vision.ageMs());
    }

    @Test
    public void tagsAreGroupedByCell() {
        camera.sample = fresh()
                .cluster(Cell.RED_FAR, 60, 12, UP)
                .cluster(Cell.RED_AUDIENCE, 70, -20, DOWN)
                .cluster(Cell.BLUE_FAR, 80, 30, DOWN, 0, 1, 0, 2, 3)
                .build();
        vision.read();
        assertEquals(-20, vision.sighting(Cell.RED_AUDIENCE).y, 1e-9);
        assertEquals(2, vision.sighting(Cell.BLUE_FAR).tagCount);
        assertEquals(30, vision.sighting(Cell.BLUE_FAR).y, 1e-9);
        assertNull(vision.sighting(Cell.BLUE_AUDIENCE));
    }

    @Test
    public void aLoneTagIsTooFew() {
        camera.sample = fresh().cluster(Cell.RED_FAR, 60, 0, UP, 0, 1, 0, 1).build();
        vision.read();
        assertEquals(Status.TOO_FEW_TAGS, vision.status());
        assertNull(vision.sighting(Cell.RED_FAR));
    }

    @Test
    public void tagsFromDifferentCellsAreNeverFittedTogether() {
        // One tag from each of two clusters: two tags in view, but no cluster has two.
        camera.sample = fresh().tag(30, 60, 0, UP).tag(34, 60, -20, DOWN).build();
        vision.read();
        assertEquals(Status.TOO_FEW_TAGS, vision.status());
    }

    @Test
    public void foreignTagsAreIgnored() {
        camera.sample = fresh().tag(7, 50, 0, 10).tag(8, 50, 5, 10).build();
        vision.read();
        assertEquals(Status.TOO_FEW_TAGS, vision.status());
    }

    @Test
    public void theHigherCellOfAHiveIsUp() {
        camera.sample = fresh()
                .cluster(Cell.BLUE_FAR, 60, 12, DOWN)
                .cluster(Cell.BLUE_AUDIENCE, 60, -12, UP)
                .build();
        vision.read();
        assertSame(Cell.BLUE_AUDIENCE, vision.upCell(Alliance.BLUE));
        assertNull("red HIVE not in view", vision.upCell(Alliance.RED));
    }

    @Test
    public void oneCellsHeightTellsWhichWayItsHiveIsTipped() {
        camera.sample = fresh().cluster(Cell.RED_FAR, 60, 0, UP).build();
        vision.read();
        assertSame(Cell.RED_FAR, vision.upCell(Alliance.RED));

        camera.sample = fresh().cluster(Cell.RED_FAR, 60, 0, DOWN).build();
        vision.read();
        assertSame("down, so the other end is up", Cell.RED_AUDIENCE, vision.upCell(Alliance.RED));
    }

    @Test
    public void aimErrorPointsAtTheUpCell() {
        camera.sample = fresh()
                .cluster(Cell.RED_FAR, 60, 20, DOWN)
                .cluster(Cell.RED_AUDIENCE, 60, -20, UP)
                .build();
        vision.read();
        assertEquals(Math.atan2(-20, 60), vision.aimError(Alliance.RED), 1e-9);
        assertTrue("blue HIVE not in view", Double.isNaN(vision.aimError(Alliance.BLUE)));
    }

    @Test
    public void aimErrorIsNaNWhenTheUpCellIsOnlyInferred() {
        // Only the down CELL in view: we know its partner is up, but not where it is.
        camera.sample = fresh().cluster(Cell.RED_FAR, 60, 20, DOWN).build();
        vision.read();
        assertSame(Cell.RED_AUDIENCE, vision.upCell(Alliance.RED));
        assertTrue(Double.isNaN(vision.aimError(Alliance.RED)));
    }

    @Test
    public void offlineCameraHasNoFix() {
        camera.online = false;
        camera.sample = fresh().cluster(Cell.RED_FAR, 60, 0, UP).build();
        vision.read();
        assertEquals(Status.OFFLINE, vision.status());
        assertNull(vision.sighting(Cell.RED_FAR));
        assertFalse(vision.online());
    }

    @Test
    public void noResultIsNoTarget() {
        vision.read();
        assertEquals(Status.NO_TARGET, vision.status());
    }

    @Test
    public void ageIsMeasuredFromTheShutterNotTheReply() {
        // Parsed 70 ms ago is fresh by getStaleness(), but with 40 ms of pipeline the frame
        // is 110 ms old — past MAX_AGE_MS.
        camera.sample = new Frame(now - 70, 40).cluster(Cell.RED_FAR, 60, 0, UP).build();
        vision.read();
        assertEquals(Status.STALE, vision.status());
    }

    @Test
    public void aFixGoesStaleWhenTheCameraStopsUpdating() {
        camera.sample = fresh().cluster(Cell.RED_FAR, 60, 0, UP).build();
        vision.read();
        assertTrue(vision.hasFix());
        now += Vision.MAX_AGE_MS;
        vision.read();
        assertEquals(Status.STALE, vision.status());
        assertNull("a stale sighting must not leak out", vision.sighting(Cell.RED_FAR));
        assertNull(vision.upCell(Alliance.RED));
    }

    @Test
    public void aCellThatLeavesTheFrameIsDropped() {
        camera.sample = fresh().cluster(Cell.RED_FAR, 60, 0, UP).cluster(Cell.BLUE_FAR, 60, 30, UP).build();
        vision.read();
        camera.sample = fresh().cluster(Cell.BLUE_FAR, 60, 30, UP).build();
        vision.read();
        assertNull(vision.sighting(Cell.RED_FAR));
        assertTrue(vision.sighting(Cell.BLUE_FAR) != null);
    }

    @Test
    public void stopClosesTheCamera() {
        vision.stop();
        assertTrue(camera.closed);
    }
}
