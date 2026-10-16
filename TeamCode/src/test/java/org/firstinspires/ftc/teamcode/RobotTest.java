package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.sim.FakeVisionSource;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Before;
import org.junit.Test;

/** What {@link Robot} adds on top of its subsystems. */
public class RobotTest {

    private SimRobot sim;

    @Before
    public void setUp() {
        sim = new SimRobot(new Pose(0, 0, 0));
        sim.camera.online = true;
    }

    @Test
    public void aimErrorSubtractsTheTurnSinceTheFrameWasCaptured() {
        sim.cycles(5);      // some heading history from before the shutter fired
        // Red's up CELL dead ahead (49 in up = the up position), captured now.
        sim.camera.sample = new FakeVisionSource.Frame(System.currentTimeMillis(), 0)
                .cluster(Cell.RED_FAR, 60, 0, 49).build();
        sim.cycle();
        assertEquals(0, sim.robot.aimError(Alliance.RED), 1e-6);

        // Turn CCW while the camera reports nothing newer: the target is now to the right.
        sim.robot.drive.teleopDrive(() -> 0, () -> 0, () -> 0.5, true).schedule();
        sim.cycleUntil(() -> sim.pose().getHeading() > 0.05, 80);
        double turned = sim.pose().getHeading();
        assertTrue("sim did not turn within the frame's 100 ms life", turned > 0.05);

        assertEquals("raw vision does not know the robot moved",
                0, sim.robot.vision.aimError(Alliance.RED), 1e-6);
        assertEquals(-turned, sim.robot.aimError(Alliance.RED), 0.01);
    }

    @Test
    public void aFrameOlderThanTheHistoryIsUsedUncorrected() {
        // Captured before any heading was recorded: no basis for a correction, so none.
        sim.camera.sample = new FakeVisionSource.Frame(System.currentTimeMillis() - 50, 0)
                .cluster(Cell.RED_FAR, 60, 20, 49).build();
        sim.cycle();
        assertEquals(sim.robot.vision.aimError(Alliance.RED),
                sim.robot.aimError(Alliance.RED), 0.0);
    }

    @Test
    public void noTargetIsStillNaN() {
        sim.cycle();
        assertTrue(Double.isNaN(sim.robot.aimError(Alliance.BLUE)));
    }

    @Test
    public void phaseSummaryReportsEveryPhase() {
        sim.cycles(5);
        String s = sim.robot.phaseSummary();
        for (String phase : new String[] {"read", "update", "execute", "write"}) {
            assertTrue(s, s.contains(phase));
        }
    }
}
