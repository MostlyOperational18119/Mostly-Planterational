package org.firstinspires.ftc.teamcode.routines;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.auto.TestPaths;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Timeouts;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;

/** The test auto's routine, stepped headlessly through {@code robot.cycle()}. */
public class TestRoutinesTest {

    private SimRobot sim;
    private TestPaths paths;

    @Before
    public void setUp() {
        sim = new SimRobot(TestPaths.START);
        paths = new TestPaths(sim.follower);
    }

    @Test
    public void outAndBackFinishesHomeWithTheIntakeOff() {
        Command routine = TestRoutines.outAndBack(sim.robot, paths);
        routine.schedule();
        long start = System.nanoTime();

        assertTrue("routine never finished", sim.cycleUntil(() -> !routine.isScheduled(), 12_000));
        double seconds = (System.nanoTime() - start) / 1e9;
        // Either leg timing out takes LEG_TIMEOUT_MS (5 s) on its own, so finishing well inside
        // that proves both paths completed rather than being cut off.
        assertTrue("took " + seconds + " s", seconds < 4.5);
        assertTrue(sim.pose().distanceFrom(TestPaths.START) < 1.5);
        assertEquals(0.0, sim.intakeMotor.power(), 0.0);
    }

    @Test
    public void intakeStartsPartwayOutAndEjectsOnTheWayBack() {
        Command routine = TestRoutines.outAndBack(sim.robot, paths);
        routine.schedule();

        double xWhenCollectStarted = Double.NaN;
        boolean ejectedWhileReturning = false;
        while (routine.isScheduled()) {
            sim.cycle();
            double power = sim.robot.intake.power();
            if (power == Intake.COLLECT_POWER && Double.isNaN(xWhenCollectStarted)) {
                xWhenCollectStarted = sim.pose().getX();
            }
            if (power == Intake.EJECT_POWER) ejectedWhileReturning = true;
        }

        // INTAKE_FROM_T = 0.3 of a 24 in line → ~7.2 in; generous bounds for the sim's step size.
        assertTrue("collect began at x=" + xWhenCollectStarted,
                xWhenCollectStarted > 5 && xWhenCollectStarted < 10);
        assertTrue(ejectedWhileReturning);
        // The gate's first write, then one per real change. The 0 between collect and eject is
        // one loop long: Ivy's Sequential starts its next step the cycle after the last ends.
        assertEquals(Arrays.asList(0.0, Intake.COLLECT_POWER, 0.0, Intake.EJECT_POWER, 0.0),
                sim.intakeMotor.writes());
    }

    @Test
    public void losingLocalizationAbortsTheRoutineAndStopsEverything() {
        Command routine = TestRoutines.outAndBack(sim.robot, paths);
        Command guarded = Groups.race(
                routine,
                Timeouts.command(28_000),
                Commands.waitUntil(sim.robot.drive::localizationLost));
        guarded.schedule();

        assertTrue(sim.cycleUntil(() -> sim.robot.intake.power() == Intake.COLLECT_POWER, 5000));
        sim.localizer.loseLocalization();
        assertTrue(sim.cycleUntil(() -> !guarded.isScheduled(), 1000));
        sim.cycle();

        assertFalse(routine.isScheduled());
        assertFalse(sim.follower.isBusy());
        assertFalse(sim.drivetrain.isCommandingMotion());
        assertEquals(0.0, sim.intakeMotor.power(), 0.0);
    }
}
