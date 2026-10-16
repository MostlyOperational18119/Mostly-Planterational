package org.firstinspires.ftc.teamcode.routines;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.AimController;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** The aim-and-shoot routine on the simulated robot, with a perfect camera. */
public class ShootingRoutinesTest {

    private SimRobot sim;

    @Before
    public void setUp() {
        sim = new SimRobot(new Pose(0, 0, 0));
    }

    @After
    public void restoreTuning() {
        ShootingRoutines.AIM_TIMEOUT_MS = 2000;
        ShootingRoutines.FEED_MS = 1500;
        Outtake.TARGET_RPM = 0;
    }

    /** The error to a fixed field point, from the simulated pose. */
    private double aimErrorTo(double targetX, double targetY) {
        Pose p = sim.pose();
        double dx = targetX - p.getX(), dy = targetY - p.getY();
        double h = p.getHeading();
        double x = dx * Math.cos(h) + dy * Math.sin(h);     // into the robot frame
        double y = -dx * Math.sin(h) + dy * Math.cos(h);
        return AimController.error(x, y);
    }

    private boolean fed() {
        return sim.transferMotor.writes().contains(Transfer.FEED_POWER);
    }

    @Test
    public void aimsThenFeedsThenReleasesEverything() {
        ShootingRoutines.AIM_TIMEOUT_MS = 4000;
        ShootingRoutines.FEED_MS = 300;
        Command routine = ShootingRoutines.aimAndShoot(sim.robot, () -> aimErrorTo(60, 40));
        routine.schedule();

        assertTrue("never started feeding", sim.cycleUntil(sim.robot.transfer::running, 4000));
        assertTrue("fed before it was aimed", AimController.onTarget(aimErrorTo(60, 40)));
        assertTrue("flywheel not spinning while feeding", sim.robot.outtake.running());

        assertTrue("never finished", sim.cycleUntil(() -> !routine.isScheduled(), 2000));
        sim.cycle();
        assertEquals(0.0, sim.transferMotor.power(), 0.0);
        assertEquals(0.0, sim.outtakeMotor.power(), 0.0);
        double heading = sim.pose().getHeading();
        sim.cycles(20);
        assertEquals("drive must stop turning once released", heading, sim.pose().getHeading(), 1e-3);
    }

    @Test
    public void withNoTargetItStillShootsAfterTheAimTimeout() {
        ShootingRoutines.AIM_TIMEOUT_MS = 300;
        ShootingRoutines.FEED_MS = 200;
        Command routine = ShootingRoutines.aimAndShoot(sim.robot, () -> Double.NaN);
        routine.schedule();
        long start = System.nanoTime();

        assertTrue("never finished", sim.cycleUntil(() -> !routine.isScheduled(), 3000));
        double ms = (System.nanoTime() - start) / 1e6;
        assertTrue("finished before the aim timeout: " + ms, ms >= 300);
        assertTrue(fed());
        assertEquals("nothing to aim at, so no turning", 0, sim.pose().getHeading(), 1e-3);
    }

    @Test
    public void underVelocityControlItWaitsForTheFlywheel() {
        Outtake.TARGET_RPM = 3000;
        ShootingRoutines.AIM_TIMEOUT_MS = 5000;
        Command routine = ShootingRoutines.aimAndShoot(sim.robot, () -> 0.0);
        routine.schedule();

        sim.cycles(20);
        assertFalse("fed into a flywheel at 0 rpm", fed());

        sim.outtakeMotor.setVelocity(3000 / 60.0 * Outtake.TICKS_PER_REV);
        assertTrue("never fed once at speed", sim.cycleUntil(this::fed, 200));
    }
}
