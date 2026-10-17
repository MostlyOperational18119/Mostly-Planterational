package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.teamcode.util.AimController;
import org.firstinspires.ftc.teamcode.util.Timeouts;
import org.junit.Before;
import org.junit.Test;

public class DriveTest {

    private SimRobot sim;
    private Drive drive;

    @Before
    public void setUp() {
        sim = new SimRobot(new Pose(0, 0, 0));
        drive = sim.robot.drive;
    }

    private static Path line(double x, double y) {
        return Paths.line(new Pose(0, 0, 0), new Pose(x, y, 0)).constant(0);
    }

    @Test
    public void followReachesTheEndOfThePath() {
        Command follow = drive.follow(line(24, 0));
        follow.schedule();
        assertTrue("path never finished", sim.cycleUntil(() -> !follow.isScheduled(), 5000));
        assertEquals(24, sim.pose().x(), 1.0);
    }

    @Test
    public void interruptedFollowStopsTheFollower() {
        Command limited = Timeouts.limit(drive.follow(line(200, 0)), 100);
        limited.schedule();
        assertTrue(sim.cycleUntil(() -> !limited.isScheduled(), 2000));
        sim.cycles(5);
        assertFalse(sim.follower.isBusy());
        assertFalse(sim.drivetrain.isCommandingMotion());
        double x = sim.pose().x();
        sim.cycles(20);
        assertEquals("robot kept moving after its path was cut short", x, sim.pose().x(), 1e-9);
    }

    @Test
    public void aPreemptingPathIsNotBrokenByThePathItReplaced() {
        drive.follow(line(200, 0)).schedule();
        sim.cycles(10);
        Command second = drive.follow(line(0, 200));
        second.schedule();
        sim.cycles(10);
        double y = sim.pose().y();
        sim.cycles(20);
        assertTrue(second.isScheduled());
        assertTrue(sim.follower.isBusy());
        assertTrue("second path should still be driving", sim.pose().y() > y + 1);
    }

    @Test
    public void driverControlResumesAfterAnAutomatedTurn() {
        Command driver = drive.teleopDrive(() -> 0, () -> 0, () -> 0, true);
        driver.schedule();
        sim.cycles(3);

        Command turn = drive.turnTo(Math.PI / 2);
        turn.schedule();
        sim.cycle();
        assertFalse("turn should suspend driver control", Scheduler.isRunning(driver));

        assertTrue("turn never finished", sim.cycleUntil(() -> !turn.isScheduled(), 4000));
        sim.cycles(2);
        assertTrue(Scheduler.isRunning(driver));
        assertTrue("driver control should be back in manual mode", sim.follower.manual());
        // Regression: Pedro 2's PedroCommands.turnTo ended after its 100 ms path-end timeout.
        assertEquals(Math.PI / 2, sim.pose().heading(), Math.toRadians(3));
    }

    @Test
    public void turnToFinishesOnlyAtTheHeading() {
        Command turn = drive.turnTo(-Math.PI / 2);
        turn.schedule();
        assertTrue(sim.cycleUntil(() -> !turn.isScheduled(), 4000));
        assertEquals(-Math.PI / 2, Angle.normalizeSigned(sim.pose().heading()), Math.toRadians(2.5));
        assertEquals(0, sim.pose().x(), 0.5);
        assertEquals(0, sim.pose().y(), 0.5);
    }

    @Test
    public void holdGoesToThePosesHeadingNotTheCurrentOne() {
        Pose target = new Pose(6, 4, Math.PI / 2);
        Command hold = drive.hold(target);
        hold.schedule();
        assertTrue(sim.cycleUntil(() -> !hold.isScheduled(), 4000));
        assertEquals(Math.PI / 2, sim.pose().heading(), Math.toRadians(2.5));
        assertTrue(sim.pose().distance(target) < 1.0);
    }

    /** A perfect camera: the error to a fixed field point, from the simulated pose. */
    private double aimErrorTo(double targetX, double targetY) {
        Pose p = sim.pose();
        double dx = targetX - p.x(), dy = targetY - p.y();
        double h = p.heading();
        double x = dx * Math.cos(h) + dy * Math.sin(h);     // into the robot frame
        double y = -dx * Math.sin(h) + dy * Math.cos(h);
        return AimController.error(x, y);
    }

    @Test
    public void aimingTurnsTheRobotOntoTheTargetFromEitherSide() {
        for (double targetY : new double[] {40, -40}) {
            setUp();
            Command aim = drive.aimedTeleopDrive(() -> 0, () -> 0, () -> 0,
                    () -> aimErrorTo(60, targetY));
            aim.schedule();
            assertTrue("never settled on target at y=" + targetY, sim.cycleUntil(() ->
                    AimController.onTarget(aimErrorTo(60, targetY)), 4000));
            sim.cycles(50);
            assertTrue("drifted off target at y=" + targetY,
                    AimController.onTarget(aimErrorTo(60, targetY)));
            assertEquals("aiming must not translate", 0, sim.pose().x(), 0.5);
        }
    }

    @Test
    public void withNoTargetTheDriversTurnStickApplies() {
        Command aim = drive.aimedTeleopDrive(() -> 0, () -> 0, () -> 0.5, () -> Double.NaN);
        aim.schedule();
        sim.cycles(30);
        assertTrue("manual turn should rotate CCW", sim.pose().heading() > 0.1);
    }

    @Test
    public void releasingAimResumesPlainDriverControl() {
        Command driver = drive.teleopDrive(() -> 0.3, () -> 0, () -> 0, true);
        driver.schedule();
        sim.cycles(3);
        Command aim = drive.aimedTeleopDrive(() -> 0, () -> 0, () -> 0, () -> 0.0);
        aim.schedule();
        sim.cycle();
        assertFalse(Scheduler.isRunning(driver));
        aim.cancel();
        sim.cycles(2);
        assertTrue(Scheduler.isRunning(driver));
    }

    @Test
    public void localizationLostIsReported() {
        assertFalse(drive.localizationLost());
        sim.localizer.loseLocalization();
        sim.cycle();
        assertTrue(drive.localizationLost());
    }
}
