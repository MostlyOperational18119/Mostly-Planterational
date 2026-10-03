package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.sim.SimRobot;
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

    private PathChain line(double x, double y) {
        return sim.follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(x, y, 0)))
                .setConstantHeadingInterpolation(0)
                .build();
    }

    @Test
    public void followReachesTheEndOfThePath() {
        Command follow = drive.follow(line(24, 0));
        follow.schedule();
        assertTrue("path never finished", sim.cycleUntil(() -> !follow.isScheduled(), 5000));
        assertEquals(24, sim.pose().getX(), 1.0);
    }

    @Test
    public void interruptedFollowStopsTheFollower() {
        Command limited = Timeouts.limit(drive.follow(line(200, 0)), 100);
        limited.schedule();
        assertTrue(sim.cycleUntil(() -> !limited.isScheduled(), 2000));
        sim.cycle();                                    // write() acts on the request
        sim.cycles(5);
        assertFalse(sim.follower.isBusy());
        assertFalse(sim.drivetrain.isCommandingMotion());
        double x = sim.pose().getX();
        sim.cycles(20);
        assertEquals("robot kept moving after its path was cut short", x, sim.pose().getX(), 1e-9);
    }

    @Test
    public void aPreemptingPathIsNotBrokenByThePathItReplaced() {
        drive.follow(line(200, 0)).schedule();
        sim.cycles(10);
        Command second = drive.follow(line(0, 200));
        second.schedule();
        sim.cycles(10);
        double y = sim.pose().getY();
        sim.cycles(20);
        assertTrue(second.isScheduled());
        assertTrue(sim.follower.isBusy());
        assertTrue("second path should still be driving", sim.pose().getY() > y + 1);
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
        assertTrue(sim.follower.isTeleopDrive());
        // Regression: PedroCommands.turnTo ended after its 100 ms path-end timeout, ~0.6 rad in.
        assertEquals(Math.PI / 2, sim.pose().getHeading(), Math.toRadians(3));
    }

    @Test
    public void turnToFinishesOnlyAtTheHeading() {
        Command turn = drive.turnTo(-Math.PI / 2);
        turn.schedule();
        assertTrue(sim.cycleUntil(() -> !turn.isScheduled(), 4000));
        assertEquals(-Math.PI / 2, sim.pose().getHeading(), Math.toRadians(2.5));
        assertEquals(0, sim.pose().getX(), 0.5);
        assertEquals(0, sim.pose().getY(), 0.5);
    }

    @Test
    public void holdGoesToThePosesHeadingNotTheCurrentOne() {
        Pose target = new Pose(6, 4, Math.PI / 2);
        Command hold = drive.hold(target);
        hold.schedule();
        assertTrue(sim.cycleUntil(() -> !hold.isScheduled(), 4000));
        assertEquals(Math.PI / 2, sim.pose().getHeading(), Math.toRadians(2.5));
        assertTrue(sim.pose().distanceFrom(target) < 1.0);
    }

    @Test
    public void localizationLostIsReported() {
        assertFalse(drive.localizationLost());
        sim.localizer.loseLocalization();
        sim.cycle();
        assertTrue(drive.localizationLost());
    }
}
