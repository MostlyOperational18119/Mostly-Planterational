package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sim.FakeMotor;
import org.firstinspires.ftc.teamcode.sim.SimDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.HubManager;
import org.junit.Before;
import org.junit.Test;

/** Robot-centric TeleOp on the no-odometry chassis: sticks must reach the wheels unchanged. */
public class StubLocalizerTest {

    private final SimDrivetrain drivetrain = new SimDrivetrain();
    private Robot robot;
    private double forward, strafe, turn;

    @Before
    public void setUp() {
        Scheduler.reset();
        Follower follower = new Follower(Constants.followerConstants, new StubLocalizer(),
                drivetrain, Constants.pathConstraints);
        robot = new Robot(HubManager.NONE, follower, new Intake(new FakeMotor().motor));
        follower.setStartingPose(new Pose());
        follower.update();
        robot.drive.teleopDrive(() -> forward, () -> strafe, () -> turn, true).schedule();
    }

    private void cycles(int n) {
        for (int i = 0; i < n; i++) robot.cycle();
    }

    @Test
    public void forwardStickDrivesForward() {
        forward = 0.5;
        cycles(3);
        assertTrue(drivetrain.teleopStarted());
        assertTrue(drivetrain.vx() > 0);
        assertEquals(0, drivetrain.vy(), 1e-9);
        assertEquals(0, drivetrain.omega(), 1e-9);
    }

    @Test
    public void strafeStickStrafesLeft() {
        strafe = 0.5;                       // DriveTeleOp passes -left_stick_x: positive is left
        cycles(3);
        assertTrue(drivetrain.vy() > 0);
        assertEquals(0, drivetrain.vx(), 1e-9);
    }

    @Test
    public void turnStickTurnsCounterClockwise() {
        turn = 0.5;
        cycles(3);
        assertTrue(drivetrain.omega() > 0);
        assertEquals(0, drivetrain.vx(), 1e-9);
        assertEquals(0, drivetrain.vy(), 1e-9);
    }

    @Test
    public void releasingTheSticksStops() {
        forward = 0.5;
        cycles(3);
        forward = 0;
        cycles(3);
        assertEquals(0, drivetrain.vx(), 1e-9);
    }
}
