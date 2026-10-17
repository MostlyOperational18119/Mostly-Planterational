package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sim.FakeMotor;
import org.firstinspires.ftc.teamcode.sim.SimDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.HubManager;
import org.firstinspires.ftc.teamcode.vision.VisionSource;
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
        // As on the robot before AutoTune: no Foresight, which manual driving never consults.
        Follower follower = new Follower(new StubLocalizer(), drivetrain, new UntunedAlgorithm());
        robot = new Robot(HubManager.NONE, follower, new Intake(new FakeMotor().motor),
                new Transfer(new FakeMotor().motor), new Outtake(new FakeMotor().motor),
                new Vision(VisionSource.NONE));
        follower.setPose(Pose.zero());
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
        assertTrue(drivetrain.manual());
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
