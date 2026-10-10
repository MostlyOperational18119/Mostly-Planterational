package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.HubManager;

import java.util.concurrent.locks.LockSupport;
import java.util.function.BooleanSupplier;

/**
 * The whole robot below the OpMode, with no Android: the real {@link Robot}, {@link Follower}
 * (with the team's {@link Constants}), subsystems and Scheduler, over a simulated drivetrain,
 * localizer and motors. Tier 2 of the Testing section of CLAUDE.md.
 *
 * <p>Resets the static Scheduler on construction — one {@code SimRobot} per test.
 */
public final class SimRobot {

    /** Nominal loop period. Physics runs on real time, so this paces the test, not the sim. */
    private static final long LOOP_NANOS = 2_000_000L;

    public final SimDrivetrain drivetrain = new SimDrivetrain();
    public final SimLocalizer localizer = new SimLocalizer(drivetrain);
    public final FakeMotor intakeMotor = new FakeMotor();
    public final Follower follower;
    public final Robot robot;

    public SimRobot(Pose start) {
        Scheduler.reset();
        follower = new Follower(Constants.followerConstants, localizer, drivetrain,
                Constants.pathConstraints);
        robot = new Robot(HubManager.NONE, follower, new Intake(intakeMotor.motor));
        follower.setStartingPose(start);
        follower.update();                  // as the OpModes do between init and the loop
    }

    public void cycle() {
        robot.cycle();
        LockSupport.parkNanos(LOOP_NANOS);
    }

    /** Cycles until {@code condition} holds or {@code timeoutMs} of wall time passes.
     *  @return whether the condition was met */
    public boolean cycleUntil(BooleanSupplier condition, long timeoutMs) {
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;
        while (!condition.getAsBoolean()) {
            if (System.nanoTime() > deadline) return false;
            cycle();
        }
        return true;
    }

    public void cycles(int n) {
        for (int i = 0; i < n; i++) cycle();
    }

    public Pose pose() {
        return follower.getPose();
    }
}
