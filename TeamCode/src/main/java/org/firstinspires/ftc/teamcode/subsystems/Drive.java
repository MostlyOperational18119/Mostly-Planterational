package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.Subsystem;

import java.util.function.DoubleSupplier;

/**
 * The drivetrain, as a requirable token wrapped around Pedro's {@link Follower}.
 *
 * <p>It exists for one reason above all: Ivy 1.0.0's {@code PedroCommands.follow/hold/turnTo}
 * declare <b>no requirements</b>, so without this wrapper two concurrently scheduled paths
 * silently fight over the drivetrain with no conflict detected (architecture doc §2.1, §5.3).
 * Every factory here appends {@code .requiring(this)}.
 *
 * <p>Two notes on where the hardware I/O actually happens:
 *
 * <ul>
 *   <li>Pedro writes the motors inside {@code follower.update()}, which the {@code Robot} runs
 *       in phase 3 — so there is nothing here for {@link org.firstinspires.ftc.teamcode.util.WriteGate}
 *       to gate, and {@link #read()} is empty. The pose is refreshed by that same call, so
 *       {@link #pose()} is a live field read rather than something cached in {@code read()};
 *       a command in phase 4 sees a pose from phase 3 of the <em>same</em> iteration.</li>
 *   <li>{@link #write()} only hands Pedro the TeleOp drive vectors; they take effect on the
 *       next {@code follower.update()}. That is one loop period — ~6 ms, the latency the whole
 *       design budgets for.</li>
 * </ul>
 */
public class Drive implements Subsystem {

    private final Follower follower;

    // --- setpoints, mutated by commands in phase 4 ---
    private boolean teleopEnabled;
    private double forward, strafe, turn;
    private boolean robotCentric = true;

    private boolean teleopWasEnabled;

    public Drive(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void read() {
        // Pedro's localization refresh is follower.update() in phase 3. Nothing to do.
    }

    @Override
    public void write() {
        if (teleopEnabled) {
            // Self-healing rather than done in the command's start(): the Scheduler resumes a
            // SUSPENDed command without calling start() again, and a path command that ran in
            // between will have taken the follower out of TeleOp mode.
            if (!follower.isTeleopDrive()) follower.startTeleOpDrive();
            follower.setTeleOpDrive(forward, strafe, turn, robotCentric);
        } else if (teleopWasEnabled && follower.isTeleopDrive()) {
            // Falling edge: push a zero so the robot does not coast on a stale drive vector.
            follower.setTeleOpDrive(0, 0, 0, robotCentric);
        }
        teleopWasEnabled = teleopEnabled;
    }

    @Override
    public void telemetry(TelemetryManager t) {
        Pose pose = follower.getPose();
        t.addLine(String.format("drive: x %.1f y %.1f h %.2frad%s",
                pose.getX(), pose.getY(), pose.getHeading(), stuck() ? "  STUCK" : ""));
    }

    @Override
    public void stop() {
        teleopEnabled = false;
        if (follower.isTeleopDrive()) follower.setTeleOpDrive(0, 0, 0, robotCentric);
        follower.breakFollowing();
    }

    // --- queries: safe in phase 4 ---

    /** Refreshed by {@code follower.update()} in phase 3 of this same iteration. */
    public Pose pose() {
        return follower.getPose();
    }

    /** Progress along the current path, 0–1. See {@link #waitForT(double)}. */
    public double t() {
        return follower.getCurrentTValue();
    }

    /** A supervisor command should watch this and bail out to a park. Architecture doc §8.4. */
    public boolean stuck() {
        return follower.isRobotStuck() || follower.isLocalizationNAN();
    }

    // --- commands ---

    public CommandBuilder follow(PathChain path) {
        return PedroCommands.follow(follower, path).requiring(this);
    }

    public CommandBuilder follow(PathChain path, double maxPower) {
        return PedroCommands.follow(follower, path, maxPower).requiring(this);
    }

    /** @param radians heading in radians — the unit convention at every API boundary. */
    public CommandBuilder turnTo(double radians) {
        return PedroCommands.turnTo(follower, radians).requiring(this);
    }

    public CommandBuilder hold(Pose pose) {
        return PedroCommands.hold(follower, pose).requiring(this);
    }

    /**
     * Fires when the follower passes {@code t} along the current path — the overlap primitive:
     * {@code deadline(drive.follow(p), sequential(drive.waitForT(0.55), lift.goTo(HIGH)))}.
     *
     * <p>Declares no requirement on purpose: it observes the drivetrain, it does not claim it.
     */
    public CommandBuilder waitForT(double t) {
        return Commands.waitUntil(() -> follower.getCurrentTValue() >= t);
    }

    /**
     * Driver control, as a proper command so that the drivetrain has exactly one owner and an
     * automated move can preempt it through the requirement system.
     *
     * <p>Interrupted behaviour is {@code SUSPEND}, so when a higher-priority (or equal-priority,
     * overriding) drive command such as {@link #turnTo(double)} finishes and releases the
     * requirement, driver control resumes by itself. Suspending is safe here only because this
     * is a leaf command — {@code Sequential}/{@code Repeat} throw when suspended (§2.1).
     *
     * <p>Suppliers rather than values: this command is built once, before the match, and reads
     * the gamepad each cycle.
     */
    public CommandBuilder teleopDrive(DoubleSupplier forward, DoubleSupplier strafe,
                                      DoubleSupplier turn, boolean robotCentric) {
        return Command.build()
                .setExecute(() -> {
                    this.teleopEnabled = true;
                    this.forward = forward.getAsDouble();
                    this.strafe = strafe.getAsDouble();
                    this.turn = turn.getAsDouble();
                    this.robotCentric = robotCentric;
                })
                .setDone(() -> false)
                .setEnd(endCondition -> this.teleopEnabled = false)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .requiring(this);
    }
}
