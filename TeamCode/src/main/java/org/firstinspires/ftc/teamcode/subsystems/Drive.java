package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.MathFunctions;
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
 * <p>The second reason: Ivy's Pedro commands have no {@code end()} handler, so a path that is
 * <em>interrupted</em> — by a timeout, a race, a preempting command — leaves the follower
 * still chasing it. {@link #owned(Command)} fixes both problems at once.
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

    /** {@link #turnTo} and {@link #hold} finish inside these; the follower keeps correcting. */
    private static final double HEADING_TOLERANCE = Math.toRadians(2);
    private static final double POSITION_TOLERANCE_IN = 1.0;

    private final Follower follower;

    // --- setpoints, mutated by commands in phase 4 ---
    private boolean teleopEnabled;
    private double forward, strafe, turn;
    private boolean robotCentric = true;

    private boolean teleopWasEnabled;

    // Set when a path command is interrupted; acted on in write(). See owned().
    private boolean breakRequested;

    public Drive(Follower follower) {
        this.follower = follower;
    }

    @Override
    public void read() {
        // Pedro's localization refresh is follower.update() in phase 3. Nothing to do.
    }

    @Override
    public void write() {
        if (breakRequested) {
            // Writes all four drive motors plus their zero-power mode: a one-off long loop on
            // an abort, which is why it is here and not in a command's end().
            breakRequested = false;
            follower.breakFollowing();
        }
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
        breakRequested = false;
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

    /**
     * Diagnostic only — <b>not</b> an abort signal. {@code isRobotStuck()} turns true the
     * first cycle Pedro sees near-zero velocity mid-path, before its own {@code stuckTimeout}
     * has run; once that expires Pedro ends the path itself and the follow command finishes.
     * Abort on {@link #localizationLost()} and on timeouts instead.
     */
    public boolean stuck() {
        return follower.isRobotStuck() || follower.isLocalizationNAN();
    }

    /** The pose is NaN: no path can be followed. A routine should abort on this (§8.4). */
    public boolean localizationLost() {
        return follower.isLocalizationNAN();
    }

    // --- commands ---

    public CommandBuilder follow(PathChain path) {
        return owned(PedroCommands.follow(follower, path));
    }

    public CommandBuilder follow(PathChain path, double maxPower) {
        return owned(PedroCommands.follow(follower, path, maxPower));
    }

    /**
     * Turns in place, finishing when the heading is within {@link #HEADING_TOLERANCE}. The
     * follower keeps holding the heading afterwards until something else takes the drive.
     *
     * <p>Not {@code PedroCommands.turnTo}: that follows a zero-length path, which is at its
     * parametric end from the first cycle, so Pedro ends it after the path-end timeout
     * ({@code PathConstraints} timeout, 100 ms here) whatever the heading. Bound this with
     * {@code Timeouts.limit}: it does not finish if the robot cannot turn.
     *
     * @param radians heading in radians — the unit convention at every API boundary.
     */
    public CommandBuilder turnTo(double radians) {
        return owned(Command.build()
                .setStart(() -> follower.holdPoint(new BezierPoint(follower.getPose()), radians, false))
                .setDone(() -> headingError(radians) < HEADING_TOLERANCE));
    }

    /**
     * Drives to and holds {@code pose}, finishing once inside {@link #POSITION_TOLERANCE_IN}
     * and {@link #HEADING_TOLERANCE}; the hold continues after that.
     *
     * <p>Not {@code PedroCommands.hold}: that holds the robot's <em>current</em> heading rather
     * than {@code pose}'s. Bound this with {@code Timeouts.limit}.
     */
    public CommandBuilder hold(Pose pose) {
        return owned(Command.build()
                .setStart(() -> follower.holdPoint(new BezierPoint(pose), pose.getHeading(), false))
                .setDone(() -> follower.getPose().distanceFrom(pose) < POSITION_TOLERANCE_IN
                        && headingError(pose.getHeading()) < HEADING_TOLERANCE));
    }

    private double headingError(double target) {
        return Math.abs(MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), target));
    }

    /**
     * Wraps a follower command so it requires the drivetrain and stops the follower when it is
     * cut short. On a natural end the follower is left alone, so {@code automaticHoldEnd}
     * still holds the final pose.
     *
     * <p>A wrapper rather than {@code setEnd()} on the inner command, because the stop must
     * be cancellable: when one path preempts another, the Scheduler ends the old command
     * <em>before</em> starting the new one in the same cycle, and the new {@code start()}
     * clears the request so {@link #write()} does not break the path that just began.
     */
    private CommandBuilder owned(Command inner) {
        return Command.build()
                .setStart(() -> {
                    breakRequested = false;
                    inner.start();
                })
                .setExecute(inner::execute)
                .setDone(inner::done)
                .setEnd(endCondition -> {
                    inner.end(endCondition);
                    if (endCondition != EndCondition.NATURALLY) breakRequested = true;
                })
                .requiring(this);
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
