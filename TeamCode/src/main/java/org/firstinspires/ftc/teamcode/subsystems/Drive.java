package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.teamcode.util.AimController;
import org.firstinspires.ftc.teamcode.util.Subsystem;

import java.util.function.DoubleSupplier;

/**
 * The drivetrain, as a requirable token wrapped around Pedro's {@link Follower}.
 *
 * <p>Every command here declares {@code .requiring(this)}, so two drive commands can never run
 * at once — the scheduler resolves the conflict instead (architecture doc §5.3). Pedro 3's
 * {@code Follower} is driven through four mode changes — {@code follow}, {@code hold},
 * {@code manual}, {@code stop} — none of which touches hardware: the motors are written inside
 * {@code follower.update()}, which the {@code Robot} runs in phase 3. So these commands can set
 * the follower's mode directly from {@code start()}/{@code end()} without breaking the
 * hardware-only-in-read/write rule. (Ivy's {@code PedroCommands} are compiled against Pedro 2
 * and are not used.)
 *
 * <p>Two notes on timing:
 *
 * <ul>
 *   <li>The pose is refreshed by {@code follower.update()} in phase 3, so {@link #pose()} is a
 *       live read rather than something cached in {@link #read()}; a command in phase 4 sees a
 *       pose from phase 3 of the <em>same</em> iteration.</li>
 *   <li>A mode set in phase 4 drives the motors on the next {@code follower.update()} — one
 *       loop period, ~6 ms, the latency the whole design budgets for.</li>
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
            // Re-asserted every cycle rather than once in a command's start(): the Scheduler
            // resumes a SUSPENDed command without calling start() again, and a path command
            // that ran in between will have taken the follower out of manual mode.
            follower.manual(robotCentric
                    ? new DrivePowers(forward, strafe, turn)
                    : ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
        } else if (follower.manual()) {
            // Driver control ended and nothing else took the drive: stop, so the robot does not
            // coast on a stale drive vector. A path started this cycle has already left manual
            // mode, so this never cuts one off.
            follower.stop();
        }
    }

    @Override
    public void telemetry(TelemetryManager t) {
        Pose pose = follower.pose();
        t.addLine(String.format("drive: x %.1f y %.1f h %.2frad %s%s",
                pose.x(), pose.y(), pose.heading(), follower.mode(),
                localizationLost() ? "  LOCALIZATION LOST" : ""));
    }

    @Override
    public void stop() {
        teleopEnabled = false;
        follower.stop();
        // The OpMode is exiting, so there is no next follower.update() to apply the stop.
        follower.drivetrain.stop();
    }

    // --- queries: safe in phase 4 ---

    /** Refreshed by {@code follower.update()} in phase 3 of this same iteration. */
    public Pose pose() {
        return follower.pose();
    }

    /** Parametric progress along the current path segment, 0–1. See {@link #waitForT(double)}. */
    public double t() {
        return follower.parametricCompletion();
    }

    /** The pose is NaN: no path can be followed. A routine should abort on this (§8.4). */
    public boolean localizationLost() {
        Pose p = follower.pose();
        return Double.isNaN(p.x()) || Double.isNaN(p.y()) || Double.isNaN(p.heading());
    }

    // --- commands ---

    /**
     * Follows {@code path}, finishing when Foresight reports it is no longer busy — at the end
     * and settled, or after its end timeout. The follower then holds the end pose until
     * something else takes the drive. Bound this with {@code Timeouts.limit}.
     */
    public CommandBuilder follow(Path path) {
        return owned(Command.build()
                .setStart(() -> follower.follow(path))
                .setDone(() -> !follower.isBusy()));
    }

    /**
     * Turns in place, finishing when the heading is within {@link #HEADING_TOLERANCE}. The
     * follower keeps holding the heading afterwards until something else takes the drive.
     * Bound this with {@code Timeouts.limit}: it does not finish if the robot cannot turn.
     *
     * @param radians heading in radians — the unit convention at every API boundary.
     */
    public CommandBuilder turnTo(double radians) {
        return owned(Command.build()
                .setStart(() -> follower.hold(follower.pose().withHeading(radians)))
                .setDone(() -> headingError(radians) < HEADING_TOLERANCE));
    }

    /**
     * Drives to and holds {@code pose}, finishing once inside {@link #POSITION_TOLERANCE_IN}
     * and {@link #HEADING_TOLERANCE}; the hold continues after that. Bound this with
     * {@code Timeouts.limit}.
     */
    public CommandBuilder hold(Pose pose) {
        return owned(Command.build()
                .setStart(() -> follower.hold(pose))
                .setDone(() -> follower.pose().distance(pose) < POSITION_TOLERANCE_IN
                        && headingError(pose.heading()) < HEADING_TOLERANCE));
    }

    private double headingError(double target) {
        return Math.abs(Angle.normalizeSigned(target - follower.pose().heading()));
    }

    /**
     * Wraps a follower command so it requires the drivetrain and stops the follower when it is
     * cut short — by a timeout, a race, or a {@code cancel()}. On a natural end the follower is
     * left alone, so it keeps holding the final pose.
     *
     * <p>Stopping in {@code end()} is safe under preemption: the Scheduler ends the old command
     * <em>before</em> starting the new one in the same cycle, so the new command's mode wins.
     */
    private CommandBuilder owned(Command inner) {
        return Command.build()
                .setStart(inner::start)
                .setExecute(inner::execute)
                .setDone(inner::done)
                .setEnd(endCondition -> {
                    inner.end(endCondition);
                    if (endCondition != EndCondition.NATURALLY) {
                        follower.stop();
                    }
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
        return Commands.waitUntil(() -> follower.following() && follower.parametricCompletion() >= t);
    }

    /**
     * Driver control with the turn taken over by aiming: forward and strafe stay with the
     * driver, rotation servos {@code aimError} to zero through {@link AimController}. While
     * {@code aimError} is NaN (no target in view) the driver's {@code manualTurn} applies, so
     * losing the target never locks the driver out.
     *
     * <p>Robot-centric, and needs no odometry — it closes the loop on the vision measurement
     * itself. Same requirement and {@code SUSPEND} behaviour as {@link #teleopDrive}, so
     * scheduling it suspends plain driver control and cancelling it resumes that.
     *
     * @param aimError radians to turn, CCW positive, NaN when there is nothing to aim at
     */
    public CommandBuilder aimedTeleopDrive(DoubleSupplier forward, DoubleSupplier strafe,
                                           DoubleSupplier manualTurn, DoubleSupplier aimError) {
        return teleopDrive(forward, strafe, () -> {
            double turn = AimController.turn(aimError.getAsDouble());
            return Double.isNaN(turn) ? manualTurn.getAsDouble() : turn;
        }, true);
    }

    /**
     * Turns in place to servo {@code aimError} to zero, and keeps holding it there; with no
     * target in view it sits still. The autonomous form of {@link #aimedTeleopDrive} — and like
     * it, needs no odometry. Never finishes: bound it, or put it under a {@code deadline}.
     */
    public CommandBuilder aimInPlace(DoubleSupplier aimError) {
        return aimedTeleopDrive(() -> 0, () -> 0, () -> 0, aimError);
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
     * <p>Pedro's convention: forward +, strafe + is <b>left</b>, turn + is counter-clockwise.
     * Suppliers rather than values: this command is built once, before the match, and reads
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
