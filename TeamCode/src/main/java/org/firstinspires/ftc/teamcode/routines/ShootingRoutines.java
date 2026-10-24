package org.firstinspires.ftc.teamcode.routines;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.RangeController;
import org.firstinspires.ftc.teamcode.util.Timeouts;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Scoring from where the robot stands: pure trees, no hardware, no state (architecture doc
 * §8.2). Built at init; the timings are read when the routine is built, so set them in Panels
 * before pressing INIT.
 */
@Configurable
public final class ShootingRoutines {

    /**
     * How long to wait for the aim and the flywheel before feeding anyway. <b>Placeholder</b> —
     * tune once the aim gains are.
     */
    public static double AIM_TIMEOUT_MS = 2000;

    /**
     * How long the transfer feeds — must cover every preloaded pollen. <b>Placeholder</b>:
     * measure how long one takes to go through, times the preload count.
     */
    public static double FEED_MS = 1500;

    /**
     * Aimed enough to feed, degrees. Deliberately wider than {@code AimController.TOLERANCE},
     * the band where the aim stops correcting: with the two equal, the robot could come to rest
     * just outside it (too little turn power left to move it) or hunt across its edge, and
     * never feed until Y was pressed again. Shots score up to ±12 in sideways of the CELL,
     * so this can be loose; at 42 in, 3° is ~2 in at the target.
     */
    public static double SHOOT_AIM_TOLERANCE_DEG = 3;

    /** How long {@link #shootIfUpThenPark} gives the move round the HIVE after shooting. */
    public static double APPROACH_TIMEOUT_MS = 3000;

    /** How long {@link #shootIfUpThenPark} gives the drive into the LOADING ZONE. */
    public static double PARK_TIMEOUT_MS = 4000;

    /**
     * How long {@link #shootIfUp} watches for its target CELL to read up before giving up on
     * the shot. Ends at once when it does; a down or unseen CELL costs all of it.
     */
    public static double DECIDE_TIMEOUT_MS = 1000;

    /**
     * How long the transfer and intake feed the preload — must cover all three pollen.
     * <b>Placeholder</b>: measure one going through, times three, plus margin. Raised from
     * 3000 to give the intake time to carry the pollen to the transfer.
     */
    public static double PRELOAD_FEED_MS = 5000;

    /**
     * How long {@link #rangeAndShoot} drives and aims before feeding anyway. Longer than
     * {@link #AIM_TIMEOUT_MS}, which only turns: this also closes the range from the start.
     */
    public static double RANGE_TIMEOUT_MS = 4000;

    /**
     * Intake power while an autonomous launch feeds ({@link #aimAndShoot},
     * {@link #rangeAndShoot}). Above {@code Intake.COLLECT_POWER}, which does not kick the
     * pollen up into the transfer there; TeleOp's launches keep the regular power.
     * <b>Placeholder</b> — raise it towards 1 if pollen still stalls.
     */
    public static double AUTO_INTAKE_POWER = 0.8;

    private ShootingRoutines() {}

    /** {@link #feed(Robot, double)} at the regular {@code Intake.COLLECT_POWER}: TeleOp's. */
    public static CommandBuilder feed(Robot r) {
        return feed(r, Intake.COLLECT_POWER);
    }

    /**
     * Feeds the flywheel: the transfer lifts pollen into it while the intake runs forward at
     * {@code intakePower} behind it, until the command ends. The intake has to run — pollen
     * still in the intake only reaches the transfer while it does. Every launch goes through
     * this.
     *
     * <p>Takes the intake's requirement, and a group claims its members' requirements from its
     * start, so a TeleOp must not bind the intake while a launch is scheduled: a bumper would
     * interrupt the whole launch.
     */
    public static CommandBuilder feed(Robot r, double intakePower) {
        return Groups.parallel(r.transfer.feed(), r.intake.run(intakePower));
    }

    /** Within {@link #SHOOT_AIM_TOLERANCE_DEG} of the target; false for NaN (no target). */
    public static boolean aimedToShoot(double aimError) {
        return Math.abs(aimError) < Math.toRadians(SHOOT_AIM_TOLERANCE_DEG);
    }

    /**
     * {@link #aimAndShoot(Robot, DoubleSupplier, DoubleSupplier)} at {@code alliance}'s up CELL,
     * launch speed by its distance.
     */
    public static Command aimAndShoot(Robot r, Alliance alliance) {
        return aimAndShoot(r, () -> r.aimError(alliance), () -> r.shotDistance(alliance));
    }

    /**
     * Turns onto the target, then feeds into the flywheel for {@link #FEED_MS}. The flywheel
     * runs at the speed for {@code distanceIn} ({@code Outtake.spinFor}), re-read every cycle.
     *
     * <p>The flywheel spins up and the aim holds for the <em>whole</em> routine — both are
     * {@code deadline} members — so spin-up overlaps aiming, and the robot keeps correcting
     * while it shoots. Feeding starts once the robot is on target and the flywheel is
     * {@linkplain org.firstinspires.ftc.teamcode.subsystems.Outtake#readyToFeed() ready}, or
     * after {@link #AIM_TIMEOUT_MS} regardless: an auto that never shoots scores nothing,
     * whereas a shot slightly off may still go in.
     *
     * <p>The inner {@code sequential} is deliberate (rule 5): feeding before the aim settles
     * throws pollen at the wrong place.
     *
     * @param aimError radians to turn, CCW positive, NaN with no target
     * @param distanceIn floor distance to the target, inches, NaN when unknown
     */
    public static Command aimAndShoot(Robot r, DoubleSupplier aimError, DoubleSupplier distanceIn) {
        return Groups.deadline(
                Groups.sequential(
                        Timeouts.limit(Commands.waitUntil(() ->
                                aimedToShoot(aimError.getAsDouble())
                                        && r.outtake.readyToFeed()), AIM_TIMEOUT_MS),
                        Timeouts.limit(feed(r, AUTO_INTAKE_POWER), FEED_MS)
                ),
                r.outtake.spinFor(distanceIn),
                r.drive.aimInPlace(aimError)
        );
    }

    /**
     * TeleOp's hold-to-shoot: while scheduled, turns onto {@code alliance}'s up CELL, drives to
     * {@link RangeController#SHOT_DISTANCE_IN} along the line to it, spins the flywheel, and
     * once aimed, in range and spun up, feeds — and keeps feeding, even if the robot is then
     * bumped off target, until it is cancelled. The driver keeps strafe; with the CELL out of
     * view, forward and turn fall back to the driver's sticks, and nothing feeds.
     *
     * <p>Never finishes: bind it level-triggered (scheduled exactly while the button is held).
     * The wait before feeding has no timeout, a deliberate exception to rule 4 — the button is
     * the bound, and in TeleOp a shot that is not lined up is better not taken.
     *
     * <p>Range is closed on the <em>live</em> vision distance, not {@link Robot#shotDistance}'s
     * held one: a held distance would drive the robot on a measurement that no longer changes.
     *
     * @param forward driver's forward, used while the CELL is out of view
     * @param strafe  driver's strafe, always
     * @param turn    driver's turn, used while the CELL is out of view
     */
    public static Command teleopShoot(Robot r, Alliance alliance, DoubleSupplier forward,
                                      DoubleSupplier strafe, DoubleSupplier turn) {
        DoubleSupplier aimError = () -> r.aimError(alliance);
        DoubleSupplier distance = () -> r.vision.targetDistance(alliance);
        return Groups.parallel(
                r.outtake.spinFor(() -> r.shotDistance(alliance)),
                r.drive.aimedTeleopDrive(() -> {
                    double f = RangeController.forward(distance.getAsDouble(),
                            aimError.getAsDouble());
                    return Double.isNaN(f) ? forward.getAsDouble() : f;
                }, strafe, turn, aimError),
                // Sequential, so feeding latches on: once started it is not re-gated.
                Groups.sequential(
                        Commands.waitUntil(() -> readyToShoot(r, alliance)),
                        feed(r))
        );
    }

    /**
     * The preload auto: {@link #shootIfUp}, then park — whether it shot or not, since PARK
     * scores either way. Parking drives to {@code approach} with {@code Drive.hold} (it starts
     * from wherever shooting left the robot, so there is no path to build ahead of time), then
     * follows {@code park}, built at init, into the LOADING ZONE, and holds there.
     *
     * <p>Sequential on purpose (rule 5): the robot cannot leave while it is still aiming and
     * feeding. Each move is bounded ({@link #APPROACH_TIMEOUT_MS}, {@link #PARK_TIMEOUT_MS});
     * if the approach times out, the park path still runs from wherever the robot got to.
     *
     * @param onDecided see {@link #shootIfUp(Robot, Alliance, Cell, Consumer)}
     */
    public static Command shootIfUpThenPark(Robot r, Alliance alliance, Cell target,
                                            Pose approach, Path park,
                                            Consumer<Boolean> onDecided) {
        return Groups.sequential(
                shootIfUp(r, alliance, target, onDecided),
                Timeouts.limit(r.drive.hold(approach), APPROACH_TIMEOUT_MS),
                Timeouts.limit(r.drive.follow(park), PARK_TIMEOUT_MS)
        );
    }

    /** {@link #shootIfUp(Robot, Alliance, Cell, Consumer)}, not reporting the decision. */
    public static Command shootIfUp(Robot r, Alliance alliance, Cell target) {
        return shootIfUp(r, alliance, target, shoot -> {});
    }

    /**
     * The autonomous preload: if {@code target} is {@code alliance}'s up CELL, shoots into it
     * ({@link #rangeAndShoot}); if it is down, or never seen, does nothing and finishes.
     *
     * <p>Shoots if {@code target} reads up on <em>any</em> cycle within
     * {@link #DECIDE_TIMEOUT_MS} — normally the first, since vision has been reading through
     * init. Acting on the first reading alone let one frame that fitted the CELL a little low
     * (it is told apart by height, with a few inches to spare) skip the shot. So a down CELL
     * costs the whole window before the routine moves on, and an up one costs nothing. A CELL
     * never seen is not shot at: firing blind at a HIVE that may be down scores nothing.
     *
     * @param onDecided told once, when the window closes: {@code true} if it will shoot
     */
    public static Command shootIfUp(Robot r, Alliance alliance, Cell target,
                                    Consumer<Boolean> onDecided) {
        // Rewritten every cycle of the wait, so it holds the reading that ended it. Built for
        // one run, like every routine here.
        boolean[] up = {false};
        return Groups.sequential(
                Timeouts.limit(Commands.waitUntil(
                        () -> up[0] = r.vision.upCell(alliance) == target), DECIDE_TIMEOUT_MS),
                Commands.conditional(() -> {
                            onDecided.accept(up[0]);
                            return up[0];
                        },
                        rangeAndShoot(r, alliance, PRELOAD_FEED_MS),
                        Commands.instant(() -> {}))
        );
    }

    /**
     * {@link #teleopShoot} for autonomous: turns onto {@code alliance}'s up CELL and drives to
     * {@link RangeController#SHOT_DISTANCE_IN} on the live vision distance, then feeds for
     * {@code feedMs} once aimed, in range and spun up — or after {@link #RANGE_TIMEOUT_MS}
     * regardless. With the CELL out of view the robot sits still. Finishes when the feed does.
     */
    public static Command rangeAndShoot(Robot r, Alliance alliance, double feedMs) {
        DoubleSupplier aimError = () -> r.aimError(alliance);
        return Groups.deadline(
                Groups.sequential(
                        Timeouts.limit(Commands.waitUntil(() -> readyToShoot(r, alliance)),
                                RANGE_TIMEOUT_MS),
                        Timeouts.limit(feed(r, AUTO_INTAKE_POWER), feedMs)
                ),
                r.outtake.spinFor(() -> r.shotDistance(alliance)),
                r.drive.aimedTeleopDrive(() -> {
                    double f = RangeController.forward(r.vision.targetDistance(alliance),
                            aimError.getAsDouble());
                    return Double.isNaN(f) ? 0 : f;
                }, () -> 0, () -> 0, aimError)
        );
    }

    /** Aimed, in range and the flywheel ready — what {@link #teleopShoot} waits for. */
    public static boolean readyToShoot(Robot r, Alliance alliance) {
        return aimedToShoot(r.aimError(alliance))
                && RangeController.inRange(r.vision.targetDistance(alliance))
                && r.outtake.readyToFeed();
    }
}
