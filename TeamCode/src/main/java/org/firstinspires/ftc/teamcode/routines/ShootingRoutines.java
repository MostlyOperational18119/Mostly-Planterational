package org.firstinspires.ftc.teamcode.routines;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.util.AimController;
import org.firstinspires.ftc.teamcode.util.Timeouts;

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

    private ShootingRoutines() {}

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
                                AimController.onTarget(aimError.getAsDouble())
                                        && r.outtake.readyToFeed()), AIM_TIMEOUT_MS),
                        Timeouts.limit(r.transfer.feed(), FEED_MS)
                ),
                r.outtake.spinFor(distanceIn),
                r.drive.aimInPlace(aimError)
        );
    }
}
