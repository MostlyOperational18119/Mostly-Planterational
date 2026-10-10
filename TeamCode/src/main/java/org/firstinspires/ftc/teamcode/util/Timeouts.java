package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import java.util.function.BooleanSupplier;

/**
 * Bounds on how long a command may take. Rule 4: <b>every wait has a timeout</b> — an auto
 * that hangs scores zero, one that gives up on a sub-goal usually still parks
 * (architecture doc §8.4).
 */
public final class Timeouts {

    private Timeouts() {}

    /**
     * Wraps {@code command} so it is interrupted after {@code milliseconds}. Prefer this to
     * {@link #after(long)}: the inner timer is reset by the race's {@code start()}, so the
     * result is reusable and the clock starts when the command actually starts.
     *
     * <p>The returned command inherits {@code command}'s requirements (Ivy's groups union
     * their children's), so {@code Timeouts.limit(drive.turnTo(0), 1500)} still requires the
     * drivetrain.
     */
    public static CommandBuilder limit(Command command, double milliseconds) {
        return Groups.race(command, Commands.waitMs(milliseconds));
    }

    /** A standalone timeout command, for {@code Groups.race(routine, Timeouts.command(28_000))}. */
    public static CommandBuilder command(double milliseconds) {
        return Commands.waitMs(milliseconds);
    }

    /**
     * A deadline predicate for {@code someCommand.until(Timeouts.after(1500))}.
     *
     * <p>The clock starts on the first poll, not at construction — {@code Commands.waitUntil}
     * has no start hook, so a deadline fixed at build time would already have expired by the
     * time the command was scheduled.
     *
     * <p><b>Single use.</b> The returned supplier latches its deadline on first call, so a new
     * one is needed per scheduling. {@link #limit(Command, double)} has no such trap.
     */
    public static BooleanSupplier after(long milliseconds) {
        final long durationNanos = milliseconds * 1_000_000L;
        return new BooleanSupplier() {
            private long deadline = Long.MIN_VALUE;

            @Override
            public boolean getAsBoolean() {
                long now = System.nanoTime();
                if (deadline == Long.MIN_VALUE) {
                    deadline = now + durationNanos;
                    return false;
                }
                return now >= deadline;
            }
        };
    }
}
