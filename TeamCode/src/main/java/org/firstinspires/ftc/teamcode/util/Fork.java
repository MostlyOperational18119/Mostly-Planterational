package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;

/**
 * Fire-and-forget scheduling — the working replacement for {@code Command.proxy()}.
 *
 * <p>Ivy 1.0.0's {@code proxy()} sets {@code done = () -> Scheduler.isScheduled(this)}, which
 * is inverted: the proxy reports done the instant the inner command <em>is</em> scheduled
 * (architecture doc §2.1). Do not use it.
 *
 * <p>Use {@link #of(Command)} sparingly: a forked command escapes its parent group's
 * lifecycle, so an interrupted routine will not clean it up. Prefer {@code parallel} /
 * {@code deadline}.
 */
public final class Fork {

    private Fork() {}

    /** Schedules {@code inner} as an independent citizen and completes in the same cycle. */
    public static CommandBuilder of(Command inner) {
        return Command.build()
                .setStart(inner::schedule)
                .setDone(() -> true);
    }

    /** Schedules {@code inner} and completes once it leaves the scheduler, cancelling it if
     *  this wrapper is interrupted first. */
    public static CommandBuilder untilDone(Command inner) {
        return Command.build()
                .setStart(inner::schedule)
                .setDone(() -> !Scheduler.isScheduled(inner))
                .setEnd(endCondition -> {
                    if (endCondition != EndCondition.NATURALLY) inner.cancel();
                });
    }
}
