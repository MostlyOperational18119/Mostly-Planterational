package org.firstinspires.ftc.teamcode.routines;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.TestPaths;
import org.firstinspires.ftc.teamcode.util.Timeouts;

/**
 * Routines for exercising the architecture rather than scoring: pure trees, no hardware, no
 * state (architecture doc §8.2).
 */
public final class TestRoutines {

    /** A 24 in leg at Pedro's default speed takes ~1–2 s; this only catches a hang. */
    private static final double LEG_TIMEOUT_MS = 5000;

    /** Path progress at which the intake starts on the way out. */
    private static final double INTAKE_FROM_T = 0.3;

    private static final double EJECT_MS = 500;

    private TestRoutines() {}

    /**
     * Out with the intake starting partway along, back while ejecting briefly. Each leg is a
     * {@code deadline} on its path, so the mechanism work overlaps the drive and costs no time
     * of its own — the property step 5 of the implementation order (§10) exists to check.
     *
     * <p>The outer {@code sequential} is deliberate (rule 5): the return leg cannot start
     * until the robot has arrived.
     */
    public static Command outAndBack(Robot r, TestPaths p) {
        return Groups.sequential(
                Groups.deadline(
                        Timeouts.limit(r.drive.follow(p.out), LEG_TIMEOUT_MS),
                        Groups.sequential(r.drive.waitForT(INTAKE_FROM_T), r.intake.collect())
                ),
                Groups.deadline(
                        Timeouts.limit(r.drive.follow(p.back), LEG_TIMEOUT_MS),
                        Timeouts.limit(r.intake.eject(), EJECT_MS)
                )
        );
    }
}
