package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Timeouts;
import org.firstinspires.ftc.teamcode.vision.CellSighting;

import java.util.function.Consumer;

/**
 * What every preload auto shares: refuse to run without Pedro, build the robot, wait out init,
 * run one routine inside the failure-containment race (§8.4), and report on the status line
 * what vision reads for the target and how the routine is going.
 *
 * <p>A subclass names its alliance and target CELL and builds its routine; everything else is
 * here. Build {@link AutoPaths} in {@link #onInit} (rule 6: never mid-routine).
 */
public abstract class AutoBase extends LinearOpMode {

    protected static final double AUTO_DEADLINE_MS = 28_000;

    private long startNanos, endNanos;
    /** What was decided, and the reading behind it; null until then. */
    private String decision;

    protected abstract Alliance alliance();

    protected abstract Cell target();

    /** Called once the robot exists, before init; build the {@link AutoPaths} this auto needs. */
    protected abstract void onInit(Robot robot);

    /** Called every init cycle, before {@code robot.initCycle()}. Nothing by default. */
    protected void onInitCycle() {}

    /**
     * The routine to run, once, at START. {@code onDecided} takes whether it will shoot; the
     * status line then shows the decision and the reading behind it.
     */
    protected abstract Command buildRoutine(Robot robot, Consumer<Boolean> onDecided);

    /** A line above the status, or null. {@link #started()} says whether the match began. */
    protected String header() {
        return null;
    }

    protected final boolean started() {
        return startNanos != 0;
    }

    @Override
    public final void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        if (!Constants.canFollowPaths()) {
            // Every auto drives, to park at least; on StubLocalizer, or before AutoTune, it
            // cannot. Refuse rather than start a routine that cannot finish.
            telemetry.addLine(!Constants.LOCALIZER_INSTALLED
                    ? "Disabled: no localizer installed (Constants.LOCALIZER_INSTALLED)."
                    : "Disabled: Foresight not tuned yet — run AutoTune (Constants.FORESIGHT_TUNED).");
            telemetry.update();
            waitForStart();
            return;
        }

        Robot robot = Robot.fromHardwareMap(hardwareMap)
                .withTelemetry(PanelsTelemetry.INSTANCE.getTelemetry(), telemetry);
        robot.withStatus(() -> status(robot));
        onInit(robot);
        robot.follower.setPose(AutoPaths.START);

        while (opModeInInit()) {
            onInitCycle();
            robot.initCycle();
        }

        // Failure containment (§8.4): ends on finishing, the deadline, or losing localization.
        Command routine = Groups.race(
                buildRoutine(robot, shoot -> decision = String.format(
                        "%s at %.1fs (%s)", shoot ? "SHOOTING" : "NOT SHOOTING",
                        (System.nanoTime() - startNanos) / 1e9, reading(robot))),
                Timeouts.command(AUTO_DEADLINE_MS),
                Commands.waitUntil(robot.drive::localizationLost));

        robot.follower.update();
        routine.schedule();
        startNanos = System.nanoTime();

        while (opModeIsActive()) {
            robot.cycle();
            if (endNanos == 0 && !routine.isScheduled()) endNanos = System.nanoTime();
        }

        Scheduler.reset();
        robot.stop();
    }

    private String status(Robot robot) {
        String routine = startNanos == 0 ? "waiting for start"
                : endNanos == 0 ? String.format("running %.2fs", (System.nanoTime() - startNanos) / 1e9)
                : String.format("done in %.2fs", (endNanos - startNanos) / 1e9);
        String now = decision != null ? "decided: " + decision : reading(robot);
        String header = header();
        return (header == null ? "" : header + "\n")
                + "target: " + target() + " | " + now + " | routine: " + routine;
    }

    /**
     * What vision makes of the target right now, with the numbers behind it: the CELL is told
     * up or down by its measured height, so a reading just under the threshold is the first
     * thing to look for when the robot does not shoot at an up CELL. "Would", because a
     * scoring-side routine decides only once it has driven to the HIVE.
     */
    private String reading(Robot robot) {
        if (!robot.vision.hasFix()) return "vision " + robot.vision.status() + " — would only park";
        Cell up = robot.vision.upCell(alliance());
        CellSighting seen = robot.vision.sighting(target());
        String height = seen == null ? "target not in view"
                : String.format("target %.1f in high, %.1f in away (up ≥ %.1f)", seen.z,
                        seen.floorDistance(), Vision.UP_CELL_MIN_HEIGHT_IN);
        String verdict = up == null ? "no " + alliance() + " CELL seen — would only park"
                : up == target() ? "UP — would shoot, then park" : "DOWN — would only park";
        return verdict + "; " + height;
    }
}
