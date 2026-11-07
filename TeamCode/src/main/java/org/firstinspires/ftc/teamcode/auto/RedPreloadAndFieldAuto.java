package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Timeouts;
import org.firstinspires.ftc.teamcode.vision.CellSighting;

/**
 * Red alliance: shoots the preload into the red HIVE's target CELL if vision sees it facing up,
 * then turns 90°, drives to the end pose collecting four pollen, returns to the start and
 * shoots them the same way, and parks in the red LOADING ZONE whether it shot or not.
 *
 * <p>Starts where {@link RedPreloadAudienceAuto} does: back against the audience wall, centred
 * on the third TILE from the red wall, facing the HIVE. So the target is the AUDIENCE CELL. The
 * poses are the {@code PRELOAD_AND_FIELD_*} settings in {@link AutoPaths}. Not yet driven on a
 * field; driving needs Pedro (a localizer and Foresight).
 */
// @Autonomous(name = "Red: Preload + Field", group = "BIOBUZZ")
public class RedPreloadAndFieldAuto extends LinearOpMode {

    private static final Alliance ALLIANCE = Alliance.RED;
    private static final Cell TARGET = Cell.RED_AUDIENCE;
    private static final double AUTO_DEADLINE_MS = 28_000;

    private long startNanos, endNanos;
    /** The latest shot's decision, and the reading behind it; null until the first. */
    private String decision;

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        if (!Constants.canFollowPaths()) {
            // It has to drive to collect; on StubLocalizer, or before AutoTune, it cannot.
            // Refuse rather than start a routine that cannot finish.
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
        AutoPaths paths = new AutoPaths(AutoPaths.Side.AUDIENCE);
        robot.follower.setPose(AutoPaths.START);

        while (opModeInInit()) {
            robot.initCycle();
        }

        // Failure containment (§8.4): ends on finishing, the deadline, or losing localization.
        Command routine = Groups.race(
                ShootingRoutines.preloadThenFieldCycle(robot, ALLIANCE, TARGET,
                        AutoPaths.START, paths.fieldTurned, paths.toFieldEnd, paths.toFieldStart,
                        paths.approachPark, paths.toPark,
                        shoot -> decision = String.format(
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
        return "target: " + TARGET + " | " + now + " | routine: " + routine;
    }

    /**
     * What vision makes of the target right now, with the numbers behind it: the CELL is told
     * up or down by its measured height, so a reading just under the threshold is the first
     * thing to look for when the robot does not shoot at an up CELL.
     */
    private String reading(Robot robot) {
        if (!robot.vision.hasFix()) return "vision " + robot.vision.status() + " — will not shoot";
        Cell up = robot.vision.upCell(ALLIANCE);
        CellSighting seen = robot.vision.sighting(TARGET);
        String height = seen == null ? "target not in view"
                : String.format("target %.1f in high, %.1f in away (up ≥ %.1f)", seen.z,
                        seen.floorDistance(), Vision.UP_CELL_MIN_HEIGHT_IN);
        String verdict = up == null ? "no " + ALLIANCE + " CELL seen — will not shoot"
                : up == TARGET ? "UP — will shoot" : "DOWN — will not shoot";
        return verdict + "; " + height;
    }
}
