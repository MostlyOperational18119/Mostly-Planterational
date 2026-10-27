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
 * Red alliance: shoots the three preloaded pollen into the red HIVE's target CELL if vision
 * sees it facing up, then parks in the red LOADING ZONE. If the CELL is down, or never seen,
 * it skips the shooting and only parks.
 *
 * <p>Starts as the red rectangular robot of the manual's Fig. 11-1 ({@link AutoPaths}): back
 * against the audience wall, centred on the third TILE from the red wall, facing the HIVE. So
 * the target is the AUDIENCE CELL, the near one from this start. The status line shows what
 * vision reads for it during init, and the decision is the reading at START. Parking needs
 * Pedro (a localizer and Foresight).
 */
@Autonomous(name = "Red: Preload Only", group = "BIOBUZZ")
public class RedPreloadAuto extends LinearOpMode {

    private static final Alliance ALLIANCE = Alliance.RED;
    private static final Cell TARGET = Cell.RED_AUDIENCE;
    private static final double AUTO_DEADLINE_MS = 28_000;

    private long startNanos, endNanos;
    /** What was decided at START, and the reading behind it; null until then. */
    private String decision;

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        if (!Constants.canFollowPaths()) {
            // It has to drive to the LOADING ZONE to park; on StubLocalizer, or before
            // AutoTune, it cannot. Refuse rather than start a routine that cannot finish.
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
        AutoPaths paths = new AutoPaths();
        robot.follower.setPose(AutoPaths.START);

        while (opModeInInit()) {
            robot.initCycle();
        }

        // Failure containment (§8.4): ends on finishing, the deadline, or losing localization.
        Command routine = Groups.race(
                ShootingRoutines.shootIfUpThenPark(robot, ALLIANCE, TARGET,
                        paths.approach, paths.toPark, shoot -> decision = String.format(
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
        if (!robot.vision.hasFix()) return "vision " + robot.vision.status() + " — will only park";
        Cell up = robot.vision.upCell(ALLIANCE);
        CellSighting seen = robot.vision.sighting(TARGET);
        String height = seen == null ? "target not in view"
                : String.format("target %.1f in high, %.1f in away (up ≥ %.1f)", seen.z,
                        seen.floorDistance(), Vision.UP_CELL_MIN_HEIGHT_IN);
        String verdict = up == null ? "no " + ALLIANCE + " CELL seen — will only park"
                : up == TARGET ? "UP — will shoot, then park" : "DOWN — will only park";
        return verdict + "; " + height;
    }
}
