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
import org.firstinspires.ftc.teamcode.util.Timeouts;

/**
 * Red alliance, from the corner: shoots the three preloaded pollen into the red HIVE's target
 * CELL if vision sees it facing up; if it is down (or never seen), does nothing.
 *
 * <p>Which red CELL the corner faces is chosen in init — dpad left AUDIENCE, dpad right FAR —
 * and the status line shows what vision reads for it. At START the robot first drives
 * {@link AutoPaths#leave} out of the corner so the camera can see the HIVE's tags, and only
 * then decides. The move is relative to the start pose, so it needs Pedro (a localizer and
 * Foresight) but no field coordinates.
 */
@Autonomous(name = "Red: Preload Only", group = "BIOBUZZ")
public class RedPreloadAuto extends LinearOpMode {

    private static final Alliance ALLIANCE = Alliance.RED;
    private static final double AUTO_DEADLINE_MS = 28_000;

    private Cell target = Cell.RED_AUDIENCE;
    private long startNanos, endNanos;

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        if (!Constants.canFollowPaths()) {
            // It has to drive out of the corner to see the tags; on StubLocalizer, or before
            // AutoTune, it cannot. Refuse rather than sit in the corner, blind.
            telemetry.addLine(!Constants.LOCALIZER_INSTALLED
                    ? "Disabled: no localizer installed (Constants.LOCALIZER_INSTALLED)."
                    : "Disabled: Foresight not tuned yet — run AutoTune (Constants.FORESIGHT_TUNED).");
            telemetry.update();
            waitForStart();
            return;
        }

        Robot robot = Robot.fromHardwareMap(hardwareMap)
                .withTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());
        robot.withStatus(() -> status(robot));
        AutoPaths paths = new AutoPaths();
        robot.follower.setPose(AutoPaths.START);

        while (opModeInInit()) {
            if (gamepad1.dpad_left) target = Cell.RED_AUDIENCE;
            if (gamepad1.dpad_right) target = Cell.RED_FAR;
            robot.initCycle();
        }

        // Built after init because it depends on the target; the path was built above.
        // Failure containment (§8.4): ends on finishing, the deadline, or losing localization.
        Command routine = Groups.race(
                ShootingRoutines.leaveThenShootIfUp(robot, paths.leave, ALLIANCE, target),
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
        Cell up = robot.vision.upCell(ALLIANCE);
        // Before START the robot is still in the corner, so "not seen" there is expected.
        String reading = up == null ? "not seen" + (startNanos == 0 ? " (yet — decides after leaving)" : "")
                : up == target ? "UP — will shoot" : "DOWN — will stop";
        String routine = startNanos == 0 ? "waiting for start"
                : endNanos == 0 ? String.format("running %.2fs", (System.nanoTime() - startNanos) / 1e9)
                : String.format("done in %.2fs", (endNanos - startNanos) / 1e9);
        return "target: " + target + " (dpad ←/→) | " + reading + " | routine: " + routine;
    }
}
