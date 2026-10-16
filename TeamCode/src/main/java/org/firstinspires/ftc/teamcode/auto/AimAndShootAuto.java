package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;
import org.firstinspires.ftc.teamcode.util.AimController;
import org.firstinspires.ftc.teamcode.util.Timeouts;

/**
 * Step 6 of the implementation order (architecture doc §10), as far as BIOBUZZ has it: aim
 * the chassis at the up CELL using vision alone, then shoot the preload. Drives no paths, so
 * unlike {@link TestAuto} it runs without a localizer.
 *
 * <p>Choose the alliance during init (X blue, B red); started without one, it does nothing.
 * Place the robot with the alliance's HIVE in the Limelight's view.
 */
@Autonomous(name = "Test: Aim and Shoot", group = "BIOBUZZ Test")
public class AimAndShootAuto extends LinearOpMode {

    private static final double AUTO_DEADLINE_MS = 28_000;

    private Alliance alliance;
    private long startNanos, endNanos;

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        Robot robot = Robot.fromHardwareMap(hardwareMap)
                .withTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());
        robot.withStatus(() -> status(robot));
        robot.follower.setStartingPose(new Pose());

        while (opModeInInit()) {
            if (gamepad1.x) alliance = Alliance.BLUE;
            if (gamepad1.b) alliance = Alliance.RED;
            robot.initCycle();
        }

        // Built after init because it depends on the alliance; aimAndShoot builds no paths, so
        // this costs nothing measurable.
        Command routine = alliance == null ? null : Groups.race(
                ShootingRoutines.aimAndShoot(robot, alliance),
                Timeouts.command(AUTO_DEADLINE_MS));

        robot.follower.update();
        if (routine != null) routine.schedule();
        startNanos = System.nanoTime();

        while (opModeIsActive()) {
            robot.cycle();
            if (routine != null && endNanos == 0 && !routine.isScheduled()) {
                endNanos = System.nanoTime();
            }
        }

        Scheduler.reset();
        robot.stop();
    }

    private String status(Robot robot) {
        if (alliance == null) return "alliance: NOT SET (X blue, B red) — will not run";
        double error = robot.aimError(alliance);
        String aim = Double.isNaN(error) ? "no target"
                : AimController.onTarget(error) ? "ON TARGET"
                : String.format("turn %+.1f°", Math.toDegrees(error));
        String routine = startNanos == 0 ? "waiting for start"
                : endNanos == 0 ? String.format("running %.2fs", (System.nanoTime() - startNanos) / 1e9)
                : String.format("done in %.2fs", (endNanos - startNanos) / 1e9);
        return "alliance: " + alliance + " | aim: " + aim + " | routine: " + routine;
    }
}
