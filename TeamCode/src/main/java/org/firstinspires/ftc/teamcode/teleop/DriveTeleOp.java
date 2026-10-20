package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;
import org.firstinspires.ftc.teamcode.util.AimController;
import org.firstinspires.ftc.teamcode.util.RangeController;
import org.firstinspires.ftc.teamcode.util.Timeouts;

/**
 * Driver control on the loop contract — and, for now, the loop-budget test fixture.
 *
 * <p>With only the drivetrain wired up, the number to watch is the one in telemetry: the loop
 * must hold <b>&le; 6 ms median</b> with nothing else running before any mechanism is added to
 * it (architecture doc §10, step 1). If it does not hold here, it will not hold later.
 *
 * <p>The A button demonstrates the part that matters architecturally: {@code turnTo} and
 * driver control both require {@link org.firstinspires.ftc.teamcode.subsystems.Drive}, so the
 * scheduler interrupts driver control for the duration of the snap and resumes it afterwards
 * — no mode flags, no state machine.
 *
 * <p>Intake: hold right bumper to collect, left bumper to eject (eject wins if both are held).
 *
 * <p>Shoot: <b>hold Y</b>. The robot turns onto your alliance's up CELL, drives forward or back
 * to the range the shot scores from ({@code RangeController.SHOT_DISTANCE_IN}), spins the
 * flywheel, and feeds once aimed, in range and spun up ({@link ShootingRoutines#teleopShoot}).
 * The left stick's sideways axis still strafes. Release Y to stop and take the robot back.
 * With the CELL out of view, the sticks drive as normal and nothing feeds.
 *
 * <p>Manual launch, for when vision cannot be used (no alliance chosen, Limelight down, CELL
 * hidden): X toggles the flywheel and the transfer together — no aiming, no range, no waiting.
 *
 * <p>Aim only: hold the right trigger to turn onto the up CELL while the left stick still
 * translates; with no target in view the right stick turns as normal.
 *
 * <p>Choose the alliance during init (X blue, B red); until one is chosen, Y and the right
 * trigger do nothing.
 */
@TeleOp(name = "Drive (Command)", group = "BIOBUZZ")
public class DriveTeleOp extends LinearOpMode {

    private static final double SNAP_TIMEOUT_MS = 1500;

    private Alliance alliance;

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        Robot robot = Robot.fromHardwareMap(hardwareMap)
                .withTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());
        robot.withStatus(() -> status(robot));
        robot.follower.setPose(Pose.zero());

        // Built once, before the match: it reads the gamepad through suppliers each cycle.
        Command driverControl = robot.drive.teleopDrive(
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                true);
        Command collect = robot.intake.collect();
        Command eject = robot.intake.eject();
        // One command over both requirements: toggling it starts and stops them together.
        Command manualLaunch = Groups.parallel(
                robot.outtake.spinFor(() -> shotDistance(robot)), robot.transfer.feed());
        Command aim = robot.drive.aimedTeleopDrive(
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> aimError(robot));

        while (opModeInInit()) {
            // xWasPressed, not x: it consumes the press, which would otherwise still be latched
            // when the loop's first xWasPressed() runs and start a manual launch at START.
            if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
            if (gamepad1.b) alliance = Alliance.RED;
            robot.initCycle();
        }

        // Built after init because it depends on the alliance; builds no paths, so it costs
        // nothing measurable.
        Command shoot = alliance == null ? null : ShootingRoutines.teleopShoot(robot, alliance,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x);

        robot.follower.update();
        driverControl.schedule();

        while (opModeIsActive()) {
            // Gamepad-triggered scheduling is the TeleOp equivalent of a routine (§8.3).
            // Needs Foresight to hold a heading; before AutoTune the snap would just freeze the
            // drive for SNAP_TIMEOUT_MS.
            if (gamepad1.aWasPressed() && Constants.canFollowPaths()) {
                Timeouts.limit(robot.drive.turnTo(0), SNAP_TIMEOUT_MS).schedule();
            }
            // Eject first: on its release it cancels before collect reschedules, so letting go
            // of eject while still holding collect drops straight back into collecting.
            whileHeld(eject, gamepad1.left_bumper);
            whileHeld(collect, gamepad1.right_bumper && !gamepad1.left_bumper);
            if (gamepad1.xWasPressed()) toggle(manualLaunch);
            // Both require the drive, like driverControl: holding suspends it, releasing
            // resumes it. Shoot also takes the flywheel and transfer from a manual launch.
            whileHeld(aim, gamepad1.right_trigger > 0.5);
            if (shoot != null) whileHeld(shoot, gamepad1.y);
            robot.cycle();
        }

        Scheduler.reset();
        robot.stop();
    }

    private double aimError(Robot robot) {
        return alliance == null ? Double.NaN : robot.aimError(alliance);
    }

    private double shotDistance(Robot robot) {
        return alliance == null ? Double.NaN : robot.shotDistance(alliance);
    }

    private String status(Robot robot) {
        if (alliance == null) return "alliance: NOT SET (X blue, B red) — aiming disabled";
        double error = aimError(robot);
        String aim = Double.isNaN(error) ? "no target"
                : AimController.onTarget(error) ? "ON TARGET"
                : String.format("turn %+.1f°", Math.toDegrees(error));
        double distance = robot.vision.targetDistance(alliance);
        String range = Double.isNaN(distance) ? "not seen"
                : String.format("%.1fin (shoot at %.0f±%.0f)%s", distance,
                        RangeController.SHOT_DISTANCE_IN, RangeController.TOLERANCE_IN,
                        RangeController.inRange(distance) ? " IN RANGE" : "");
        return "alliance: " + alliance + " | aim: " + aim + " | range: " + range
                + "\nshoot (Y): " + shootBlockers(robot, error, distance);
    }

    /** What hold-Y is waiting on before it feeds — {@link ShootingRoutines#readyToShoot}'s
     *  three conditions, each named when it fails. */
    private static String shootBlockers(Robot robot, double aimError, double distance) {
        StringBuilder waiting = new StringBuilder();
        if (!ShootingRoutines.aimedToShoot(aimError)) {
            waiting.append(Double.isNaN(aimError) ? " aim (no target)" : String.format(
                    " aim (%.1f° off, need <%.1f°)", Math.abs(Math.toDegrees(aimError)),
                    ShootingRoutines.SHOOT_AIM_TOLERANCE_DEG));
        }
        if (!RangeController.inRange(distance)) {
            waiting.append(Double.isNaN(distance) ? " range (no target)" : String.format(
                    " range (%.1fin, need %.0f±%.0f)", distance,
                    RangeController.SHOT_DISTANCE_IN, RangeController.TOLERANCE_IN));
        }
        if (!robot.outtake.readyToFeed()) {
            waiting.append(robot.outtake.running() ? " flywheel (spinning up)" : " flywheel (off)");
        }
        return waiting.length() == 0 ? "READY — feeding" : "waiting on" + waiting;
    }

    /** Edge-triggered binding: each press flips {@code command} between scheduled and not. */
    private static void toggle(Command command) {
        if (command.isScheduled()) command.cancel();
        else command.schedule();
    }

    /** Level-triggered binding: {@code command} is scheduled exactly while {@code held}. */
    private static void whileHeld(Command command, boolean held) {
        if (held && !command.isScheduled()) command.schedule();
        else if (!held && command.isScheduled()) command.cancel();
    }
}
