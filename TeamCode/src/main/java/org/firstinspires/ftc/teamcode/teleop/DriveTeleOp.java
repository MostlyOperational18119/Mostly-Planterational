package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.util.AimController;
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
 * Launch: Y toggles the flywheel and the transfer together, so the transfer carries pollen
 * straight up into the spinning flywheel.
 *
 * <p>Aim: hold the right trigger to turn the robot onto your alliance's up CELL — there is no
 * turret, so the chassis aims the flywheel. The left stick still translates; with no target in
 * view the right stick turns as normal. Choose the alliance during init (X blue, B red); until
 * one is chosen, aiming does nothing.
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
        robot.follower.setStartingPose(new Pose());

        // Built once, before the match: it reads the gamepad through suppliers each cycle.
        Command driverControl = robot.drive.teleopDrive(
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                true);
        Command collect = robot.intake.collect();
        Command eject = robot.intake.eject();
        // One command over both requirements: toggling it starts and stops them together.
        Command launch = Groups.parallel(robot.outtake.spin(), robot.transfer.feed());
        Command aim = robot.drive.aimedTeleopDrive(
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> aimError(robot));

        while (opModeInInit()) {
            if (gamepad1.x) alliance = Alliance.BLUE;
            if (gamepad1.b) alliance = Alliance.RED;
            robot.initCycle();
        }

        robot.follower.update();
        driverControl.schedule();

        while (opModeIsActive()) {
            // Gamepad-triggered scheduling is the TeleOp equivalent of a routine (§8.3).
            if (gamepad1.aWasPressed()) {
                Timeouts.limit(robot.drive.turnTo(0), SNAP_TIMEOUT_MS).schedule();
            }
            // Eject first: on its release it cancels before collect reschedules, so letting go
            // of eject while still holding collect drops straight back into collecting.
            whileHeld(eject, gamepad1.left_bumper);
            whileHeld(collect, gamepad1.right_bumper && !gamepad1.left_bumper);
            if (gamepad1.yWasPressed()) toggle(launch);
            // Same requirement as driverControl: holding suspends it, releasing resumes it.
            whileHeld(aim, gamepad1.right_trigger > 0.5);
            robot.cycle();
        }

        Scheduler.reset();
        robot.stop();
    }

    private double aimError(Robot robot) {
        return alliance == null ? Double.NaN : robot.vision.aimError(alliance);
    }

    private String status(Robot robot) {
        if (alliance == null) return "alliance: NOT SET (X blue, B red) — aiming disabled";
        double error = aimError(robot);
        String aim = Double.isNaN(error) ? "no target"
                : AimController.onTarget(error) ? "ON TARGET"
                : String.format("turn %+.1f°", Math.toDegrees(error));
        return "alliance: " + alliance + " | aim: " + aim;
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
