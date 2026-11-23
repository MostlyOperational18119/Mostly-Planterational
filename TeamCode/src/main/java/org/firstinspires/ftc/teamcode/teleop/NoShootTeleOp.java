package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Timeouts;

/**
 * "Intake Test": driving and the intake only, no shooting, for trying out the intake and its
 * two servos. Nothing here runs the flywheel or the transfer.
 *
 * <p>Drive: left stick translates, right stick turns. A snaps the heading back to 0
 * ({@code Drive.turnTo}); it needs Pedro tuned ({@code Constants.canFollowPaths()}), and
 * driver control resumes when the snap ends.
 *
 * <p>Intake motor: hold right bumper to collect, left bumper to eject (eject wins if both are
 * held).
 *
 * <p>Intake servos, one press each ({@link org.firstinspires.ftc.teamcode.subsystems.Intake}'s
 * instant commands). None of them stops the intake motor, so they work while collecting:
 * <ul>
 *   <li>D-pad left: flower ({@code setFlower}) — ramp down, intake servo up.</li>
 *   <li>D-pad right: wall ({@code setWall}) — ramp up, intake servo up.</li>
 *   <li>D-pad up: nectar ({@code setForNectar}).</li>
 *   <li>D-pad down: pollen ({@code setForPollen}).</li>
 * </ul>
 *
 * <p>It still builds the whole robot ({@code Robot.fromHardwareMap}), so the robot
 * configuration must name every device, {@code "transfer"} and {@code "outtake"} included,
 * even though this OpMode never uses them; a missing one stops it at INIT.
 */
@TeleOp(name = "Intake Test", group = "BIOBUZZ")
public class NoShootTeleOp extends LinearOpMode {

    private static final double SNAP_TIMEOUT_MS = 1500;

    private Alliance alliance;

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        Robot robot = Robot.fromHardwareMap(hardwareMap)
                .withTelemetry(PanelsTelemetry.INSTANCE.getTelemetry(), telemetry);
        robot.follower.setPose(Pose.zero());

        // Built once, before the match: it reads the gamepad through suppliers each cycle.
        Command driverControl = robot.drive.teleopDrive(
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                true);
        Command collect = robot.intake.collect();
        Command eject = robot.intake.eject();
        Command setWall = robot.intake.setWall();
        Command setFlower = robot.intake.setFlower();
        Command nectar = robot.intake.setForNectar();
        Command pollen = robot.intake.setForPollen();

        while (opModeInInit()) {
            robot.initCycle();
        }

        robot.follower.update();
        driverControl.schedule();

        while (opModeIsActive()) {
            // Gamepad-triggered scheduling is the TeleOp equivalent of a routine (§8.3).
            // Needs Foresight to hold a heading; before AutoTune the snap would just freeze the
            // drive for SNAP_TIMEOUT_MS.
            if (gamepad1.aWasPressed() && Constants.canFollowPaths()) {
                Timeouts.limit(robot.drive.turnTo(0), SNAP_TIMEOUT_MS).schedule();
            }
            // Flower or wall: where the ramp sits is the driver's call.
            if (gamepad1.dpadRightWasPressed()) {setWall.schedule();}
            if (gamepad1.dpadLeftWasPressed()) {setFlower.schedule();}

            // Pollen or nectar: the driver picks which is being collected.
            if (gamepad1.dpadUpWasPressed()) {nectar.schedule();}
            if (gamepad1.dpadDownWasPressed()) {pollen.schedule();}

            // Eject first: on its release it cancels before collect reschedules, so letting go
            // of eject while still holding collect drops straight back into collecting.
            whileHeld(eject, gamepad1.left_bumper);
            whileHeld(collect, gamepad1.right_bumper && !gamepad1.left_bumper);
            robot.cycle();
        }

        Scheduler.reset();
        robot.stop();
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
