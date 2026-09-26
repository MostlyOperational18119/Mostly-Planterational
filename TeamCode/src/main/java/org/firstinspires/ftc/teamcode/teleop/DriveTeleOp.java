package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
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
 */
@TeleOp(name = "Drive (Command)", group = "BIOBUZZ")
public class DriveTeleOp extends LinearOpMode {

    private static final double SNAP_TIMEOUT_MS = 1500;

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        Robot robot = Robot.fromHardwareMap(hardwareMap)
                .withTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());
        robot.follower.setStartingPose(new Pose());

        // Built once, before the match: it reads the gamepad through suppliers each cycle.
        Command driverControl = robot.drive.teleopDrive(
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                true);

        while (opModeInInit()) {
            robot.initCycle();
        }

        robot.follower.update();
        driverControl.schedule();

        while (opModeIsActive()) {
            // Gamepad-triggered scheduling is the TeleOp equivalent of a routine (§8.3).
            if (gamepad1.aWasPressed()) {
                Timeouts.limit(robot.drive.turnTo(0), SNAP_TIMEOUT_MS).schedule();
            }
            robot.cycle();
        }

        Scheduler.reset();
        robot.stop();
    }
}
