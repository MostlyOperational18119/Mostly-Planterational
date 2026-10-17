package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.AimController;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Bring-up for the flywheel and the shot table, the procedure in {@link Outtake}'s Javadoc.
 * Drives like {@link DriveTeleOp}, so the robot can be moved between distances.
 *
 * <p>Choose the alliance during init (X blue, B red) — the distance is to its up CELL. Then:
 * <ul>
 *   <li><b>Ticks per rev:</b> BACK zeroes the encoder readout; turn the flywheel one
 *       revolution by hand and read the count.</li>
 *   <li><b>Free speed:</b> hold the left trigger to run the flywheel at power 1; once the rpm
 *       settles, the suggested {@code KV} is shown.</li>
 *   <li><b>Shot points:</b> Y toggles the flywheel at the calibration speed, dpad up/down
 *       changes it by {@link #RPM_STEP}. Hold the right trigger to aim, the right bumper to
 *       collect (left ejects), A to feed a shot. When shots score, X records
 *       {@code {distance, rpm}} — it needs the CELL in view this cycle, not a held distance.</li>
 * </ul>
 * Recorded points are shown sorted, as Java to paste into {@link Outtake#SHOT_POINTS}, and
 * written to the robot log ({@code adb logcat -s RobotCore | grep SHOT_POINT}).
 */
@Configurable
@TeleOp(name = "Calibrate: Flywheel and Shots", group = "BIOBUZZ Test")
public class ShotCalibration extends LinearOpMode {

    /** Where the calibration speed starts. */
    public static double START_RPM = 3000;

    /** Dpad up/down step. */
    public static double RPM_STEP = 50;

    private Alliance alliance;
    private double calibrationRpm;
    private int encoderZero;
    private final List<double[]> points = new ArrayList<>();

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        Robot robot = Robot.fromHardwareMap(hardwareMap)
                .withTelemetry(PanelsTelemetry.INSTANCE.getTelemetry());
        robot.withStatus(() -> status(robot));
        robot.follower.setPose(Pose.zero());
        calibrationRpm = START_RPM;

        Command driverControl = robot.drive.teleopDrive(
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                true);
        Command aim = robot.drive.aimedTeleopDrive(
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> alliance == null ? Double.NaN : robot.aimError(alliance));
        Command spin = robot.outtake.spinAt(() -> calibrationRpm);
        Command fullPower = robot.outtake.run(1.0);
        Command collect = robot.intake.collect();
        Command eject = robot.intake.eject();
        Command feed = robot.transfer.feed();

        while (opModeInInit()) {
            if (gamepad1.x) alliance = Alliance.BLUE;
            if (gamepad1.b) alliance = Alliance.RED;
            robot.initCycle();
        }

        robot.follower.update();
        driverControl.schedule();

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) calibrationRpm += RPM_STEP;
            if (gamepad1.dpadDownWasPressed()) calibrationRpm = Math.max(0, calibrationRpm - RPM_STEP);
            if (gamepad1.backWasPressed()) encoderZero = robot.outtake.position();
            if (gamepad1.xWasPressed()) record(robot);
            if (gamepad1.yWasPressed()) {
                if (spin.isScheduled()) spin.cancel();
                else spin.schedule();
            }
            // Same requirement as spin: holding interrupts it, so Y has to be pressed again.
            whileHeld(fullPower, gamepad1.left_trigger > 0.5);
            whileHeld(eject, gamepad1.left_bumper);
            whileHeld(collect, gamepad1.right_bumper && !gamepad1.left_bumper);
            whileHeld(feed, gamepad1.a);
            whileHeld(aim, gamepad1.right_trigger > 0.5);
            robot.cycle();
        }

        Scheduler.reset();
        robot.stop();
    }

    /** Records this cycle's live distance with the calibration speed; replaces a point at the
     *  same distance to the tenth of an inch. */
    private void record(Robot robot) {
        double distance = alliance == null ? Double.NaN : robot.vision.targetDistance(alliance);
        if (Double.isNaN(distance)) return;
        double d = Math.round(distance * 10) / 10.0;
        points.removeIf(p -> p[0] == d);
        points.add(new double[] {d, calibrationRpm});
        Collections.sort(points, (a, b) -> Double.compare(a[0], b[0]));
        RobotLog.ii("SHOT_POINT", "{%.1f, %.0f},", d, calibrationRpm);
    }

    private String status(Robot robot) {
        StringBuilder s = new StringBuilder();
        if (alliance == null) {
            s.append("alliance: NOT SET (X blue, B red) — no distance, no aim\n");
        } else {
            double live = robot.vision.targetDistance(alliance);
            double error = robot.aimError(alliance);
            String aim = Double.isNaN(error) ? "no target"
                    : AimController.onTarget(error) ? "ON TARGET"
                    : String.format(Locale.US, "turn %+.1f°", Math.toDegrees(error));
            s.append(String.format(Locale.US, "alliance: %s | aim: %s | distance: %s\n",
                    alliance, aim, Double.isNaN(live) ? "not in view (X will not record)"
                            : String.format(Locale.US, "%.1fin", live)));
        }
        s.append(String.format(Locale.US, "calibration speed: %.0f rpm (dpad up/down)\n",
                calibrationRpm));
        double rpm = robot.outtake.rpm();
        s.append(String.format(Locale.US, "at power %.2f: %.0f rpm%s\n", robot.outtake.power(),
                rpm, robot.outtake.power() == 1 && rpm > 0
                        ? String.format(Locale.US, " → KV = 1.0 / %.0f", rpm) : ""));
        s.append(String.format(Locale.US, "encoder since BACK: %d ticks (TICKS_PER_REV = %.0f)\n",
                robot.outtake.position() - encoderZero, Outtake.TICKS_PER_REV));
        s.append("SHOT_POINTS:");
        if (points.isEmpty()) s.append(" none recorded (X)");
        for (double[] p : points) s.append(String.format(Locale.US, " {%.1f, %.0f},", p[0], p[1]));
        return s.toString();
    }

    /** Level-triggered binding: {@code command} is scheduled exactly while {@code held}. */
    private static void whileHeld(Command command, boolean held) {
        if (held && !command.isScheduled()) command.schedule();
        else if (!held && command.isScheduled()) command.cancel();
    }
}
