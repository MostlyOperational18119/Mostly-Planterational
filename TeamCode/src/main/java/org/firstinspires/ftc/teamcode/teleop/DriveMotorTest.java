package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drive-motor bring-up: spins one drive motor at a time, by the names and directions in
 * {@link Constants#driveConstants}, so you can see which wheel each config name really is.
 *
 * <p>The buttons sit where the wheels do: left bumper = front-left, right bumper = front-right,
 * left trigger = back-left, right trigger = back-right. Hold one with the robot on blocks:
 *
 * <ul>
 *   <li>The <b>wrong wheel</b> spins → the config names (or cables) are swapped. Fix the robot
 *       configuration on the Driver Station; no code change.</li>
 *   <li>The right wheel spins <b>backwards</b> → flip that motor's direction in
 *       {@code Constants}.</li>
 *   <li>Every wheel is right and forward, yet strafing is still wrong → the mecanum wheels are
 *       on the wrong corners. Seen from above, the rollers should form an X.</li>
 * </ul>
 *
 * <p>A diagnostic, deliberately outside the {@code Robot} loop contract: it drives raw motors
 * and nothing else runs.
 */
@TeleOp(name = "Test: Drive Motors", group = "BIOBUZZ Test")
@Disabled
public class DriveMotorTest extends LinearOpMode {

    private static final double POWER = 0.3;

    @Override
    public void runOpMode() {
        MecanumConstants c = Constants.driveConstants;
        DcMotorEx fl = motor(c.leftFrontMotorName, c.leftFrontMotorDirection);
        DcMotorEx fr = motor(c.rightFrontMotorName, c.rightFrontMotorDirection);
        DcMotorEx bl = motor(c.leftRearMotorName, c.leftRearMotorDirection);
        DcMotorEx br = motor(c.rightRearMotorName, c.rightRearMotorDirection);

        telemetry.addLine("Robot on blocks. Hold LB = front-left, RB = front-right,");
        telemetry.addLine("LT = back-left, RT = back-right. Each should spin its wheel forward.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean flOn = gamepad1.left_bumper;
            boolean frOn = gamepad1.right_bumper;
            boolean blOn = gamepad1.left_trigger > 0.5;
            boolean brOn = gamepad1.right_trigger > 0.5;
            fl.setPower(flOn ? POWER : 0);
            fr.setPower(frOn ? POWER : 0);
            bl.setPower(blOn ? POWER : 0);
            br.setPower(brOn ? POWER : 0);

            telemetry.addData("LB front-left ", "%s %s", c.leftFrontMotorName, flOn ? "ON" : "");
            telemetry.addData("RB front-right", "%s %s", c.rightFrontMotorName, frOn ? "ON" : "");
            telemetry.addData("LT back-left  ", "%s %s", c.leftRearMotorName, blOn ? "ON" : "");
            telemetry.addData("RT back-right ", "%s %s", c.rightRearMotorName, brOn ? "ON" : "");
            telemetry.update();
        }
    }

    private DcMotorEx motor(String name, DcMotorSimple.Direction direction) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(direction);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }
}
