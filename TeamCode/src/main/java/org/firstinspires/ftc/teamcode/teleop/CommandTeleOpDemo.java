package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.ivy.pedro.PedroCommands.*;

public class CommandTeleOpDemo extends LinearOpMode {
    Follower follower;
    TelemetryManager telemetryManager;

    @Override
    public void runOpMode() {
        // It's static, so we need to reset it at the start of each OpMode
        Scheduler.reset();

        // Create and setup the Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        follower.update();

        // Setup telemetry
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        // Does what it says on the tin
        waitForStart();

        // Make the follower do TeleOp driving
        follower.startTeleOpDrive();

        // Main loop
        while (opModeIsActive()) {
            // Update both the follower and telemetry
            follower.update();
            telemetryManager.update();

            // Actually drive the robot with Pedro
            // I don't think there's a way to make this an Ivy command
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
        }
    }
}
