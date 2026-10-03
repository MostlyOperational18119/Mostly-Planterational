package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    /**
     * False until the goBILDA Pinpoint and dead wheels are on the chassis. While false the
     * follower runs on {@link StubLocalizer}: robot-centric TeleOp works, and every OpMode
     * that follows a path or needs a real pose (autos, Pedro's {@code Tuning}) refuses to
     * start. When the Pinpoint goes on, flip this, replace the {@code setLocalizer} call
     * below with {@code .pinpointLocalizer(new PinpointConstants()...)}, then run
     * {@code Tuning} → Localization before anything else.
     */
    public static final boolean LOCALIZER_INSTALLED = false;

    public static FollowerConstants followerConstants = new FollowerConstants();

    // Names from the robot configuration. Directions are Pedro's defaults (left side
    // reversed); if pushing forward spins a wheel backwards, flip that wheel here.
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("motorFL")
            .leftRearMotorName("motorBL")
            .rightFrontMotorName("motorFR")
            .rightRearMotorName("motorBR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(new StubLocalizer())
                .pathConstraints(pathConstraints)
                .build();
    }
}
