package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    /**
     * Whether the goBILDA Pinpoint and its dead wheels are on the chassis. True: the follower
     * localizes with {@link #localizerConstants}. False (a chassis without pods, or the
     * Pinpoint unplugged for a test): it runs on {@link StubLocalizer}, robot-centric TeleOp
     * still works, and every OpMode that follows a path or needs a real pose (autos, Pedro's
     * {@code Tuning}) refuses to start.
     */
    public static final boolean LOCALIZER_INSTALLED = true;

    public static FollowerConstants followerConstants = new FollowerConstants();

    // Names from the robot configuration. Directions verified on the robot (right side
    // reversed); if pushing forward spins a wheel backwards, flip that wheel here.
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("motorFL")
            .leftRearMotorName("motorBL")
            .rightFrontMotorName("motorFR")
            .rightRearMotorName("motorBR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    /**
     * goBILDA Pinpoint Odometry Computer, I2C device {@code "pinpoint"}, two goBILDA 4-Bar pods.
     *
     * <p>Offsets are the Pinpoint's convention, measured from the centre of rotation: the
     * forward pod's <em>sideways</em> offset (left +) and the strafe pod's <em>fore/aft</em>
     * offset (forward +). Measured: forward pod 3 in right of centre, strafe pod on the
     * centreline fore/aft.
     *
     * <p>Encoder directions are Pedro's defaults, <b>unverified</b>: in Tuning → Localization
     * Test, push the robot forward (x must rise) and to the left (y must rise); flip whichever
     * is backwards. Then spin it in place — if x/y drift while only turning, the offsets are off.
     */
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardPodY(-3)
            .strafePodX(0)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        FollowerBuilder builder = new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints);
        if (LOCALIZER_INSTALLED) builder.pinpointLocalizer(localizerConstants);
        else builder.setLocalizer(new StubLocalizer());
        return builder.build();
    }
}
