package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3 configuration. A {@link Follower} is three parts — a localizer, a drivetrain
 * and a path-following algorithm (Foresight) — each with its own config below. AutoTune
 * ({@link Tuning}) measures all three and prints the Java to paste back here.
 */
public class Constants {

    /**
     * Whether the goBILDA Pinpoint and its dead wheels are on the chassis. True: the follower
     * localizes with {@link #localizerConfig}. False (a chassis without pods, or the Pinpoint
     * unplugged for a test): it runs on {@link StubLocalizer}, robot-centric TeleOp still
     * works, and every OpMode that needs a real pose refuses to start.
     */
    public static final boolean LOCALIZER_INSTALLED = true;

    /**
     * Whether {@link #foresightConfig} holds AutoTune's measurements. Foresight cannot even be
     * constructed without them, so while this is false the follower runs on
     * {@link UntunedAlgorithm}: TeleOp driving, hold-to-aim and the AutoTune pages work; path
     * following, {@code turnTo}/{@code hold} and {@code TestAuto} do not. Flip it once the
     * ForesightTuner's output is pasted into {@link #foresightConfig}.
     */
    public static final boolean FORESIGHT_TUNED = true;

    /** True when paths, {@code turnTo} and {@code hold} can work. */
    public static boolean canFollowPaths() {
        return LOCALIZER_INSTALLED && FORESIGHT_TUNED;
    }

    // Names from the robot configuration. Directions verified on the robot (right side
    // reversed); if pushing forward spins a wheel backwards, flip that wheel here. AutoTune's
    // Mecanum Tuner checks them interactively.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("motorFL");
        c.backLeftName.set("motorBL");
        c.frontRightName.set("motorFR");
        c.backRightName.set("motorBR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    /**
     * goBILDA Pinpoint Odometry Computer, I2C device {@code "pinpoint"}, two goBILDA 4-Bar pods.
     *
     * <p>Offsets are the Pinpoint's convention, measured from the centre of rotation: the
     * x (forward-measuring) pod's <em>sideways</em> offset, left +, and the y (strafe-measuring)
     * pod's <em>fore/aft</em> offset, forward +. Measured: forward pod 3 in right of centre,
     * strafe pod on the centreline fore/aft. Directions as verified in Pedro 2's Localization
     * Test. AutoTune's Pinpoint Tuner re-measures all four.
     *
     * <p>Constructing the localizer recalibrates the Pinpoint's IMU and sleeps 500 ms — init
     * only, and the robot must be still.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.5); // measured closer to -3.0
        c.yPodOffset.set(-0.5); // and 0.0, but tuner outputted this
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /**
     * Foresight, Pedro 3's path follower. <b>Empty until AutoTune has run</b>: paste the
     * ForesightTuner's generated block here, then set {@link #FORESIGHT_TUNED}.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27821191648616167);
                Controller secondaryTranslationalForward = Controller.proportional(0.10279186918231503);
                Controller primaryTranslationalLateral = Controller.proportional(0.3650113621296331);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13486194502367452);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012111074949409481));
                c.brake.set(Controller.proportionalFeedforward(0.010294413706998058));

                c.headingFeedback.set(Controller.proportional(4.67241772496501));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05495324202840157, 0.004803787126252708));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11758689312120184, 0.06353945655825877));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.213075840783736E-4, 0.00166257899579791));

                c.maxAchievableForwardVelocity.set(80.29533174944831);
                c.maxAchievableStrafeVelocity.set(65.40178736314184);
                c.naturalForwardDeceleration.set(28.188957576751253);
                c.naturalStrafeDeceleration.set(58.60574023521338);
            }
    );

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return LOCALIZER_INSTALLED ? new PinpointLocalizer(hardwareMap, localizerConfig)
                : new StubLocalizer();
    }

    public static Algorithm createAlgorithm() {
        return FORESIGHT_TUNED ? new Foresight(foresightConfig) : new UntunedAlgorithm();
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap),
                createAlgorithm());
    }
}
