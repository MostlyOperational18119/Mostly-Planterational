package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedroPathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedroPathing.procedures.Tests;

/**
 * Pedro 3's AutoTune procedures, registered for the robot-hosted tuning webpage. Each
 * {@code @Tuner} method is found by the tuning library's class scanner; there is no OpMode to
 * pick on the Driver Station.
 *
 * <p>Run them in this order, pasting each one's generated Java into {@link Constants}:
 * <ol>
 *   <li><b>Mecanum Tuner</b> → {@code drivetrainConfig} (motor directions).</li>
 *   <li><b>Pinpoint Tuner</b> → {@code localizerConfig} (pod directions and offsets).</li>
 *   <li><b>Foresight Tuner</b> → {@code foresightConfig}; then set
 *       {@code Constants.FORESIGHT_TUNED = true} and redeploy.</li>
 *   <li><b>Tests</b> — Localization/Odometry/Pose/Driving work at any time; Hold, Line, Curve and
 *       Interpolation need step 3 done first.</li>
 * </ol>
 *
 * <p>The procedures themselves (in {@code procedures/}) are copied unchanged from the
 * Pedro-Pathing Quickstart.
 */
public final class Tuning {

    private Tuning() {}

    @Tuner(name = "Mecanum Tuner")
    static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Tuner")
    static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight Tuner")
    static Procedure foresight() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }
}
