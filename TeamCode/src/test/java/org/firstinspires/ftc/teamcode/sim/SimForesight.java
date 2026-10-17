package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/**
 * A Foresight configuration matched to {@link SimDrivetrain} — the stand-in for what AutoTune
 * would measure on a robot that has no inertia. Not a starting point for the real robot: these
 * numbers describe the simulator, and the real ones come from the Foresight Tuner.
 */
public final class SimForesight {

    private SimForesight() {}

    public static ForesightConfig config() {
        return new ForesightConfig(c -> {
            c.forwardTranslational.set(Controller.proportional(0.15));
            c.strafeTranslational.set(Controller.proportional(0.15));
            // Power is speed / MAX_SPEED in the sim, so the velocity feedforwards are 1 / MAX_SPEED.
            c.coast.set(Controller.proportionalFeedforward(1 / SimDrivetrain.MAX_SPEED));
            c.brake.set(Controller.proportionalFeedforward(1 / SimDrivetrain.MAX_SPEED));
            c.headingFeedback.set(Controller.proportional(1.0));
            // Stopping takes one loop with no inertia: small, nonzero brake distances.
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.01, 0.0));
            c.linearBrakeCoefficients.set(Matrix.diag(0.01, 0.01));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0001, 0.0001));
            c.maxAchievableForwardVelocity.set(SimDrivetrain.MAX_SPEED);
            c.maxAchievableStrafeVelocity.set(SimDrivetrain.MAX_SPEED);
            c.naturalForwardDeceleration.set(2000.0);
            c.naturalStrafeDeceleration.set(2000.0);
        });
    }
}
