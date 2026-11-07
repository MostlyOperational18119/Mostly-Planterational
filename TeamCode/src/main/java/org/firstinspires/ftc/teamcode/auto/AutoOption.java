package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;

import java.util.function.Consumer;

/**
 * One entry of an auto menu: a label, the side whose paths it needs, and how to build its
 * routine. It names its side alongside its routine because {@link AutoPaths} leaves the other
 * side's paths null.
 *
 * <p>Alliance and target are arguments to the builder, not read from an OpMode: touching one
 * would load {@code LinearOpMode}, which a JVM test cannot.
 */
final class AutoOption {

    /** Builds an option's routine, once, at START. */
    interface RoutineBuilder {
        Command build(Robot r, Alliance alliance, Cell target, AutoPaths p,
                      Consumer<Boolean> onDecided);
    }

    final String label;
    final AutoPaths.Side side;
    final RoutineBuilder routine;

    AutoOption(String label, AutoPaths.Side side, RoutineBuilder routine) {
        this.label = label;
        this.side = side;
        this.routine = routine;
    }

    @Override
    public String toString() {
        return label;
    }
}
