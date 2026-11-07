package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;
import java.util.function.Consumer;

/**
 * An {@link AutoBase} with the auto chosen during init: dpad up/down on gamepad 1 moves through
 * {@link #options()}, and whichever is selected at START runs. Nobody touching the gamepad runs
 * the first one.
 *
 * <p>The options are complete autos rather than steps to assemble on the Driver Station, so
 * every one can be driven on a field before a match relies on it. To add one: put its poses
 * and paths in {@link AutoPaths}, its routine in
 * {@link org.firstinspires.ftc.teamcode.routines.ShootingRoutines} with a timeout on every leg,
 * and a line in the subclass's options. Long routines must fit {@link #AUTO_DEADLINE_MS}, which
 * ends the routine wherever the robot has got to.
 */
public abstract class AutoMenuBase extends AutoBase {

    private int selected;
    private AutoPaths audience, scoring;

    /** The menu, in order; the first is the default. */
    abstract List<AutoOption> options();

    @Override
    protected void onInit(Robot robot) {
        // Both built now, since the choice can change until START (rule 6). The start pose does
        // not depend on the choice: each side's poses are in the frame of its own start.
        audience = new AutoPaths(AutoPaths.Side.AUDIENCE);
        scoring = new AutoPaths(AutoPaths.Side.SCORING);
    }

    @Override
    protected void onInitCycle() {
        // Each WasPressed() resets when read, so once per loop.
        int size = options().size();
        if (gamepad1.dpadUpWasPressed()) selected = AutoMenus.step(selected, -1, size);
        if (gamepad1.dpadDownWasPressed()) selected = AutoMenus.step(selected, 1, size);
    }

    @Override
    protected Command buildRoutine(Robot robot, Consumer<Boolean> onDecided) {
        AutoOption option = options().get(selected);
        AutoPaths paths = option.side == AutoPaths.Side.AUDIENCE ? audience : scoring;
        return option.routine.build(robot, alliance(), target(), paths, onDecided);
    }

    @Override
    protected String header() {
        return started() ? "auto: " + options().get(selected).label : menu();
    }

    /** The options, one per line, marking the selected one. */
    private String menu() {
        StringBuilder menu = new StringBuilder("Auto (dpad up/down):");
        List<AutoOption> options = options();
        for (int i = 0; i < options.size(); i++) {
            menu.append(i == selected ? "\n> " : "\n   ").append(options.get(i).label);
        }
        return menu.toString();
    }
}
