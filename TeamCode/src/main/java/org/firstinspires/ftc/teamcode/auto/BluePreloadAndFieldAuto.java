package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;

import java.util.function.Consumer;

/**
 * Blue alliance: shoots the preload into the blue HIVE's target CELL if vision sees it facing
 * up, then turns 90°, drives to the end pose collecting four pollen, returns to the start and
 * shoots them the same way, and parks in the blue LOADING ZONE whether it shot or not.
 *
 * <p>Starts where {@link BluePreloadScoringAuto} does: back against the rear wall, centred on
 * the third TILE from the blue wall, facing the HIVE. So the target is the FAR CELL. The poses
 * are the {@code PRELOAD_AND_FIELD_*} settings in {@link AutoPaths}. Not yet driven on a field;
 * driving needs Pedro (a localizer and Foresight).
 */
@Autonomous(name = "Blue: Preload + Field", group = "BIOBUZZ")
public class BluePreloadAndFieldAuto extends AutoBase {

    private AutoPaths paths;

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected Cell target() {
        return Cell.BLUE_FAR;
    }

    @Override
    protected void onInit(Robot robot) {
        paths = new AutoPaths(AutoPaths.Side.AUDIENCE);
    }

    @Override
    protected Command buildRoutine(Robot robot, Consumer<Boolean> onDecided) {
        return ShootingRoutines.preloadThenFieldCycle(robot, alliance(), target(),
                AutoPaths.START, paths.fieldTurned, paths.toFieldEnd, paths.toFieldStart,
                paths.toPark, onDecided);
    }
}
