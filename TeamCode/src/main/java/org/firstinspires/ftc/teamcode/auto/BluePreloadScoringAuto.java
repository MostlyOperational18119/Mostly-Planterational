package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;

import java.util.function.Consumer;

/**
 * Blue alliance: shoots the three preloaded pollen into the blue HIVE's target CELL if vision
 * sees it facing up, then parks in the blue LOADING ZONE. If the CELL is down, or never seen,
 * it skips the shooting and only parks.
 *
 * <p>Starts as the red rectangular robot of the manual's Fig. 11-1, turned 180° for blue
 * ({@link AutoPaths}): back against the rear wall, centred on the third TILE from the blue
 * wall, facing the HIVE. So the target is the FAR CELL, the near one from this start. The
 * status line shows what vision reads for it during init, and the decision is the reading at
 * START. Parking needs Pedro (a localizer and Foresight).
 */
@Autonomous(name = "Blue: Preload Audience", group = "BIOBUZZ")
public class BluePreloadScoringAuto extends AutoBase {

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
        return ShootingRoutines.shootIfUpThenPark(robot, alliance(), target(),
                paths.approachPark, paths.toPark, onDecided);
    }
}
