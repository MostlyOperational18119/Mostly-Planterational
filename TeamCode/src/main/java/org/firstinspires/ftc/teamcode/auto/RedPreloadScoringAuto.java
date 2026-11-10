package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;

import java.util.function.Consumer;

/**
 * Red alliance: shoots the three preloaded pollen into the red HIVE's target CELL if vision
 * sees it facing up, then parks in the red LOADING ZONE. If the CELL is down, or never seen,
 * it skips the shooting and only parks.
 *
 * <p>{@link RedPreloadAudienceAuto} from the scoring side: same target (the AUDIENCE CELL), but
 * the start cannot see it, so it first drives to the shooting spot
 * ({@link AutoPaths.Side#SCORING}) and decides on arrival. The status line shows what vision
 * reads for the target until then. Driving there and parking need Pedro (a localizer and
 * Foresight).
 */
@Autonomous(name = "Red: Preload Scoring", group = "BIOBUZZ")
public class RedPreloadScoringAuto extends AutoBase {

    private AutoPaths paths;

    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected Cell target() {
        return Cell.RED_AUDIENCE;
    }

    @Override
    protected void onInit(Robot robot) {
        paths = new AutoPaths(AutoPaths.Side.SCORING);
    }

    @Override
    protected Command buildRoutine(Robot robot, Consumer<Boolean> onDecided) {
        return ShootingRoutines.driveToHiveThenShootAndPark(robot, alliance(), target(),
                paths.toHive, paths.shootFrom, paths.toPark, onDecided);
    }
}
