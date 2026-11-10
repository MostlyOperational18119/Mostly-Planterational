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
 * <p>{@link BluePreloadScoringAuto} from the scoring side: same target (the FAR CELL), but
 * the start cannot see it, so it first drives to the shooting spot
 * ({@link AutoPaths.Side#SCORING}) and decides on arrival. The status line shows what vision
 * reads for the target until then. Driving there and parking need Pedro (a localizer and
 * Foresight).
 */
@Autonomous(name = "Blue: Preload Scoring", group = "BIOBUZZ")
public class BluePreloadAudienceAuto extends AutoBase {

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
        paths = new AutoPaths(AutoPaths.Side.SCORING);
    }

    @Override
    protected Command buildRoutine(Robot robot, Consumer<Boolean> onDecided) {
        return ShootingRoutines.driveToHiveThenShootAndPark(robot, alliance(), target(),
                paths.toApproachHive, paths.toHive, paths.approachPark, paths.toPark, onDecided);
    }
}
