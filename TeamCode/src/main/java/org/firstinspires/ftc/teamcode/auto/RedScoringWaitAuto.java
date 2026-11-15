package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;

import java.util.function.Consumer;

/**
 * Red alliance, scoring side ({@link AutoPaths.Side#SCORING}'s poses).
 * Starts in the corner: robot centre 135 in from the back wall and 9 in from the alliance
 * wall; drives out to the shooting spot past the HIVE's far end
 * ({@link AutoPaths#fieldShoot}, back 120 / alliance 60, the one the Preload + Field autos
 * use), turning on the way to face the HIVE; waits {@code ShootingRoutines.WAIT_BEFORE_SHOOT_MS}
 * there; shoots the preload into the red HIVE's FAR CELL, the one in front of it from
 * there, if vision sees it facing up; then parks in the red LOADING ZONE whether it shot or not.
 *
 * <p>Not yet driven on a field; driving needs Pedro (a localizer and Foresight).
 */
//@Autonomous(name = "Red: Scoring Wait", group = "BIOBUZZ")
public class RedScoringWaitAuto extends AutoBase {

    private AutoPaths paths;

    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    protected Cell target() {
        return Cell.RED_FAR;
    }

    @Override
    protected void onInit(Robot robot) {
        paths = new AutoPaths(AutoPaths.Side.SCORING);
    }

    @Override
    protected Command buildRoutine(Robot robot, Consumer<Boolean> onDecided) {
        return ShootingRoutines.driveToHiveThenWaitThenShoot(robot, alliance(), target(),
                paths.toFieldShoot, paths.fieldShoot, paths.toParkFromField, onDecided);
    }
}
