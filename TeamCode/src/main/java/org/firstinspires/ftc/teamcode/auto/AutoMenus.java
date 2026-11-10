package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * What each alliance's menu offers, in menu order; the first is the default. Kept out of the
 * OpModes because touching an OpMode class loads {@code LinearOpMode}, which a JVM test cannot.
 */
final class AutoMenus {

    private AutoMenus() {}

    /** The index {@code by} places along a menu of {@code size}, wrapping at either end. */
    static int step(int index, int by, int size) {
        return (index + by % size + size) % size;
    }

    static final List<AutoOption> BLUE = Collections.unmodifiableList(Arrays.asList(
            new AutoOption("Blue Audience: move to audience wall, shoot, park",
                    AutoPaths.Side.SCORING,
                    (r, alliance, target, p, d) -> ShootingRoutines.driveToHiveThenShootAndPark(r,
                            alliance, target, p.toApproachHive, p.toHive, p.approachPark, p.toPark,
                            d)),
            new AutoOption("Blue Scoring: shoot preload, park", AutoPaths.Side.AUDIENCE,
                    (r, alliance, target, p, d) -> ShootingRoutines.shootIfUpThenPark(r, alliance,
                            target, p.approachPark, p.toPark, d))));

    static final List<AutoOption> RED = Collections.unmodifiableList(Arrays.asList(
            new AutoOption("Red Scoring: move to audience wall, shoot, park",
                    AutoPaths.Side.SCORING,
                    (r, alliance, target, p, d) -> ShootingRoutines.driveToHiveThenShootAndPark(r,
                            alliance, target, p.toApproachHive, p.toHive, p.approachPark, p.toPark,
                            d)),
            new AutoOption("Red Audience: shoot preload, park", AutoPaths.Side.AUDIENCE,
                    (r, alliance, target, p, d) -> ShootingRoutines.shootIfUpThenPark(r, alliance,
                            target, p.approachPark, p.toPark, d))));
}
