package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.routines.ShootingRoutines;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Timeouts;
import org.firstinspires.ftc.teamcode.vision.CellSighting;

import java.util.function.Consumer;

/**
 * Blue alliance, with the auto chosen during init: dpad up/down on gamepad 1 moves through the
 * {@link Option}s, and whichever is selected at START runs. Nobody touching the gamepad runs
 * the first one.
 *
 * <p>The options are complete autos rather than steps to assemble on the Driver Station, so
 * every one can be driven on a field before a match relies on it. To add one: put its poses
 * and paths in {@link AutoPaths}, its routine in {@link ShootingRoutines} with a timeout on
 * every leg, and a line in {@link Option}. Long routines must fit {@link #AUTO_DEADLINE_MS},
 * which ends the routine wherever the robot has got to.
 *
 * <p>The status line shows what vision reads for the target until the routine decides whether
 * to shoot. Driving and parking need Pedro (a localizer and Foresight).
 */
@Autonomous(name = "Blue: Auto Menu", group = "BIOBUZZ")
public class BlueAutoMenu extends LinearOpMode {

    private static final Alliance ALLIANCE = Alliance.BLUE;
    private static final Cell TARGET = Cell.BLUE_FAR;
    private static final double AUTO_DEADLINE_MS = 28_000;

    private Option selected = Option.values()[0];
    private long startNanos, endNanos;
    /** What was decided, and the reading behind it; null until then. */
    private String decision;

    /**
     * Builds an option's routine, once, at START. Alliance and target are arguments, not read
     * from this class: touching it would load {@code LinearOpMode}, which a JVM test cannot.
     */
    interface RoutineBuilder {
        Command build(Robot r, Alliance alliance, Cell target, AutoPaths p,
                      Consumer<Boolean> onDecided);
    }

    /**
     * What the menu offers, in menu order; the first is the default. An option names its side
     * alongside its routine because {@link AutoPaths} leaves the other side's paths null.
     */
    enum Option {
        AUDIENCE_PRELOAD("Blue Audience: move to audience wall, shoot, park", AutoPaths.Side.SCORING,
                (r, alliance, target, p, d) -> ShootingRoutines.driveToHiveThenShootAndPark(r,
                        alliance, target, p.toApproachHive, p.toHive, p.approachPark, p.toPark, d)),
        SCORING_PRELOAD("Blue Scoring: shoot preload, park", AutoPaths.Side.AUDIENCE,
                (r, alliance, target, p, d) -> ShootingRoutines.shootIfUpThenPark(r, alliance,
                        target, p.approachPark, p.toPark, d));
        final String label;
        final AutoPaths.Side side;
        final RoutineBuilder routine;

        Option(String label, AutoPaths.Side side, RoutineBuilder routine) {
            this.label = label;
            this.side = side;
            this.routine = routine;
        }

        /** The option {@code by} places along the menu, wrapping at either end. */
        Option step(int by) {
            Option[] all = values();
            return all[(ordinal() + by + all.length) % all.length];
        }
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        if (!Constants.canFollowPaths()) {
            // Every option drives; on StubLocalizer, or before AutoTune, it cannot. Refuse
            // rather than start a routine that cannot finish.
            telemetry.addLine(!Constants.LOCALIZER_INSTALLED
                    ? "Disabled: no localizer installed (Constants.LOCALIZER_INSTALLED)."
                    : "Disabled: Foresight not tuned yet — run AutoTune (Constants.FORESIGHT_TUNED).");
            telemetry.update();
            waitForStart();
            return;
        }

        Robot robot = Robot.fromHardwareMap(hardwareMap)
                .withTelemetry(PanelsTelemetry.INSTANCE.getTelemetry(), telemetry);
        robot.withStatus(() -> status(robot));
        // Both built now, since the choice can change until START (rule 6). The start pose does
        // not depend on the choice: each side's poses are in the frame of its own start.
        AutoPaths audience = new AutoPaths(AutoPaths.Side.AUDIENCE);
        AutoPaths scoring = new AutoPaths(AutoPaths.Side.SCORING);
        robot.follower.setPose(AutoPaths.START);

        while (opModeInInit()) {
            // Each WasPressed() resets when read, so once per loop.
            if (gamepad1.dpadUpWasPressed()) selected = selected.step(-1);
            if (gamepad1.dpadDownWasPressed()) selected = selected.step(1);
            robot.initCycle();
        }

        AutoPaths paths = selected.side == AutoPaths.Side.AUDIENCE ? audience : scoring;

        // Failure containment (§8.4): ends on finishing, the deadline, or losing localization.
        Command routine = Groups.race(
                selected.routine.build(robot, ALLIANCE, TARGET, paths,
                        shoot -> decision = String.format(
                                "%s at %.1fs (%s)", shoot ? "SHOOTING" : "NOT SHOOTING",
                                (System.nanoTime() - startNanos) / 1e9, reading(robot))),
                Timeouts.command(AUTO_DEADLINE_MS),
                Commands.waitUntil(robot.drive::localizationLost));

        robot.follower.update();
        routine.schedule();
        startNanos = System.nanoTime();

        while (opModeIsActive()) {
            robot.cycle();
            if (endNanos == 0 && !routine.isScheduled()) endNanos = System.nanoTime();
        }

        Scheduler.reset();
        robot.stop();
    }

    private String status(Robot robot) {
        String routine = startNanos == 0 ? "waiting for start"
                : endNanos == 0 ? String.format("running %.2fs", (System.nanoTime() - startNanos) / 1e9)
                : String.format("done in %.2fs", (endNanos - startNanos) / 1e9);
        String now = decision != null ? "decided: " + decision : reading(robot);
        String auto = startNanos == 0 ? menu() : "auto: " + selected.label;
        return auto + "\ntarget: " + TARGET + " | " + now + " | routine: " + routine;
    }

    /** The options, one per line, marking the selected one. */
    private String menu() {
        StringBuilder menu = new StringBuilder("Auto (dpad up/down):");
        for (Option option : Option.values()) {
            menu.append(option == selected ? "\n> " : "\n   ").append(option.label);
        }
        return menu.toString();
    }

    /**
     * What vision makes of the target right now, with the numbers behind it: the CELL is told
     * up or down by its measured height, so a reading just under the threshold is the first
     * thing to look for when the robot does not shoot at an up CELL. "Would", because a
     * scoring-side routine decides only once it has driven to the HIVE.
     */
    private String reading(Robot robot) {
        if (!robot.vision.hasFix()) return "vision " + robot.vision.status() + " — would only park";
        Cell up = robot.vision.upCell(ALLIANCE);
        CellSighting seen = robot.vision.sighting(TARGET);
        String height = seen == null ? "target not in view"
                : String.format("target %.1f in high, %.1f in away (up ≥ %.1f)", seen.z,
                        seen.floorDistance(), Vision.UP_CELL_MIN_HEIGHT_IN);
        String verdict = up == null ? "no " + ALLIANCE + " CELL seen — would only park"
                : up == TARGET ? "UP — would shoot, then park" : "DOWN — would only park";
        return verdict + "; " + height;
    }
}
