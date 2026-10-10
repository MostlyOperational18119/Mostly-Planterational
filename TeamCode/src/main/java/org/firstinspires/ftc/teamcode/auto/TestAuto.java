package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.routines.TestRoutines;
import org.firstinspires.ftc.teamcode.util.Timeouts;

/**
 * Step 5 of the implementation order (architecture doc §10): one routine that overlaps a
 * mechanism with path following, on the real loop. Needs ~30 in of clear floor along the
 * robot's +x (forward) direction.
 *
 * <p>Telemetry shows the routine's wall-clock time and the loop timing captured the moment it
 * finished — the two numbers to compare against last season. The live loop line keeps
 * updating afterwards, but by then it is mostly idle cycles.
 */
@Autonomous(name = "Test: Out and Back", group = "BIOBUZZ Test")
public class TestAuto extends LinearOpMode {

    private static final double AUTO_DEADLINE_MS = 28_000;

    private long startNanos, endNanos;
    private String loopAtEnd = "";

    @Override
    public void runOpMode() {
        Scheduler.reset();                      // MANDATORY — the Scheduler's state is static

        if (!Constants.LOCALIZER_INSTALLED) {
            // On StubLocalizer the pose never moves, so each leg would drive at full power until
            // its 5 s timeout. Refuse rather than crash, so the reason is on the Driver Station.
            telemetry.addLine("Disabled: no localizer installed (Constants.LOCALIZER_INSTALLED).");
            telemetry.update();
            waitForStart();
            return;
        }

        Robot robot = Robot.fromHardwareMap(hardwareMap)
                .withTelemetry(PanelsTelemetry.INSTANCE.getTelemetry())
                .withStatus(this::status);
        TestPaths paths = new TestPaths(robot.follower);
        robot.follower.setStartingPose(TestPaths.START);

        // Failure containment (§8.4): the routine ends on whichever comes first — finishing,
        // the global deadline, or losing localization.
        Command routine = Groups.race(
                TestRoutines.outAndBack(robot, paths),
                Timeouts.command(AUTO_DEADLINE_MS),
                Commands.waitUntil(robot.drive::localizationLost));

        while (opModeInInit()) {
            robot.initCycle();
        }

        robot.follower.update();
        routine.schedule();
        startNanos = System.nanoTime();

        while (opModeIsActive()) {
            robot.cycle();
            if (endNanos == 0 && !routine.isScheduled()) {
                endNanos = System.nanoTime();
                loopAtEnd = robot.loopTimer().summary();
            }
        }

        Scheduler.reset();
        robot.stop();
    }

    private String status() {
        if (startNanos == 0) return "routine: waiting for start";
        if (endNanos == 0) {
            return String.format("routine: running %.2fs", (System.nanoTime() - startNanos) / 1e9);
        }
        return String.format("routine: done in %.2fs | at end: %s",
                (endNanos - startNanos) / 1e9, loopAtEnd);
    }
}
