package org.firstinspires.ftc.teamcode.routines;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.field.Alliance;
import org.firstinspires.ftc.teamcode.field.Cell;
import org.firstinspires.ftc.teamcode.sim.FakeVisionSource;
import org.firstinspires.ftc.teamcode.sim.OuttakeTuning;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.AimController;
import org.firstinspires.ftc.teamcode.util.RangeController;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** The aim-and-shoot routine on the simulated robot, with a perfect camera. */
public class ShootingRoutinesTest {

    private SimRobot sim;
    private OuttakeTuning tuning;

    @Before
    public void setUp() {
        tuning = OuttakeTuning.pinForTests();
        sim = new SimRobot(new Pose(0, 0, 0));
    }

    @After
    public void restoreTuning() {
        ShootingRoutines.AIM_TIMEOUT_MS = 2000;
        ShootingRoutines.FEED_MS = 1500;
        ShootingRoutines.SHOOT_AIM_TOLERANCE_DEG = 3;
        ShootingRoutines.LEAVE_TIMEOUT_MS = 3000;
        ShootingRoutines.DECIDE_TIMEOUT_MS = 1000;
        ShootingRoutines.PRELOAD_FEED_MS = 5000;
        ShootingRoutines.RANGE_TIMEOUT_MS = 4000;
        ShootingRoutines.AUTO_INTAKE_POWER = 0.8;
        tuning.restore();
    }

    /** The error to a fixed field point, from the simulated pose. */
    private double aimErrorTo(double targetX, double targetY) {
        Pose p = sim.pose();
        double dx = targetX - p.x(), dy = targetY - p.y();
        double h = p.heading();
        double x = dx * Math.cos(h) + dy * Math.sin(h);     // into the robot frame
        double y = -dx * Math.sin(h) + dy * Math.cos(h);
        return AimController.error(x, y);
    }

    private boolean fed() {
        return sim.transferMotor.writes().contains(Transfer.FEED_POWER);
    }

    @Test
    public void aimsThenFeedsThenReleasesEverything() {
        ShootingRoutines.AIM_TIMEOUT_MS = 4000;
        ShootingRoutines.FEED_MS = 300;
        Command routine = ShootingRoutines.aimAndShoot(sim.robot, () -> aimErrorTo(60, 40), () -> Double.NaN);
        routine.schedule();

        assertTrue("never started feeding", sim.cycleUntil(sim.robot.transfer::running, 4000));
        assertTrue("fed before it was aimed", ShootingRoutines.aimedToShoot(aimErrorTo(60, 40)));
        assertTrue("flywheel not spinning while feeding", sim.robot.outtake.running());
        sim.cycle();
        assertEquals("an auto launch runs the intake at its own, higher power",
                ShootingRoutines.AUTO_INTAKE_POWER, sim.intakeMotor.power(), 0.0);

        assertTrue("never finished", sim.cycleUntil(() -> !routine.isScheduled(), 2000));
        sim.cycle();
        assertEquals(0.0, sim.transferMotor.power(), 0.0);
        assertEquals(0.0, sim.intakeMotor.power(), 0.0);
        assertEquals(0.0, sim.outtakeMotor.power(), 0.0);
        double heading = sim.pose().heading();
        sim.cycles(20);
        assertEquals("drive must stop turning once released", heading, sim.pose().heading(), 1e-3);
    }

    @Test
    public void withNoTargetItStillShootsAfterTheAimTimeout() {
        ShootingRoutines.AIM_TIMEOUT_MS = 300;
        ShootingRoutines.FEED_MS = 200;
        Command routine = ShootingRoutines.aimAndShoot(sim.robot, () -> Double.NaN, () -> Double.NaN);
        routine.schedule();
        long start = System.nanoTime();

        assertTrue("never finished", sim.cycleUntil(() -> !routine.isScheduled(), 3000));
        double ms = (System.nanoTime() - start) / 1e6;
        assertTrue("finished before the aim timeout: " + ms, ms >= 300);
        assertTrue(fed());
        assertEquals("nothing to aim at, so no turning", 0, sim.pose().heading(), 1e-3);
    }

    @Test
    public void underVelocityControlItWaitsForTheFlywheel() {
        Outtake.TARGET_RPM = 3000;
        ShootingRoutines.AIM_TIMEOUT_MS = 5000;
        Command routine = ShootingRoutines.aimAndShoot(sim.robot, () -> 0.0, () -> Double.NaN);
        routine.schedule();

        sim.cycles(20);
        assertFalse("fed into a flywheel at 0 rpm", fed());

        sim.outtakeMotor.setVelocity(3000 / 60.0 * Outtake.TICKS_PER_REV);
        assertTrue("never fed once at speed", sim.cycleUntil(this::fed, 200));
    }

    /** A camera that sees red's up CELL at field point (tx, ty), from wherever the sim is. */
    private void see(double tx, double ty) {
        Pose p = sim.pose();
        double dx = tx - p.x(), dy = ty - p.y(), h = p.heading();
        sim.camera.sample = new FakeVisionSource.Frame(System.currentTimeMillis(), 0)
                .cluster(Cell.RED_FAR, dx * Math.cos(h) + dy * Math.sin(h),
                        -dx * Math.sin(h) + dy * Math.cos(h), 49)
                .build();
    }

    /** Cycles until {@code routine} ends, showing the camera red's FAR CELL at {@code height}. */
    private boolean runSeeingRedFarAt(Command routine, double height, long timeoutMs) {
        return runSeeing(Cell.RED_FAR, routine, height, timeoutMs);
    }

    /** Cycles until {@code routine} ends, showing the camera {@code cell} at {@code height},
     *  50 in ahead of the start. */
    private boolean runSeeing(Cell cell, Command routine, double height, long timeoutMs) {
        double tx = 50, ty = 0;
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;
        while (routine.isScheduled() && System.nanoTime() < deadline) {
            Pose p = sim.pose();
            double dx = tx - p.x(), dy = ty - p.y(), h = p.heading();
            sim.camera.sample = new FakeVisionSource.Frame(System.currentTimeMillis(), 0)
                    .cluster(cell, dx * Math.cos(h) + dy * Math.sin(h),
                            -dx * Math.sin(h) + dy * Math.cos(h), height)
                    .build();
            sim.cycle();
        }
        return !routine.isScheduled();
    }

    @Test
    public void shootIfUpShootsWhenTheTargetCellIsUp() {
        Outtake.SPIN_UP_MS = 0;
        ShootingRoutines.PRELOAD_FEED_MS = 200;
        sim.camera.online = true;
        Command routine = ShootingRoutines.shootIfUp(sim.robot, Alliance.RED, Cell.RED_FAR);
        routine.schedule();

        assertTrue("never finished", runSeeingRedFarAt(routine, 49, 6000));
        assertTrue("did not shoot at an up CELL", fed());
    }

    @Test
    public void shootIfUpStopsWhenTheTargetCellIsDown() {
        sim.camera.online = true;
        Command routine = ShootingRoutines.shootIfUp(sim.robot, Alliance.RED, Cell.RED_FAR);
        routine.schedule();

        assertTrue("never finished", runSeeingRedFarAt(routine, 34, 1000));
        assertFalse("shot at a down CELL", fed());
        assertFalse(sim.robot.outtake.running());
    }

    @Test
    public void blueShootsWhenItsAudienceCellIsUp() {
        // What the blue auto runs: blue's tags (38–41) must go all the way to a launch.
        Command routine = ShootingRoutines.shootIfUp(sim.robot, Alliance.BLUE, Cell.BLUE_AUDIENCE);
        routine.schedule();

        assertTrue("never finished", runSeeing(Cell.BLUE_AUDIENCE, routine, 49, 6000));
        assertTrue("did not shoot at blue's up CELL", fed());
    }

    @Test
    public void blueStopsWhenItsAudienceCellIsDown() {
        Command routine = ShootingRoutines.shootIfUp(sim.robot, Alliance.BLUE, Cell.BLUE_AUDIENCE);
        routine.schedule();

        assertTrue("never finished", runSeeing(Cell.BLUE_AUDIENCE, routine, 34, 1000));
        assertFalse("shot at a down CELL", fed());
    }

    @Test
    public void blueIgnoresTheRedHive() {
        // Red's CELL up and in view is "not seen" for blue: it must not shoot at it.
        Command routine = ShootingRoutines.shootIfUp(sim.robot, Alliance.BLUE, Cell.BLUE_AUDIENCE);
        routine.schedule();

        assertTrue("never finished", runSeeing(Cell.RED_AUDIENCE, routine, 49, 1000));
        assertFalse("shot at the other alliance's HIVE", fed());
    }

    @Test
    public void shootIfUpStopsWhenItNeverSeesTheHive() {
        ShootingRoutines.DECIDE_TIMEOUT_MS = 200;
        Command routine = ShootingRoutines.shootIfUp(sim.robot, Alliance.RED, Cell.RED_FAR);
        routine.schedule();

        assertTrue("never finished", sim.cycleUntil(() -> !routine.isScheduled(), 1000));
        assertFalse("shot blind", fed());
    }

    @Test
    public void leavesTheCornerBeforeDecidingThenShoots() {
        Outtake.SPIN_UP_MS = 0;
        ShootingRoutines.PRELOAD_FEED_MS = 200;
        sim.camera.online = true;
        Path leave = Paths.line(new Pose(0, 0, 0), new Pose(12, 0, 0)).constant(0);
        Command routine = ShootingRoutines.leaveThenShootIfUp(sim.robot, leave, Alliance.RED,
                Cell.RED_FAR);
        routine.schedule();

        // The camera sees nothing from the corner; from the move's end it sees the up CELL.
        long deadline = System.nanoTime() + 8_000_000_000L;
        while (routine.isScheduled() && System.nanoTime() < deadline) {
            if (sim.pose().x() > 10) see(80, 0);
            else sim.camera.sample = null;
            sim.cycle();
            if (fed()) assertTrue("fed before leaving the corner", sim.pose().x() > 10);
        }
        assertFalse("never finished", routine.isScheduled());
        assertTrue("did not shoot once it could see the up CELL", fed());
    }

    @Test
    public void teleopShootTurnsDrivesToRangeThenFeeds() {
        Outtake.SPIN_UP_MS = 100;
        sim.camera.online = true;
        // Target 70 in away and 20 in to the left: has to turn and close 20 in.
        double tx = 70 * Math.cos(0.28), ty = 70 * Math.sin(0.28);
        Command shoot = ShootingRoutines.teleopShoot(sim.robot, Alliance.RED,
                () -> 0, () -> 0, () -> 0);
        see(tx, ty);
        sim.cycle();
        shoot.schedule();

        long deadline = System.nanoTime() + 5_000_000_000L;
        while (!sim.robot.transfer.running() && System.nanoTime() < deadline) {
            see(tx, ty);
            sim.cycle();
        }
        assertTrue("never fed", sim.robot.transfer.running());
        double distance = Math.hypot(tx - sim.pose().x(), ty - sim.pose().y());
        assertEquals("fed out of range", RangeController.SHOT_DISTANCE_IN, distance,
                RangeController.TOLERANCE_IN);
        assertTrue("fed before aimed", ShootingRoutines.aimedToShoot(sim.robot.aimError(Alliance.RED)));
        assertTrue(sim.robot.outtake.running());
        assertEquals("hold-Y keeps the regular intake power",
                Intake.COLLECT_POWER, sim.robot.intake.power(), 0.0);

        shoot.cancel();
        sim.cycle();
        assertEquals("releasing stops the feed", 0.0, sim.transferMotor.power(), 0.0);
        assertEquals("releasing stops the intake", 0.0, sim.intakeMotor.power(), 0.0);
        assertEquals(0.0, sim.outtakeMotor.power(), 0.0);
    }

    @Test
    public void teleopShootWithNoTargetNeverFeedsAndLeavesTheSticksLive() {
        Outtake.SPIN_UP_MS = 0;
        Command shoot = ShootingRoutines.teleopShoot(sim.robot, Alliance.RED,
                () -> 0.5, () -> 0, () -> 0);
        shoot.schedule();
        sim.cycles(100);
        assertFalse("fed blind", fed());
        assertTrue("driver's forward ignored", sim.pose().x() > 1);
    }

    @Test
    public void aRobotStalledJustOutsideTheAimDeadbandStillFeeds() {
        // The aim controller stops correcting inside AimController.TOLERANCE (1.5°); a robot
        // that comes to rest at 2° off — too little turn power left to move it — must still
        // count as aimed, or hold-Y would never feed until pressed again.
        Outtake.SPIN_UP_MS = 0;
        double stalled = Math.toRadians(2);
        assertFalse("premise: outside the controller's deadband", AimController.onTarget(stalled));
        assertTrue(ShootingRoutines.aimedToShoot(stalled));
        assertFalse(ShootingRoutines.aimedToShoot(Math.toRadians(5)));
        assertFalse(ShootingRoutines.aimedToShoot(Double.NaN));
    }
}
