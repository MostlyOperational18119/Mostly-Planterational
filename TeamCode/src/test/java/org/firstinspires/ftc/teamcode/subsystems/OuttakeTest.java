package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.sim.FakeMotor;
import org.firstinspires.ftc.teamcode.util.ShotMap;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** Outtake on its own: no follower needed, so the loop is just read → execute → write. */
public class OuttakeTest {

    private FakeMotor motor;
    private Outtake outtake;
    private double volts = Battery.NOMINAL_VOLTS;

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeMotor();
        outtake = new Outtake(motor.motor, () -> volts);
    }

    @After
    public void restoreTuning() {
        // Panels-tunable statics: a test that sets them must not leak into the next.
        Outtake.TARGET_RPM = 0;
        Outtake.KV = 1.0 / 6000;
        Outtake.KP = 0.0005;
    }

    /** Sets what the encoder reports, in flywheel rpm. */
    private void measure(double rpm) {
        motor.setVelocity(rpm / 60 * Outtake.TICKS_PER_REV);
    }

    private void cycle() {
        outtake.read();
        Scheduler.execute();
        outtake.write();
    }

    @Test
    public void spinRunsAtSpinPowerUntilCancelled() {
        Command spin = outtake.spin();
        spin.schedule();
        for (int i = 0; i < 50; i++) cycle();
        assertEquals(Outtake.SPIN_POWER, motor.power(), 0.0);
        assertEquals("unchanged power must not be rewritten", 1, motor.writes().size());

        spin.cancel();
        cycle();
        assertEquals(0.0, motor.power(), 0.0);
    }

    @Test
    public void togglingTwiceTurnsItBackOff() {
        // The toggle DriveTeleOp's Y binding uses: schedule if idle, cancel if running.
        Command spin = outtake.spin();
        spin.schedule();
        cycle();
        assertTrue(outtake.running());
        spin.cancel();
        cycle();
        assertFalse(outtake.running());
        spin.schedule();
        cycle();
        assertTrue("a cancelled command must be reschedulable", outtake.running());
    }

    @Test
    public void offEndsSpin() {
        Command spin = outtake.spin();
        spin.schedule();
        cycle();
        outtake.off().schedule();
        cycle();
        assertFalse(spin.isScheduled());
        assertEquals(0.0, motor.power(), 0.0);
    }

    @Test
    public void deadlineCutsPowerWhenItEnds() {
        Groups.deadline(Commands.waitUntil(() -> true), outtake.spin()).schedule();
        cycle();
        cycle();
        assertEquals(0.0, motor.power(), 0.0);
    }

    @Test
    public void stopZeroesTheMotorDirectly() {
        outtake.spin().schedule();
        cycle();
        outtake.stop();
        assertEquals(0.0, motor.power(), 0.0);
    }

    @Test
    public void unsetTargetMeansOpenLoopAndReadyOnceSpinning() {
        assertFalse(outtake.readyToFeed());
        outtake.spin().schedule();
        measure(2500);
        cycle();
        assertEquals(Outtake.SPIN_POWER, motor.power(), 0.0);
        assertEquals(2500, outtake.rpm(), 1e-9);
        assertFalse("no target, so never 'at speed'", outtake.atSpeed());
        assertTrue(outtake.readyToFeed());
    }

    @Test
    public void atTargetTheFeedforwardAloneHoldsSpeed() {
        Outtake.TARGET_RPM = 3000;
        outtake.spin().schedule();
        measure(3000);
        cycle();
        assertEquals(0.5, motor.power(), 1e-9);
        assertTrue(outtake.atSpeed());
        assertTrue(outtake.readyToFeed());
    }

    @Test
    public void belowTargetPushesHarderAndIsNotReady() {
        Outtake.TARGET_RPM = 3000;
        outtake.spin().schedule();
        measure(2600);          // a shot just pulled it down
        cycle();
        assertEquals(0.5 + 0.0005 * 400, motor.power(), 1e-9);
        assertFalse(outtake.atSpeed());
        assertFalse(outtake.readyToFeed());
    }

    @Test
    public void powerIsClippedAndNeverReversed() {
        Outtake.TARGET_RPM = 3000;
        outtake.spin().schedule();
        measure(0);
        cycle();
        assertEquals("spin-up saturates", 1.0, motor.power(), 0.0);
        measure(6000);
        cycle();
        assertEquals("over speed coasts, never brakes", 0.0, motor.power(), 0.0);
    }

    @Test
    public void endingSpinCutsPowerUnderVelocityControlToo() {
        Outtake.TARGET_RPM = 3000;
        Command spin = outtake.spin();
        spin.schedule();
        measure(2000);
        cycle();
        spin.cancel();
        cycle();
        assertEquals(0.0, motor.power(), 0.0);
        assertFalse(outtake.atSpeed());
    }

    @Test
    public void spinForFollowsTheShotMapAsTheDistanceChanges() {
        Outtake mapped = new Outtake(motor.motor, () -> volts,
                new ShotMap(new double[][] {{40, 2400}, {80, 3600}}));
        double[] distance = {40};
        mapped.spinFor(() -> distance[0]).schedule();
        measure(2400);
        mapped.read();
        Scheduler.execute();
        mapped.write();
        assertEquals(2400, mapped.targetRpm(), 0.0);
        assertTrue(mapped.atSpeed());
        assertEquals("at speed, feedforward alone", Outtake.KV * 2400, motor.power(), 1e-9);

        distance[0] = 60;               // drove back: the target moves with it, same command
        mapped.read();
        Scheduler.execute();
        mapped.write();
        assertEquals(3000, mapped.targetRpm(), 1e-9);
        assertFalse("600 rpm short is not at speed", mapped.atSpeed());
        assertFalse(mapped.readyToFeed());
    }

    @Test
    public void spinForWithoutADistanceFallsBackToTargetRpmThenOpenLoop() {
        Outtake mapped = new Outtake(motor.motor, () -> volts, new ShotMap(new double[][] {{40, 2400}}));
        Command spin = mapped.spinFor(() -> Double.NaN);
        spin.schedule();
        mapped.read();
        Scheduler.execute();
        mapped.write();
        assertEquals("no distance, no TARGET_RPM: open loop", Outtake.SPIN_POWER, motor.power(), 0.0);
        assertEquals(0, mapped.targetRpm(), 0.0);
        assertTrue(mapped.readyToFeed());

        Outtake.TARGET_RPM = 3000;
        mapped.read();
        Scheduler.execute();
        mapped.write();
        assertEquals(3000, mapped.targetRpm(), 0.0);
    }

    @Test
    public void anEmptyShotMapFallsBackToTargetRpm() {
        Outtake.TARGET_RPM = 3000;
        Outtake unmapped = new Outtake(motor.motor, () -> volts, new ShotMap(new double[][] {}));
        assertEquals(3000, unmapped.rpmFor(55), 0.0);
    }

    @Test
    public void spinAtReadsItsSupplierEveryCycle() {
        double[] rpm = {3000};
        outtake.spinAt(() -> rpm[0]).schedule();
        cycle();
        assertEquals(3000, outtake.targetRpm(), 0.0);
        rpm[0] = 3300;
        cycle();
        assertEquals(3300, outtake.targetRpm(), 0.0);
    }

    @Test
    public void endingSpinClearsTheTarget() {
        Command spin = outtake.spinAt(() -> 3000);
        spin.schedule();
        cycle();
        spin.cancel();
        cycle();
        assertEquals(0, outtake.targetRpm(), 0.0);
        assertFalse(outtake.readyToFeed());
    }

    @Test
    public void openLoopPowerRisesAsTheBatterySags() {
        outtake.spin().schedule();
        cycle();
        assertEquals("at nominal, unchanged", Outtake.SPIN_POWER, motor.power(), 1e-9);
        volts = 10;             // the intake just pulled the battery down
        cycle();
        assertEquals(Outtake.SPIN_POWER * 12 / 10, motor.power(), 1e-9);
        volts = 8;
        cycle();
        assertEquals("never past full power", 1.0, motor.power(), 0.0);
    }

    @Test
    public void theFeedforwardIsCompensatedToo() {
        Outtake.TARGET_RPM = 3000;
        outtake.spin().schedule();
        measure(3000);
        volts = 13;             // fresh battery: less power for the same speed
        cycle();
        assertEquals(Outtake.KV * 3000 * 12 / 13, motor.power(), 1e-9);
    }

    @Test
    public void rawRunIsNotCompensated() {
        volts = 10;
        outtake.run(1.0).schedule();
        cycle();
        assertEquals("a measurement tool: exactly the power asked for", 1.0, motor.power(), 0.0);
        outtake.run(0.5).schedule();
        cycle();
        assertEquals(0.5, motor.power(), 0.0);
    }
}
