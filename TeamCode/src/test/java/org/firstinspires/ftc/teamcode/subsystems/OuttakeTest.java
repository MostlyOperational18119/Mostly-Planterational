package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.sim.FakeMotor;
import org.junit.Before;
import org.junit.Test;

/** Outtake on its own: no follower needed, so the loop is just execute → write. */
public class OuttakeTest {

    private FakeMotor motor;
    private Outtake outtake;

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeMotor();
        outtake = new Outtake(motor.motor);
    }

    private void cycle() {
        outtake.read();
        Scheduler.execute();
        outtake.write();
    }

    @Test
    public void spinRunsAtFullPowerUntilCancelled() {
        Command spin = outtake.spin();
        spin.schedule();
        for (int i = 0; i < 50; i++) cycle();
        assertEquals(1.0, motor.power(), 0.0);
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
}
