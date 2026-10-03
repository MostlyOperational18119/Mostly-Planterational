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

import java.util.Arrays;

/** Intake on its own: no follower needed, so the loop is just execute → write. */
public class IntakeTest {

    private FakeMotor motor;
    private Intake intake;

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeMotor();
        intake = new Intake(motor.motor);
    }

    private void cycle() {
        intake.read();
        Scheduler.execute();
        intake.write();
    }

    @Test
    public void collectRunsWhileScheduledAndStopsWhenCancelled() {
        Command collect = intake.collect();
        collect.schedule();
        cycle();
        cycle();
        assertEquals(Intake.COLLECT_POWER, motor.power(), 0.0);

        collect.cancel();
        cycle();
        assertEquals(0.0, motor.power(), 0.0);
    }

    @Test
    public void ejectPreemptsCollectWithoutAZeroInBetween() {
        Command collect = intake.collect();
        collect.schedule();
        cycle();
        intake.eject().schedule();
        cycle();
        assertFalse(collect.isScheduled());
        assertEquals(Arrays.asList(Intake.COLLECT_POWER, Intake.EJECT_POWER), motor.writes());
    }

    @Test
    public void offEndsWhateverWasRunning() {
        Command collect = intake.collect();
        collect.schedule();
        cycle();
        intake.off().schedule();
        cycle();
        assertFalse(collect.isScheduled());
        assertEquals(0.0, motor.power(), 0.0);
    }

    @Test
    public void reschedulingWhileRunningKeepsRunning() {
        Command collect = intake.collect();
        collect.schedule();
        cycle();
        collect.schedule();
        cycle();
        assertTrue(collect.isScheduled());
        assertEquals(Intake.COLLECT_POWER, motor.power(), 0.0);
    }

    @Test
    public void deadlineStopsTheIntakeWhenItEnds() {
        Groups.deadline(Commands.waitUntil(() -> true), intake.collect()).schedule();
        cycle();
        cycle();
        assertEquals(0.0, intake.power(), 0.0);
        assertEquals(0.0, motor.power(), 0.0);
    }

    @Test
    public void unchangedPowerIsNotRewritten() {
        intake.collect().schedule();
        for (int i = 0; i < 50; i++) cycle();
        assertEquals(1, motor.writes().size());
    }

    @Test
    public void stopZeroesTheMotorDirectly() {
        intake.collect().schedule();
        cycle();
        intake.stop();
        assertEquals(0.0, motor.power(), 0.0);
    }
}
