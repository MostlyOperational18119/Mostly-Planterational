package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import org.firstinspires.ftc.teamcode.sim.FakeMotor;
import org.firstinspires.ftc.teamcode.sim.FakeServo;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;

/** Intake on its own: no follower needed, so the loop is just execute → write. */
public class IntakeTest {

    private FakeMotor motor;
    private FakeServo intakeServo;
    private FakeServo ramp;
    private Intake intake;

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeMotor();
        intakeServo = new FakeServo();
        ramp = new FakeServo();
        intake = new Intake(motor.motor, intakeServo.servo, ramp.servo);
    }

    private void cycle() {
        intake.read();
        Scheduler.execute();
        intake.write();
    }

    @Test
    public void theServosGoToTheirStartPositionsOnConstructionAndAreNotRewritten() {
        assertEquals(Arrays.asList(Intake.INTAKE_UP), intakeServo.writes());
        assertEquals(Arrays.asList(Intake.RAMP_UP), ramp.writes());
        intake.collect().schedule();
        for (int i = 0; i < 20; i++) cycle();
        assertEquals(1, intakeServo.writes().size());
        assertEquals(1, ramp.writes().size());
    }

    @Test
    public void rampCommandsMoveTheRampOnceAndFinish() {
        Command down = intake.rampDown();
        down.schedule();
        cycle();
        cycle();
        assertFalse(down.isScheduled());
        assertEquals(Arrays.asList(Intake.RAMP_UP, Intake.RAMP_DOWN), ramp.writes());

        intake.rampUp().schedule();
        cycle();
        assertEquals(Intake.RAMP_UP, ramp.position(), 0.0);
        assertEquals(3, ramp.writes().size());
    }

    @Test
    public void intakeServoCommandsSetEachPosition() {
        intake.setForNectar().schedule();
        cycle();
        assertEquals(Intake.INTAKE_NECTAR, intakeServo.position(), 0.0);
        intake.setForPollen().schedule();
        cycle();
        assertEquals(Intake.INTAKE_POLLEN, intakeServo.position(), 0.0);
        intake.intakeServoStart().schedule();
        cycle();
        assertEquals(Intake.INTAKE_UP, intakeServo.position(), 0.0);
    }

    @Test
    public void movingTheServosDoesNotEndCollect() {
        Command collect = intake.collect();
        collect.schedule();
        cycle();
        intake.rampDown().schedule();
        intake.setForPollen().schedule();
        cycle();
        assertTrue(collect.isScheduled());
        assertEquals(Arrays.asList(Intake.COLLECT_POWER), motor.writes());
        assertEquals(Intake.RAMP_DOWN, ramp.position(), 0.0);
        assertEquals(Intake.INTAKE_POLLEN, intakeServo.position(), 0.0);
    }

    @Test
    public void setFlowerLowersTheRampAndRaisesTheIntakeTogether() {
        double original = Intake.INTAKE_UP;
        try {
            Intake.INTAKE_UP = 0.5;     // off where the constructor put it, so the move shows
            Command collect = intake.collect();
            collect.schedule();
            cycle();
            Command flower = intake.setFlower();
            flower.schedule();
            cycle();
            cycle();
            assertFalse(flower.isScheduled());
            assertTrue(collect.isScheduled());
            assertEquals(Intake.RAMP_DOWN, ramp.position(), 0.0);
            assertEquals(0.5, intakeServo.position(), 0.0);
        } finally {
            Intake.INTAKE_UP = original;
        }
    }

    @Test
    public void setWallRaisesTheRampAndKeepsTheIntakeUp() {
        intake.setFlower().schedule();
        intake.setForPollen().schedule();
        cycle();
        Command collect = intake.collect();
        collect.schedule();
        cycle();
        Command wall = intake.setWall();
        wall.schedule();
        cycle();
        cycle();
        assertFalse(wall.isScheduled());
        assertTrue(collect.isScheduled());
        assertEquals(Intake.RAMP_UP, ramp.position(), 0.0);
        assertEquals(Intake.INTAKE_UP, intakeServo.position(), 0.0);
    }

    @Test
    public void servoPositionsAreClampedToTheServoRange() {
        intake.setRamp(1.4).schedule();
        intake.setIntakeServo(-0.2).schedule();
        cycle();
        assertEquals(1.0, ramp.position(), 0.0);
        assertEquals(0.0, intakeServo.position(), 0.0);
    }

    @Test
    public void aPanelsEditAppliesTheNextTimeTheCommandStarts() {
        double original = Intake.COLLECT_POWER;
        try {
            Command collect = intake.collect();
            Intake.COLLECT_POWER = 0.25;
            collect.schedule();
            cycle();
            assertEquals(0.25, motor.power(), 0.0);
        } finally {
            Intake.COLLECT_POWER = original;
        }
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
