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

/**
 * Transfer on its own, and with the Outtake as DriveTeleOp's launch binding pairs them: no
 * follower needed, so the loop is just execute → write.
 */
public class TransferTest {

    private FakeMotor transferMotor;
    private FakeMotor outtakeMotor;
    private Transfer transfer;
    private Outtake outtake;

    @Before
    public void setUp() {
        Scheduler.reset();
        transferMotor = new FakeMotor();
        outtakeMotor = new FakeMotor();
        transfer = new Transfer(transferMotor.motor);
        outtake = new Outtake(outtakeMotor.motor);
    }

    private void cycle() {
        transfer.read();
        outtake.read();
        Scheduler.execute();
        transfer.write();
        outtake.write();
    }

    @Test
    public void feedRunsUntilCancelled() {
        Command feed = transfer.feed();
        feed.schedule();
        for (int i = 0; i < 50; i++) cycle();
        assertEquals(Transfer.FEED_POWER, transferMotor.power(), 0.0);
        assertEquals("unchanged power must not be rewritten", 1, transferMotor.writes().size());

        feed.cancel();
        cycle();
        assertEquals(0.0, transferMotor.power(), 0.0);
    }

    @Test
    public void reversePreemptsFeedWithoutAZeroInBetween() {
        transfer.feed().schedule();
        cycle();
        Command reverse = transfer.reverse();
        reverse.schedule();
        cycle();
        assertEquals(Transfer.REVERSE_POWER, transferMotor.power(), 0.0);
        assertEquals(2, transferMotor.writes().size());

        reverse.cancel();
        cycle();
        assertFalse(transfer.running());
    }

    @Test
    public void offEndsFeed() {
        Command feed = transfer.feed();
        feed.schedule();
        cycle();
        transfer.off().schedule();
        cycle();
        assertFalse(feed.isScheduled());
        assertEquals(0.0, transferMotor.power(), 0.0);
    }

    @Test
    public void stopZeroesTheMotorDirectly() {
        transfer.feed().schedule();
        cycle();
        transfer.stop();
        assertEquals(0.0, transferMotor.power(), 0.0);
    }

    @Test
    public void launchTogglesFlywheelAndTransferTogether() {
        // DriveTeleOp's Y binding: one command over both, toggled.
        Command launch = Groups.parallel(outtake.spin(), transfer.feed());
        launch.schedule();
        cycle();
        assertEquals(Outtake.SPIN_POWER, outtakeMotor.power(), 0.0);
        assertEquals(Transfer.FEED_POWER, transferMotor.power(), 0.0);

        launch.cancel();
        cycle();
        assertEquals(0.0, outtakeMotor.power(), 0.0);
        assertEquals(0.0, transferMotor.power(), 0.0);

        launch.schedule();
        cycle();
        assertTrue("a cancelled launch must be reschedulable", outtake.running());
        assertTrue("a cancelled launch must be reschedulable", transfer.running());
    }

    @Test
    public void reverseInterruptsLaunchEntirely() {
        // Ivy requirements are per group: taking the transfer ends the whole launch, so the
        // flywheel stops too rather than spinning on alone.
        Command launch = Groups.parallel(outtake.spin(), transfer.feed());
        launch.schedule();
        cycle();
        transfer.reverse().schedule();
        cycle();
        assertFalse(launch.isScheduled());
        assertEquals(0.0, outtakeMotor.power(), 0.0);
        assertEquals(Transfer.REVERSE_POWER, transferMotor.power(), 0.0);
    }

    @Test
    public void deadlineCutsPowerWhenItEnds() {
        Groups.deadline(Commands.waitUntil(() -> true), transfer.feed()).schedule();
        cycle();
        cycle();
        assertEquals(0.0, transferMotor.power(), 0.0);
    }
}
