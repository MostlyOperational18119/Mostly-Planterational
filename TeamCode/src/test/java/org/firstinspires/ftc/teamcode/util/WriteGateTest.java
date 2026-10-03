package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class WriteGateTest {

    private final WriteGate gate = new WriteGate(WriteGate.MOTOR_POWER);

    @Test
    public void firstValueIsAlwaysWritten() {
        assertTrue(gate.changed(0.0));
    }

    @Test
    public void suppressesChangesWithinEpsilon() {
        gate.changed(0.5);
        assertFalse(gate.changed(0.5));
        assertFalse(gate.changed(0.51));
        assertEquals(0.5, gate.lastWritten(), 0.0);
    }

    @Test
    public void writesChangesBeyondEpsilon() {
        gate.changed(0.5);
        assertTrue(gate.changed(0.52));
        assertEquals(0.52, gate.lastWritten(), 0.0);
    }

    @Test
    public void suppressedValuesDoNotMoveTheBaseline() {
        // Creeping by less than epsilon each call must still write once the total drift exceeds it.
        gate.changed(0.5);
        assertFalse(gate.changed(0.51));
        assertTrue(gate.changed(0.52));
    }

    @Test
    public void stopLandsEvenWithinEpsilon() {
        gate.changed(0.01);
        assertTrue(gate.changed(0.0));
        assertTrue(gate.changed(0.01));
    }

    @Test
    public void invalidateForcesNextWrite() {
        gate.changed(0.5);
        gate.invalidate();
        assertTrue(gate.changed(0.5));
    }
}
