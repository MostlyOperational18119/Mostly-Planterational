package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.function.BooleanSupplier;

import org.junit.Test;

public class TimeoutsTest {

    @Test
    public void afterStartsItsClockOnFirstPoll() {
        BooleanSupplier expired = Timeouts.after(0);
        assertFalse("first poll latches the deadline", expired.getAsBoolean());
        assertTrue(expired.getAsBoolean());
    }

    @Test
    public void afterDoesNotExpireEarly() {
        BooleanSupplier expired = Timeouts.after(60_000);
        assertFalse(expired.getAsBoolean());
        assertFalse(expired.getAsBoolean());
    }
}
