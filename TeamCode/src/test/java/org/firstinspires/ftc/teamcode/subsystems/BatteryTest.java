package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

/** Sampling and glitch rejection, on a hand-stepped clock. */
public class BatteryTest {

    private double reading = 12.5;
    private long nowMs = 1_000;
    private int samples;
    private final Battery battery = new Battery(() -> {
        samples++;
        return reading;
    }, () -> nowMs);

    @Test
    public void nominalBeforeTheFirstRead() {
        assertEquals(Battery.NOMINAL_VOLTS, battery.volts(), 0.0);
    }

    @Test
    public void samplesOncePerPeriodAndHoldsBetween() {
        battery.read();
        assertEquals(12.5, battery.volts(), 0.0);

        reading = 11.0;
        for (int i = 0; i < 20; i++) {
            nowMs += 2;             // 40 ms of 2 ms loops: inside the period
            battery.read();
        }
        assertEquals("held between samples", 12.5, battery.volts(), 0.0);
        assertEquals("one hub transaction, not one per loop", 1, samples);

        nowMs += Battery.SAMPLE_PERIOD_MS;
        battery.read();
        assertEquals(11.0, battery.volts(), 0.0);
        assertEquals(2, samples);
    }

    @Test
    public void anImplausibleReadingKeepsTheLastGoodOne() {
        battery.read();
        reading = 0;                // a failed read, not a dead battery
        nowMs += Battery.SAMPLE_PERIOD_MS;
        battery.read();
        assertEquals(12.5, battery.volts(), 0.0);
    }
}
