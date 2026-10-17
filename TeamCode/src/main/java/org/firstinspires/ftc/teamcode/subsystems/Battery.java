package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.Subsystem;

import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

/**
 * Battery voltage, for subsystems that compensate for sag. A motor's speed at a given power is
 * proportional to the voltage behind it, and the battery sags whenever something else draws
 * current — the intake, the drivetrain — so an uncompensated flywheel slows down exactly when
 * the robot is busiest. Read-only: no commands, no requirements, writes nothing.
 *
 * <p><b>Sampled, not read every loop.</b> The battery voltage is not in the hub's bulk data:
 * {@code VoltageSensor.getVoltage()} is a separate hub transaction ({@code LynxGetADCCommand},
 * checked in the SDK 12.0 classes) — the kind of read the loop budget does not allow per cycle.
 * So {@link #read()} samples once every {@link #SAMPLE_PERIOD_MS} and holds the value between;
 * sag from a mechanism lasts far longer than that. The sample's cost shows in the {@code read}
 * max of {@code Robot.DEBUG_TIMING}.
 */
public class Battery implements Subsystem {

    /** The voltage compensated powers are expressed at. */
    public static final double NOMINAL_VOLTS = 12.0;

    public static final long SAMPLE_PERIOD_MS = 100;

    /**
     * Below this a reading is taken as a glitch, not the battery — compensating for it would
     * drive everything at full power — and the last good value is kept.
     */
    private static final double MIN_PLAUSIBLE_VOLTS = 7.0;

    private final DoubleSupplier sensor;
    private final LongSupplier clockMs;

    // --- refreshed in read() ---
    private double volts = NOMINAL_VOLTS;
    private boolean sampled;
    private long lastSampleMs;

    /** Both hubs see the same battery, so the first voltage sensor (the Control Hub) does. */
    public static Battery fromHardwareMap(HardwareMap hardwareMap) {
        VoltageSensor sensor = hardwareMap.voltageSensor.iterator().next();
        return new Battery(sensor::getVoltage);
    }

    public Battery(DoubleSupplier sensor) {
        this(sensor, System::currentTimeMillis);
    }

    /** @param clockMs injectable so tests can step the sample period by hand. */
    public Battery(DoubleSupplier sensor, LongSupplier clockMs) {
        this.sensor = sensor;
        this.clockMs = clockMs;
    }

    @Override
    public void read() {
        long now = clockMs.getAsLong();
        if (sampled && now - lastSampleMs < SAMPLE_PERIOD_MS) return;
        sampled = true;
        lastSampleMs = now;
        double v = sensor.getAsDouble();
        if (v >= MIN_PLAUSIBLE_VOLTS) volts = v;
    }

    @Override
    public void write() {}

    @Override
    public void telemetry(TelemetryManager t) {
        t.addLine(String.format("battery: %.2f V", volts));
    }

    @Override
    public void stop() {}

    // --- queries: safe in phase 4 ---

    /** The last good sample; {@link #NOMINAL_VOLTS} before the first. */
    public double volts() {
        return volts;
    }
}
