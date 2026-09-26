package org.firstinspires.ftc.teamcode.util;

/**
 * Suppresses no-op hardware writes.
 *
 * <p>Every {@code setPower()} / {@code setPosition()} is a USB/RS-485 transaction costing
 * ~2 ms whether or not the value changed. Gating on a change typically eliminates 70–90% of
 * writes (architecture doc §4.4). Usage, inside a subsystem's {@code write()}:
 *
 * <pre>{@code
 * if (powerGate.changed(power)) motor.setPower(power);
 * }</pre>
 *
 * <p>{@link #changed(double)} both tests <em>and</em> commits the value, so call it exactly
 * once per write, in the {@code if}.
 *
 * <p>No SDK imports — unit-testable as-is.
 */
public final class WriteGate {

    /** Motor power: below this the hub cannot tell the difference. */
    public static final double MOTOR_POWER = 0.015;

    /** Servo position: below servo resolution, so nothing is lost. */
    public static final double SERVO_POSITION = 0.003;

    private final double epsilon;
    private double last = Double.NaN;

    public WriteGate(double epsilon) {
        this.epsilon = epsilon;
    }

    /**
     * @return true if {@code value} should be written to hardware, in which case it is now
     *         recorded as the last written value.
     */
    public boolean changed(double value) {
        // The exact-zero transition is always written: a stop command must land even when it
        // is within epsilon of the last power.
        boolean write = Double.isNaN(last)
                || Math.abs(value - last) > epsilon
                || (value == 0.0) != (last == 0.0);
        if (write) last = value;
        return write;
    }

    /** Forces the next {@link #changed(double)} to write — use after anything else may have
     *  moved the actuator (e.g. a mode change, or {@code stop()}). */
    public void invalidate() {
        last = Double.NaN;
    }

    public double lastWritten() {
        return last;
    }
}
