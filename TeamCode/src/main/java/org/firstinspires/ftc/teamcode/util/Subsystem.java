package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.TelemetryManager;

/**
 * The periodic hook Ivy does not provide.
 *
 * <p>Ivy has no subsystem abstraction: nothing in the library will call your hardware every
 * loop. {@link org.firstinspires.ftc.teamcode.Robot} supplies that, and this interface is the
 * contract it calls. The split is what makes sensor-to-actuator latency exactly one loop
 * period (architecture doc §4.1, §5.1).
 *
 * <p>The rule that holds the whole design together: <b>hardware I/O happens in {@link #read()}
 * and {@link #write()}, nowhere else.</b> Commands mutate setpoint fields; they never call
 * {@code motor.setPower()}.
 */
public interface Subsystem {

    /** Phase 2: copy cached hardware state into fields. No logic, no decisions. */
    void read();

    /** Phase 5: flush setpoint fields to hardware, write-gated. No decisions. */
    void write();

    /** Phase 6: optional, throttled to ~10 Hz by the {@code Robot}. */
    default void telemetry(TelemetryManager t) {}

    /** Safe state on OpMode stop. Called once, after the loop exits. */
    default void stop() {}
}
