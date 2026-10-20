package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

/**
 * Pins {@link Outtake}'s Panels-tunable statics to the values the tests are written against,
 * and puts the team's real tuning back afterwards. Tests must not inherit the tuned values:
 * with a measured {@code TARGET_RPM} in the code the flywheel runs closed loop, and a fake
 * motor that never reaches speed would make every "spun up" wait time out.
 *
 * <pre>{@code
 * private OuttakeTuning tuning;
 * @Before public void setUp() { tuning = OuttakeTuning.pinForTests(); ... }
 * @After  public void restore() { tuning.restore(); }
 * }</pre>
 */
public final class OuttakeTuning {

    private final double spinPower, targetRpm, kv, kp, spinUpMs, atSpeedTolerance;

    private OuttakeTuning() {
        spinPower = Outtake.SPIN_POWER;
        targetRpm = Outtake.TARGET_RPM;
        kv = Outtake.KV;
        kp = Outtake.KP;
        spinUpMs = Outtake.SPIN_UP_MS;
        atSpeedTolerance = Outtake.AT_SPEED_TOLERANCE_RPM;
    }

    /** Saves the current values, then sets open loop at 0.8 with the untuned gains. */
    public static OuttakeTuning pinForTests() {
        OuttakeTuning saved = new OuttakeTuning();
        Outtake.SPIN_POWER = 0.8;
        Outtake.TARGET_RPM = 0;
        Outtake.KV = 1.0 / 6000;
        Outtake.KP = 0.0005;
        Outtake.SPIN_UP_MS = 1000;
        Outtake.AT_SPEED_TOLERANCE_RPM = 100;
        return saved;
    }

    public void restore() {
        Outtake.SPIN_POWER = spinPower;
        Outtake.TARGET_RPM = targetRpm;
        Outtake.KV = kv;
        Outtake.KP = kp;
        Outtake.SPIN_UP_MS = spinUpMs;
        Outtake.AT_SPEED_TOLERANCE_RPM = atSpeedTolerance;
    }
}
