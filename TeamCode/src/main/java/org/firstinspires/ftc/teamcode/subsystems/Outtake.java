package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.WriteGate;

/**
 * The outtake: a flywheel on one motor with an encoder, {@code "outtake"} in the hardware
 * configuration (goBILDA 5000 Series, SKU 5000-0002-0001).
 *
 * <p>The architecture's closed-loop subsystem (§5.2, step 3 of §10): the encoder velocity is
 * read in {@link #read()} (bulk-cached, free) and the velocity controller runs in
 * {@link #write()}, so the flywheel is regulated every cycle whichever command owns it.
 * {@link #spin()} runs for as long as it is scheduled and lets the flywheel coast down when it
 * ends, so in a routine it sits under a {@code deadline} or {@code race}:
 *
 * <pre>{@code
 * Groups.deadline(scoreSequence, outtake.spin())    // flywheel up for the whole sequence
 * }</pre>
 *
 * <p><b>Bring-up, "measure first".</b> {@link #TARGET_RPM} starts at 0, which means <em>no
 * target yet</em>: {@link #spin()} then runs open loop at {@link #SPIN_POWER}, exactly as
 * before, while telemetry shows the measured speed. To switch to velocity control, in Panels:
 * <ol>
 *   <li>Set {@link #SPIN_POWER} to 1, spin, and note the steady rpm; set {@link #KV} to
 *       1 / that rpm.</li>
 *   <li>Set {@link #TARGET_RPM} to the launch speed you want (below that free speed, so the
 *       controller has headroom to recover after a shot).</li>
 *   <li>Raise {@link #KP} until the speed recovers quickly after a shot without oscillating.</li>
 * </ol>
 * Then copy the values back into this file — Panels edits do not survive a restart.
 */
@Configurable
public class Outtake implements Subsystem {

    /** Flip this, not the power sign, if the flywheel spins backwards. */
    private static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    /**
     * Encoder counts per motor revolution, 1:1 to the flywheel. 28 is what goBILDA's geared
     * versions of this motor imply (e.g. 2786 counts at 99.5:1); <b>unverified</b> for the
     * bare 5000-0002-0001 — check by turning the wheel one revolution by hand and reading
     * the encoder.
     */
    public static final double TICKS_PER_REV = 28;

    /** Open-loop power, used while {@link #TARGET_RPM} is 0. */
    public static double SPIN_POWER = 0.8;

    /** Launch speed. 0 = not measured yet: {@link #spin()} runs open loop at {@link #SPIN_POWER}. */
    public static double TARGET_RPM = 0;

    /** Feedforward, power per rpm: 1 / free speed at power 1. Assumes ~6000 rpm — measure it. */
    public static double KV = 1.0 / 6000;

    /** Proportional, power per rpm of error. Untuned starting point. */
    public static double KP = 0.0005;

    /** Within this of {@link #TARGET_RPM} the flywheel counts as at speed. */
    public static double AT_SPEED_TOLERANCE_RPM = 100;

    private final DcMotorEx motor;
    private final WriteGate powerGate = new WriteGate(WriteGate.MOTOR_POWER);

    // --- state, refreshed in read() ---
    private double rpm;

    // --- setpoints, mutated by commands in phase 4 ---
    private boolean spinning;
    private double openLoopPower;

    // --- what write() last commanded, for telemetry and queries ---
    private double power;

    /** The competition path: the lookup and one-time motor configuration, at init. */
    public static Outtake fromHardwareMap(HardwareMap hardwareMap) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "outtake");
        motor.setDirection(DIRECTION);
        // Our controller runs in write(); the hub's own velocity PID stays out of it. The
        // encoder still counts in this mode.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // FLOAT, never BRAKE: braking a spinning flywheel dumps its energy into the gearbox.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return new Outtake(motor);
    }

    public Outtake(DcMotorEx motor) {
        this.motor = motor;
    }

    @Override
    public void read() {
        rpm = motor.getVelocity() / TICKS_PER_REV * 60;     // ticks/s, bulk-cached
    }

    @Override
    public void write() {
        power = spinning ? spinPower() : openLoopPower;
        if (powerGate.changed(power)) motor.setPower(power);
    }

    /**
     * Feedforward plus P, clipped to [0, 1]: never driven backwards, since reverse power on a
     * spinning flywheel is a brake. Above target it coasts down instead.
     */
    private double spinPower() {
        if (TARGET_RPM <= 0) return SPIN_POWER;
        double p = KV * TARGET_RPM + KP * (TARGET_RPM - rpm);
        return Math.max(0, Math.min(1, p));
    }

    @Override
    public void telemetry(TelemetryManager t) {
        String target = !spinning ? "" : TARGET_RPM <= 0 ? " (open loop)"
                : String.format(" / %.0f%s", TARGET_RPM, atSpeed() ? " AT SPEED" : "");
        t.addLine(String.format("outtake: power %.2f | %.0f rpm%s", power, rpm, target));
    }

    @Override
    public void stop() {
        spinning = false;
        openLoopPower = 0;
        power = 0;
        powerGate.invalidate();
        motor.setPower(0);
    }

    // --- queries: safe in phase 4 ---

    /** The power {@link #write()} last sent. */
    public double power() {
        return power;
    }

    public double rpm() {
        return rpm;
    }

    public boolean running() {
        return spinning || openLoopPower != 0;
    }

    /** Spinning under velocity control and within {@link #AT_SPEED_TOLERANCE_RPM} of target. */
    public boolean atSpeed() {
        return spinning && TARGET_RPM > 0 && Math.abs(rpm - TARGET_RPM) < AT_SPEED_TOLERANCE_RPM;
    }

    /**
     * Whether feeding now makes sense: {@link #atSpeed()} under velocity control. Open loop
     * there is no target to be at, so this is just "spinning" — gating on {@link #atSpeed()}
     * there would wait out the whole timeout.
     */
    public boolean readyToFeed() {
        return TARGET_RPM > 0 ? atSpeed() : spinning;
    }

    // --- commands ---

    /** Holds the flywheel at launch speed until the command ends; coasts down after. */
    public CommandBuilder spin() {
        return Command.build()
                .setStart(() -> spinning = true)
                .setDone(() -> false)
                .setEnd(endCondition -> spinning = false)
                .requiring(this);
    }

    /** Holds a raw open-loop {@code power} while scheduled; zero on any end. */
    public CommandBuilder run(double power) {
        return Command.build()
                .setStart(() -> openLoopPower = power)
                .setDone(() -> false)
                .setEnd(endCondition -> openLoopPower = 0)
                .requiring(this);
    }

    /**
     * Cuts power and, by taking the requirement, ends whatever was running the flywheel. Not
     * named {@code stop()}: that is {@link Subsystem#stop()}, the OpMode-exit safe state.
     */
    public CommandBuilder off() {
        return Commands.instant(() -> {
            spinning = false;
            openLoopPower = 0;
        }).requiring(this);
    }
}
