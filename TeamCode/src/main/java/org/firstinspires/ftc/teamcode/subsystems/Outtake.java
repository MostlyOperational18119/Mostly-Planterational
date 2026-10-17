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

import org.firstinspires.ftc.teamcode.util.ShotMap;
import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.WriteGate;

import java.util.function.DoubleSupplier;

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
 * <p><b>Launch speed by distance.</b> The hood is fixed, so the flywheel speed alone decides
 * where the pollen lands. {@link #spinFor} sets the target every cycle from the distance to the
 * target through {@link #SHOT_POINTS}, a measured table. With no distance (the target not seen
 * yet) or an empty table it falls back to {@link #TARGET_RPM}; with that 0 too, it runs open loop
 * at {@link #SPIN_POWER}. So every stage of bring-up below still shoots.
 *
 * <p><b>Voltage compensated.</b> A motor's speed at a given power scales with the battery
 * voltage, and the battery sags while the intake or drivetrain draws current — enough to drop
 * shots short right after intaking. So {@link #SPIN_POWER}, {@link #KV} and {@link #KP} are
 * all "at {@link Battery#NOMINAL_VOLTS}", and {@link #write()} scales the output by nominal ÷
 * measured volts (sampled by {@link Battery}). This matters even under velocity control:
 * the feedforward does most of the work, and P alone would leave a sag-sized speed error.
 * {@link #run} is the exception — a raw power, for measuring.
 *
 * <p><b>Bring-up, "measure first"</b>, all in the {@code ShotCalibration} OpMode:
 * <ol>
 *   <li><b>Ticks per rev:</b> turn the flywheel exactly one revolution by hand and read the
 *       encoder count; fix {@link #TICKS_PER_REV} if it is not 28.</li>
 *   <li><b>Feedforward:</b> run at power 1, note the steady rpm, set {@link #KV} to 1 / that
 *       rpm.</li>
 *   <li><b>P:</b> raise {@link #KP} until the speed recovers quickly after a shot without
 *       oscillating.</li>
 *   <li><b>Shot table:</b> from several distances, aim, adjust the speed until shots score,
 *       and record the point; paste the printed points into {@link #SHOT_POINTS}. Every speed
 *       must stay below the free speed, so the controller has headroom to recover after a
 *       shot.</li>
 * </ol>
 * Copy Panels edits back into this file — they do not survive a restart.
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

    /**
     * Measured launch speeds, {@code {distanceIn, rpm}} in increasing distance — the distance is
     * the floor distance to the up CELL's tag cluster, as {@code Vision} reports it. Empty until
     * {@code ShotCalibration} has been run; paste its output here.
     */
    public static final double[][] SHOT_POINTS = {
    };

    public static final ShotMap SHOT_MAP = new ShotMap(SHOT_POINTS);

    /** Open-loop power at {@link Battery#NOMINAL_VOLTS}, used when there is no target speed. */
    public static double SPIN_POWER = 0.8;

    /**
     * Fixed launch speed: {@link #spin()}'s target, and {@link #spinFor}'s when it has no
     * distance or no table. 0 = not measured yet: run open loop at {@link #SPIN_POWER}.
     */
    public static double TARGET_RPM = 0;

    /**
     * Feedforward, power per rpm at {@link Battery#NOMINAL_VOLTS}: 1 / the free speed at power 1
     * and 12 V ({@code ShotCalibration} does the conversion). 600RPM actually works better,
     * look into more tomorrow when calibrating
     */
    public static double KV = 1.0 / 6000;

    /** Proportional, power per rpm of error. Untuned starting point. */
    public static double KP = 0.0005;

    /** Within this of {@link #TARGET_RPM} the flywheel counts as at speed. */
    public static double AT_SPEED_TOLERANCE_RPM = 100;

    private final DcMotorEx motor;
    private final DoubleSupplier volts;
    private final ShotMap shotMap;
    private final WriteGate powerGate = new WriteGate(WriteGate.MOTOR_POWER);

    // --- state, refreshed in read() ---
    private double rpm;
    private int position;

    // --- setpoints, mutated by commands in phase 4 ---
    private boolean spinning;
    private double targetRpm;           // while spinning; 0 or NaN = open loop
    private double openLoopPower;

    // --- what write() last commanded, for telemetry and queries ---
    private double power;

    /**
     * The competition path: the lookup and one-time motor configuration, at init.
     *
     * @param volts battery voltage, e.g. {@code battery::volts}
     */
    public static Outtake fromHardwareMap(HardwareMap hardwareMap, DoubleSupplier volts) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "outtake");
        motor.setDirection(DIRECTION);
        // Our controller runs in write(); the hub's own velocity PID stays out of it. The
        // encoder still counts in this mode.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // FLOAT, never BRAKE: braking a spinning flywheel dumps its energy into the gearbox.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return new Outtake(motor, volts);
    }

    public Outtake(DcMotorEx motor, DoubleSupplier volts) {
        this(motor, volts, SHOT_MAP);
    }

    /** @param shotMap injectable so tests need not depend on the measured table. */
    public Outtake(DcMotorEx motor, DoubleSupplier volts, ShotMap shotMap) {
        this.motor = motor;
        this.volts = volts;
        this.shotMap = shotMap;
    }

    @Override
    public void read() {
        rpm = motor.getVelocity() / TICKS_PER_REV * 60;     // ticks/s, bulk-cached
        position = motor.getCurrentPosition();              // bulk-cached too
    }

    @Override
    public void write() {
        power = spinning ? spinPower() : openLoopPower;
        if (powerGate.changed(power)) motor.setPower(power);
    }

    /**
     * Feedforward plus P — or {@link #SPIN_POWER} open loop — compensated for the battery and
     * clipped to [0, 1]: never driven backwards, since reverse power on a spinning flywheel is
     * a brake. Above target it coasts down instead.
     */
    private double spinPower() {
        double p = closedLoop() ? KV * targetRpm + KP * (targetRpm - rpm) : SPIN_POWER;
        p *= Battery.NOMINAL_VOLTS / volts.getAsDouble();
        return Math.max(0, Math.min(1, p));
    }

    /** {@code !(x > 0)} rather than {@code x <= 0}: a NaN target is no target. */
    private boolean closedLoop() {
        return spinning && targetRpm > 0;
    }

    @Override
    public void telemetry(TelemetryManager t) {
        String target = !spinning ? "" : !closedLoop() ? " (open loop)"
                : String.format(" / %.0f%s", targetRpm, atSpeed() ? " AT SPEED" : "");
        t.addLine(String.format("outtake: power %.2f | %.0f rpm%s", power, rpm, target));
    }

    @Override
    public void stop() {
        spinning = false;
        targetRpm = 0;
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

    /** Raw encoder count, for checking {@link #TICKS_PER_REV} by hand. */
    public int position() {
        return position;
    }

    /** The speed being held, rpm; 0 when not spinning under velocity control. */
    public double targetRpm() {
        return closedLoop() ? targetRpm : 0;
    }

    /**
     * The launch speed for a target {@code distanceIn} away: {@link #SHOT_POINTS},
     * interpolated, or {@link #TARGET_RPM} when the distance is unknown (NaN) or the table is
     * empty.
     */
    public double rpmFor(double distanceIn) {
        double fromMap = shotMap.rpmAt(distanceIn);
        return Double.isNaN(fromMap) ? TARGET_RPM : fromMap;
    }

    public boolean running() {
        return spinning || openLoopPower != 0;
    }

    /** Spinning under velocity control and within {@link #AT_SPEED_TOLERANCE_RPM} of target. */
    public boolean atSpeed() {
        return closedLoop() && Math.abs(rpm - targetRpm) < AT_SPEED_TOLERANCE_RPM;
    }

    /**
     * Whether feeding now makes sense: {@link #atSpeed()} under velocity control. Open loop
     * there is no target to be at, so this is just "spinning" — gating on {@link #atSpeed()}
     * there would wait out the whole timeout.
     */
    public boolean readyToFeed() {
        return closedLoop() ? atSpeed() : spinning;
    }

    // --- commands ---

    /** Holds the flywheel at {@link #TARGET_RPM} until the command ends; coasts down after. */
    public CommandBuilder spin() {
        return spinAt(() -> TARGET_RPM);
    }

    /**
     * Holds the flywheel at the launch speed for the target's distance, re-read every cycle, so
     * the speed follows the robot as it moves. See {@link #rpmFor}.
     *
     * @param distanceIn floor distance to the target, inches; NaN when unknown
     */
    public CommandBuilder spinFor(DoubleSupplier distanceIn) {
        return spinAt(() -> rpmFor(distanceIn.getAsDouble()));
    }

    /**
     * Holds the flywheel at {@code rpm}, re-read every cycle, until the command ends; coasts
     * down after. A value of 0 or NaN runs open loop at {@link #SPIN_POWER}.
     */
    public CommandBuilder spinAt(DoubleSupplier rpm) {
        return Command.build()
                .setStart(() -> {
                    spinning = true;
                    targetRpm = rpm.getAsDouble();
                })
                .setExecute(() -> targetRpm = rpm.getAsDouble())
                .setDone(() -> false)
                .setEnd(endCondition -> {
                    spinning = false;
                    targetRpm = 0;
                })
                .requiring(this);
    }

    /**
     * Holds a raw open-loop {@code power} while scheduled, <em>not</em> voltage compensated —
     * {@code ShotCalibration} measures the free speed with it; zero on any end.
     */
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
