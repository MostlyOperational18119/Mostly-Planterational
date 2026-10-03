package org.firstinspires.ftc.teamcode.subsystems;

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
 * The intake: one open-loop motor, {@code "intake"} in the hardware configuration.
 *
 * <p>The only state is a power setpoint. Commands set it; {@link #write()} flushes it, gated,
 * in phase 5. Nothing else touches the motor.
 *
 * <p>{@link #collect()} and {@link #eject()} run <em>for as long as they are scheduled</em> and
 * zero the motor when they end, however they end. That makes the intake's duration a property
 * of the composition rather than of the intake:
 *
 * <pre>{@code
 * Groups.deadline(drive.follow(toPickup), intake.collect())       // runs while driving
 * Timeouts.limit(intake.eject(), 400)                              // a timed spit
 * }</pre>
 *
 * <p>They never finish on their own, so outside TeleOp they must sit under a {@code deadline},
 * a {@code race} or {@link org.firstinspires.ftc.teamcode.util.Timeouts#limit} (rule 4).
 */
public class Intake implements Subsystem {

    /** Flip this, not the power signs, if the intake runs backwards. */
    private static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static final double COLLECT_POWER = 1.0;
    public static final double EJECT_POWER = -0.6;

    private final DcMotorEx motor;
    private final WriteGate powerGate = new WriteGate(WriteGate.MOTOR_POWER);

    // --- setpoint, mutated by commands in phase 4 ---
    private double power;

    /** The competition path: the lookup and one-time motor configuration, at init. */
    public static Intake fromHardwareMap(HardwareMap hardwareMap) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setDirection(DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return new Intake(motor);
    }

    public Intake(DcMotorEx motor) {
        this.motor = motor;
    }

    @Override
    public void read() {
        // Open loop, no sensors yet. A game-element sensor would be sampled here.
    }

    @Override
    public void write() {
        if (powerGate.changed(power)) motor.setPower(power);
    }

    @Override
    public void telemetry(TelemetryManager t) {
        t.addLine(String.format("intake: power %.2f", power));
    }

    @Override
    public void stop() {
        power = 0;
        powerGate.invalidate();
        motor.setPower(0);
    }

    // --- queries: safe in phase 4 ---

    public double power() {
        return power;
    }

    public boolean running() {
        return power != 0;
    }

    // --- commands ---

    /** Pulls game elements in until the command ends. */
    public CommandBuilder collect() {
        return run(COLLECT_POWER);
    }

    /** Pushes game elements out until the command ends. */
    public CommandBuilder eject() {
        return run(EJECT_POWER);
    }

    /**
     * Holds {@code power} while scheduled; zero on any end.
     *
     * <p>Interrupt order makes handoffs clean: when {@code eject()} preempts {@code collect()},
     * the Scheduler ends {@code collect()} (power → 0) <em>before</em> starting {@code eject()}
     * (power → {@link #EJECT_POWER}), so the setpoint {@code write()} sees is the new one and
     * the motor never gets a zero in between.
     */
    public CommandBuilder run(double power) {
        return Command.build()
                .setStart(() -> this.power = power)
                .setDone(() -> false)
                .setEnd(endCondition -> this.power = 0)
                .requiring(this);
    }

    /**
     * Zeroes the intake and, by taking the requirement, ends whatever was running it. Not
     * named {@code stop()}: that is {@link Subsystem#stop()}, the OpMode-exit safe state.
     */
    public CommandBuilder off() {
        return Commands.instant(() -> power = 0).requiring(this);
    }
}
