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
 * The outtake: a flywheel on one open-loop motor, {@code "outtake"} in the hardware
 * configuration.
 *
 * <p>Same shape as {@link Intake} — a power setpoint that commands set and {@link #write()}
 * flushes, gated, in phase 5. {@link #spin()} runs for as long as it is scheduled and cuts
 * power when it ends, so in a routine it sits under a {@code deadline} or {@code race}:
 *
 * <pre>{@code
 * Groups.deadline(scoreSequence, outtake.spin())    // flywheel up for the whole sequence
 * }</pre>
 *
 * <p>Open loop for now: power 1 gives a speed that sags with battery voltage and after each
 * shot. Consistent shots will want velocity control in {@code write()} (encoder velocity read
 * in {@link #read()}, bulk-cached) and an {@code atSpeed()} query to gate feeding on.
 */
public class Outtake implements Subsystem {

    /** Flip this, not the power sign, if the flywheel spins backwards. */
    private static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final double SPIN_POWER = 1.0;

    private final DcMotorEx motor;
    private final WriteGate powerGate = new WriteGate(WriteGate.MOTOR_POWER);

    // --- setpoint, mutated by commands in phase 4 ---
    private double power;

    /** The competition path: the lookup and one-time motor configuration, at init. */
    public static Outtake fromHardwareMap(HardwareMap hardwareMap) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "outtake");
        motor.setDirection(DIRECTION);
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
        // Open loop, no sensors yet. Flywheel velocity would be sampled here.
    }

    @Override
    public void write() {
        if (powerGate.changed(power)) motor.setPower(power);
    }

    @Override
    public void telemetry(TelemetryManager t) {
        t.addLine(String.format("outtake: power %.2f", power));
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

    /** Spins the flywheel at {@link #SPIN_POWER} until the command ends; coasts down after. */
    public CommandBuilder spin() {
        return run(SPIN_POWER);
    }

    /** Holds {@code power} while scheduled; zero on any end. */
    public CommandBuilder run(double power) {
        return Command.build()
                .setStart(() -> this.power = power)
                .setDone(() -> false)
                .setEnd(endCondition -> this.power = 0)
                .requiring(this);
    }

    /**
     * Cuts power and, by taking the requirement, ends whatever was running the flywheel. Not
     * named {@code stop()}: that is {@link Subsystem#stop()}, the OpMode-exit safe state.
     */
    public CommandBuilder off() {
        return Commands.instant(() -> power = 0).requiring(this);
    }
}
