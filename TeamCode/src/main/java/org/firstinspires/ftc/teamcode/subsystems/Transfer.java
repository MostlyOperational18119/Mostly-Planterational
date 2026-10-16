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
 * The transfer: one open-loop motor, {@code "transfer"} in the hardware configuration, that
 * carries pollen up from the intake and feeds it into the {@link Outtake} flywheel from
 * underneath.
 *
 * <p>Same shape as {@link Intake} — a power setpoint that commands set and {@link #write()}
 * flushes, gated, in phase 5. {@link #feed()} and {@link #reverse()} run for as long as they
 * are scheduled and zero the motor when they end, so outside TeleOp they sit under a
 * {@code deadline}, a {@code race} or
 * {@link org.firstinspires.ftc.teamcode.util.Timeouts#limit} (rule 4):
 *
 * <pre>{@code
 * Groups.parallel(outtake.spin(), transfer.feed())     // DriveTeleOp's Y: lift and launch
 * }</pre>
 *
 * <p>Nothing here waits for the flywheel to reach speed: with the outtake open loop there is
 * no speed to wait on. An {@code Outtake.atSpeed()} query would be what gates {@link #feed()}.
 */
public class Transfer implements Subsystem {

    /**
     * REVERSE so that {@link #feed()} is a positive power. Flip this, not the power signs, if
     * the transfer runs backwards.
     */
    private static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static final double FEED_POWER = 0.6;
    public static final double REVERSE_POWER = -0.6;

    private final DcMotorEx motor;
    private final WriteGate powerGate = new WriteGate(WriteGate.MOTOR_POWER);

    // --- setpoint, mutated by commands in phase 4 ---
    private double power;

    /** The competition path: the lookup and one-time motor configuration, at init. */
    public static Transfer fromHardwareMap(HardwareMap hardwareMap) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "transfer");
        motor.setDirection(DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return new Transfer(motor);
    }

    public Transfer(DcMotorEx motor) {
        this.motor = motor;
    }

    @Override
    public void read() {
        // Open loop, no sensors yet. A pollen-present sensor would be sampled here.
    }

    @Override
    public void write() {
        if (powerGate.changed(power)) motor.setPower(power);
    }

    @Override
    public void telemetry(TelemetryManager t) {
        t.addLine(String.format("transfer: power %.2f", power));
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

    /** Carries pollen up into the flywheel until the command ends. */
    public CommandBuilder feed() {
        return run(FEED_POWER);
    }

    /** Runs the transfer back down towards the intake until the command ends. */
    public CommandBuilder reverse() {
        return run(REVERSE_POWER);
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
     * Zeroes the transfer and, by taking the requirement, ends whatever was running it. Not
     * named {@code stop()}: that is {@link Subsystem#stop()}, the OpMode-exit safe state.
     */
    public CommandBuilder off() {
        return Commands.instant(() -> power = 0).requiring(this);
    }
}
