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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Subsystem;
import org.firstinspires.ftc.teamcode.util.WriteGate;

import java.util.function.DoubleSupplier;

/**
 * The intake: one open-loop motor, {@code "intake"} in the hardware configuration, and two
 * servos, {@code "intakeServo"} and {@code "ramp"}.
 *
 * <p>The state is a power setpoint and a position setpoint per servo. Commands set them;
 * {@link #write()} flushes them, gated, in phase 5. Nothing else touches the hardware.
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
 *
 * <p>The servo commands ({@link #rampUp()}, {@link #rampDown()}, {@link #intakeServoStart()},
 * {@link #setForNectar()}, {@link #setForPollen()}, and {@link #setFlower()} /
 * {@link #setWall()}, which set both servos at once) are instant: they set the position and
 * finish, and the servo holds it. They finish as soon as the position is <em>commanded</em>, not
 * when the servo reaches it; a step that needs the servo in place should follow with a
 * {@code Commands.waitMs}. They require only the servos they move, never the Intake, so they
 * run alongside {@link #collect()}:
 *
 * <pre>{@code
 * Groups.sequential(intake.rampDown(), Groups.deadline(drive.follow(toPickup), intake.collect()))
 * }</pre>
 */
@Configurable
public class Intake implements Subsystem {

    /** Flip this, not the power signs, if the intake runs backwards. */
    private static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    // Servo positions, Panels-editable.
    /** {@code "ramp"} servo: raised, where it starts. */
    public static double RAMP_UP = 0.08;
    /** {@code "ramp"} servo: lowered. */
    public static double RAMP_DOWN = 0.8;
    /**
     * {@code "intakeServo"}: raised, for flowers ({@link #setFlower()}). <b>Placeholder:</b>
     * the start position until the up position is measured on the robot.
     */
    public static double INTAKE_UP = 0.15;
    /** {@code "intakeServo"}: set for nectar. */
    public static double INTAKE_NECTAR = 0.78;
    /** {@code "intakeServo"}: set for pollen. */
    public static double INTAKE_POLLEN = 0.9;

    // Panels-editable; a command reads them when it starts, so an edit applies on the next press.
    public static double COLLECT_POWER = 0.8;
    public static double EJECT_POWER = -0.6;

    private final DcMotorEx motor;
    private final Servo intakeServo;
    private final Servo ramp;
    private final WriteGate powerGate = new WriteGate(WriteGate.MOTOR_POWER);
    private final WriteGate intakeServoGate = new WriteGate(WriteGate.SERVO_POSITION);
    private final WriteGate rampGate = new WriteGate(WriteGate.SERVO_POSITION);

    /**
     * What the servo commands require, one per servo, instead of the Intake itself: so
     * moving a servo does not end a running {@link #collect()} (which requires the Intake),
     * and two commands for the same servo still preempt each other.
     */
    private final Object intakeServoRequirement = new Object();
    private final Object rampRequirement = new Object();

    // --- setpoints, mutated by commands in phase 4 ---
    private double power;
    private double intakeServoPosition;
    private double rampPosition;

    /** The competition path: the lookup and one-time motor configuration, at init. */
    public static Intake fromHardwareMap(HardwareMap hardwareMap) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "intake");
        Servo intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        Servo ramp = hardwareMap.get(Servo.class, "ramp");
        motor.setDirection(DIRECTION);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        return new Intake(motor, intakeServo, ramp);
    }

    /**
     * Sends both servos to their start positions ({@link #INTAKE_UP},
     * {@link #RAMP_UP}) at once, so they move to them during init rather than on the first
     * loop — construction is at init. The gates record those positions, so {@link #write()}
     * only writes a servo again when a command changes its setpoint.
     */
    public Intake(DcMotorEx motor, Servo intakeServo, Servo ramp) {
        this.motor = motor;
        this.intakeServo = intakeServo;
        this.ramp = ramp;
        intakeServoPosition = INTAKE_UP;
        rampPosition = RAMP_UP;
        if (intakeServoGate.changed(intakeServoPosition)) intakeServo.setPosition(intakeServoPosition);
        if (rampGate.changed(rampPosition)) ramp.setPosition(rampPosition);
    }

    @Override
    public void read() {
        // Open loop, no sensors yet. A game-element sensor would be sampled here.
    }

    @Override
    public void write() {
        if (powerGate.changed(power)) motor.setPower(power);
        if (intakeServoGate.changed(intakeServoPosition)) intakeServo.setPosition(intakeServoPosition);
        if (rampGate.changed(rampPosition)) ramp.setPosition(rampPosition);
    }

    @Override
    public void telemetry(TelemetryManager t) {
        t.addLine(String.format("intake: power %.2f  servo %.3f  ramp %.3f",
                power, intakeServoPosition, rampPosition));
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

    public double intakeServoPosition() {
        return intakeServoPosition;
    }

    public double rampPosition() {
        return rampPosition;
    }

    // --- commands ---

    /** Pulls game elements in until the command ends. */
    public CommandBuilder collect() {
        return run(() -> COLLECT_POWER);
    }

    /** Pushes game elements out until the command ends. */
    public CommandBuilder eject() {
        return run(() -> EJECT_POWER);
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
        return run(() -> power);
    }

    /** {@link #run(double)} with the power read each time the command starts. */
    public CommandBuilder run(DoubleSupplier power) {
        return Command.build()
                .setStart(() -> this.power = power.getAsDouble())
                .setDone(() -> false)
                .setEnd(endCondition -> this.power = 0)
                .requiring(this);
    }

    // --- servo commands: instant, each requiring only its own servo ---

    /** Raises the ramp ({@link #RAMP_UP}). */
    public CommandBuilder rampUp() {
        return setRamp(() -> RAMP_UP);
    }

    /** Lowers the ramp ({@link #RAMP_DOWN}). */
    public CommandBuilder rampDown() {
        return setRamp(() -> RAMP_DOWN);
    }

    /** Moves the ramp to {@code position}, 0–1. */
    public CommandBuilder setRamp(double position) {
        return setRamp(() -> position);
    }

    /** {@link #setRamp(double)} with the position read when the command runs. */
    public CommandBuilder setRamp(DoubleSupplier position) {
        return Commands.instant(() -> rampPosition = clamp(position.getAsDouble()))
                .requiring(rampRequirement);
    }

    /** Returns {@code "intakeServo"} to where it starts ({@link #INTAKE_UP}). */
    public CommandBuilder intakeServoStart() {
        return setIntakeServo(() -> INTAKE_UP);
    }

    /** Sets {@code "intakeServo"} for nectar ({@link #INTAKE_NECTAR}). */
    public CommandBuilder setForNectar() {
        return setIntakeServo(() -> INTAKE_NECTAR);
    }

    /** Sets {@code "intakeServo"} for pollen ({@link #INTAKE_POLLEN}). */
    public CommandBuilder setForPollen() {
        return setIntakeServo(() -> INTAKE_POLLEN);
    }

    /** Moves {@code "intakeServo"} to {@code position}, 0–1. */
    public CommandBuilder setIntakeServo(double position) {
        return setIntakeServo(() -> position);
    }

    /** {@link #setIntakeServo(double)} with the position read when the command runs. */
    public CommandBuilder setIntakeServo(DoubleSupplier position) {
        return Commands.instant(() -> intakeServoPosition = clamp(position.getAsDouble()))
                .requiring(intakeServoRequirement);
    }

    /**
     * Sets up for flowers: lowers the ramp ({@link #RAMP_DOWN}) and raises {@code "intakeServo"}
     * ({@link #INTAKE_UP}), both at once. Instant, like the other servo commands, and requires
     * both servos, so it replaces any command running either one; it does not end
     * {@link #collect()}.
     */
    public CommandBuilder setFlower() {
        return Commands.instant(() -> {
                    rampPosition = clamp(RAMP_DOWN);
                    intakeServoPosition = clamp(INTAKE_UP);
                })
                .requiring(rampRequirement, intakeServoRequirement);
    }

    /**
     * Sets up for the wall: raises the ramp ({@link #RAMP_UP}) and raises {@code "intakeServo"}
     * ({@link #INTAKE_UP}), both at once, undoing {@link #setFlower()}'s ramp. Instant and
     * requiring both servos, like {@link #setFlower()}; it does not end {@link #collect()}.
     */
    public CommandBuilder setWall() {
        return Commands.instant(() -> {
                    rampPosition = clamp(RAMP_UP);
                    intakeServoPosition = clamp(INTAKE_UP);
                })
                .requiring(rampRequirement, intakeServoRequirement);
    }

    private static double clamp(double position) {
        return Math.max(0, Math.min(1, position));
    }

    /**
     * Zeroes the intake and, by taking the requirement, ends whatever was running it. Not
     * named {@code stop()}: that is {@link Subsystem#stop()}, the OpMode-exit safe state.
     */
    public CommandBuilder off() {
        return Commands.instant(() -> power = 0).requiring(this);
    }
}
