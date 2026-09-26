package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.util.HubManager;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.firstinspires.ftc.teamcode.util.LynxHubs;
import org.firstinspires.ftc.teamcode.util.Subsystem;

import java.util.Arrays;
import java.util.List;

/**
 * The subsystem registry and the owner of the loop contract.
 *
 * <p>The phases are strictly ordered so that a command reading a sensor value in phase 4 sees
 * data captured in phase 2 of the same iteration, and an actuator written in phase 5 reflects
 * a decision made microseconds earlier. <b>Sensor-to-actuator latency is one loop period,
 * bounded and known</b> — the property last season's code lacked (architecture doc §4.1).
 *
 * <pre>
 *   1. startCycle()        clearBulkCache() on every hub      ~0.3 ms
 *   2. read()              all sensor reads (cache hits)      ~0.0 ms
 *   3. follower.update()   localization + drivetrain PIDF     ~0.5 ms
 *   4. Scheduler.execute() ALL decision logic, no I/O         ~0.1 ms
 *   5. write()             gated actuator writes              0–4 ms
 *   6. telemetry()         throttled to ~10 Hz                ~0.0 ms amortised
 * </pre>
 *
 * <p>{@link #cycle()} runs all six in that order, which is why OpModes call it instead of
 * spelling the phases out: the ordering <em>is</em> the design, and it should not be possible
 * to get it wrong in a new OpMode at 2 a.m. the night before a competition. It also means the
 * loop can be stepped by something that is not a {@code LinearOpMode} — see the Testing
 * section of CLAUDE.md.
 *
 * <p>{@code follower.update()} sits before {@code Scheduler.execute()} deliberately: an aim
 * solution computed from the pose in phase 4 uses a pose refreshed this cycle, not last cycle,
 * and Pedro's path callbacks fire before the scheduler, so a command scheduled by a callback
 * starts on the same iteration.
 */
public class Robot {

    private static final long TELEMETRY_PERIOD_NANOS = 100_000_000L;   // 10 Hz

    public final Follower follower;
    public final Drive drive;

    private final HubManager hubs;
    private final List<Subsystem> subsystems;
    private final LoopTimer loopTimer = new LoopTimer();

    private TelemetryManager telemetry;
    private long lastTelemetryNanos;

    /** The competition path: builds the Follower and puts every hub in MANUAL bulk caching. */
    public static Robot fromHardwareMap(HardwareMap hardwareMap) {
        return new Robot(new LynxHubs(hardwareMap), Constants.createFollower(hardwareMap));
    }

    /**
     * Hardware-free constructor. Subsystems are handed their collaborators; nothing below this
     * line ever sees a {@code HardwareMap}. That is what keeps the stack constructible off the
     * robot — {@code new Robot(HubManager.NONE, follower)}.
     */
    public Robot(HubManager hubs, Follower follower) {
        this.hubs = hubs;
        this.follower = follower;
        this.drive = new Drive(follower);
        this.subsystems = Arrays.asList((Subsystem) drive);   // add new subsystems here
    }

    public Robot withTelemetry(TelemetryManager telemetry) {
        this.telemetry = telemetry;
        return this;
    }

    // --- phases ---

    public void startCycle() {
        hubs.startCycle();
    }

    public void read() {
        // Indexed loops rather than enhanced-for: no Iterator allocation per phase per loop.
        for (int i = 0; i < subsystems.size(); i++) subsystems.get(i).read();
    }

    public void write() {
        for (int i = 0; i < subsystems.size(); i++) subsystems.get(i).write();
    }

    /** Phase 6, self-throttled to 10 Hz. Keep the SDK's own telemetry off the critical path. */
    public void telemetry() {
        if (telemetry == null) return;
        long now = System.nanoTime();
        if (now - lastTelemetryNanos < TELEMETRY_PERIOD_NANOS) return;
        lastTelemetryNanos = now;

        telemetry.addLine(loopTimer.summary());
        for (int i = 0; i < subsystems.size(); i++) subsystems.get(i).telemetry(telemetry);
        telemetry.update();
    }

    // --- the loop ---

    /** One full iteration. The only thing an OpMode's main loop should need to call. */
    public void cycle() {
        loopTimer.tick();
        startCycle();
        read();
        follower.update();
        Scheduler.execute();
        write();
        telemetry();
    }

    /**
     * One init-loop iteration: sensors and telemetry live, no commands and no actuator writes.
     * Keeping the loop shape identical to {@link #cycle()} means the loop period reported
     * during init is a real prediction of the match.
     */
    public void initCycle() {
        loopTimer.tick();
        startCycle();
        read();
        telemetry();
    }

    /** Safe state. Call after the loop exits, alongside {@code Scheduler.reset()}. */
    public void stop() {
        for (int i = 0; i < subsystems.size(); i++) subsystems.get(i).stop();
    }

    public LoopTimer loopTimer() {
        return loopTimer;
    }
}
