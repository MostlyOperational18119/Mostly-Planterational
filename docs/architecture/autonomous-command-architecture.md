# BIOBUZZ Autonomous Architecture — Ivy + Pedro Pathing

**Team:** BIOBUZZ · **Season:** FTC 2026–27 · **Status:** Design, not yet implemented
**Targets:** `com.pedropathing:ivy:1.0.0`, `com.pedropathing:ftc:2.1.2`, FTC SDK 11.2.1

---

## 1. The problem this design exists to solve

Last season every mechanism other than the drivetrain moved *synchronously*. In a
`LinearOpMode` that looks like:

```java
follower.followPath(toScoringPose);
while (follower.isBusy()) { follower.update(); }   // drivetrain moves, nothing else does

lift.setTargetPosition(HIGH);
lift.setPower(1.0);
while (lift.isBusy()) { }                          // ← nothing else runs. At all.
sleep(300);                                        // ← nor here
turret.setPosition(aimAngle);
sleep(250);                                        // ← nor here
```

Two costs compound here, and the second is the one that wrecked aiming:

1. **Wall-clock cost.** Mechanism time and drive time add instead of overlapping. A
   1.2 s lift extension that could have happened *during* a 1.5 s drive instead makes
   the routine 1.2 s longer, every cycle, every match.

2. **Control-loop starvation — the real aiming killer.** While that `while (lift.isBusy())`
   spins, `follower.update()` is **not** being called. Pedro's localizer integrates pose
   from encoder/odometry deltas and its PIDFs act on error computed at update time. Stop
   calling `update()` for 300 ms and you get: stale pose, a heading controller that
   integrates nothing, and a turret aiming at a pose estimate that is a third of a second
   old. Any aim solution computed from `follower.getPose()` inherits that staleness
   directly. The robot was not aiming badly — it was aiming *late*, at where it used to be.

The fix is structural, not a matter of tuning: **the control loop must never block, and
every actuator must be commanded from the same non-blocking loop iteration that produced
the sensor data it depends on.** Ivy gives us the composition primitives to express that;
this document specifies the runtime around Ivy that makes the latency deterministic.

### Design goals, in priority order

| # | Goal | Measurable target |
|---|------|-------------------|
| 1 | No blocking call ever executes on the control thread | Zero `sleep`/`isBusy()` spin outside init |
| 2 | Deterministic, short loop period | Median ≤ 6 ms, p99 ≤ 12 ms, max ≤ 20 ms |
| 3 | Sensor→actuator latency bounded to one cycle | Aim solution uses pose from *this* cycle |
| 4 | Mechanism motion overlaps drivetrain motion by default | Serial motion is opt-in, not accidental |
| 5 | Auto degrades instead of hanging when something fails | Every command has a timeout or escape |

---

## 2. Runtime model — what Ivy actually does

This section is derived from reading the Ivy 1.0.0 sources, not the docs. It matters
because several of the design decisions below are direct consequences.

`Scheduler` is a **static, single-threaded, cooperative** dispatcher. `Scheduler.execute()`
walks every running command once, calling `execute()`, then `done()`, then `end()` if
finished; then it promotes queued and suspended commands whose requirements are free.
There is no thread, no timer, no preemption. **A command's `execute()` gets exactly one
turn per loop iteration, and if it blocks, the entire robot blocks.**

Consequences we build on:

- **Latency is quantised to the loop period.** Every millisecond added to the loop is a
  millisecond of extra latency on *every* control path simultaneously. Loop period is
  therefore the single number that governs aim quality. Section 4 is entirely about it.
- **Requirements (`Set<Object>`) are the mutual-exclusion mechanism.** A command holds its
  requirement tokens while running; a conflicting schedule is resolved by `priority()` and
  the three behaviour enums. We use the subsystem instance itself as the token.
- **`Scheduler` is not thread-safe and holds static state.** `Scheduler.reset()` is
  mandatory at the top of `runOpMode()`. Nothing off the control thread may call
  `schedule()` / `cancel()` — this constrains the vision design in §7.
- **Scheduling from inside a command is safe.** `execute()` iterates a defensive copy of
  the running deque, so a command scheduled mid-cycle gets `start()` immediately and its
  first `execute()` next cycle.
- **`Scheduler.execute()` allocates** (a fresh `ArrayDeque` per call, plus streams inside
  the group classes). At our command counts this is on the order of microseconds — two
  orders of magnitude below a single hardware read — so it is *not* worth contorting the
  design around. It is worth not scheduling hundreds of commands.

### 2.1 Verified caveats in Ivy 1.0.0

These are real behaviours of the released library, confirmed in source. Code written
against this design must account for them.

| Caveat | Impact | Our mitigation |
|---|---|---|
| `PedroCommands.follow/hold/turnTo` declare **no requirements** (`Follow`, `Hold`, `Turn` never call `requiring(...)`) | Two path commands can run concurrently and stomp each other's `followPath()` calls with no conflict detected | Every Pedro command is wrapped by `Drive` (§5.1), which appends `.requiring(this)` |
| `Command.proxy()` sets `done = () -> Scheduler.isScheduled(this)` — inverted; the proxy reports done the instant the inner command *is* scheduled | A `proxy()`'d command finishes immediately instead of tracking the inner command | Do not use `proxy()`. Use `Fork.of(...)` (§5.4) |
| `Parallel`, `Race`, `Deadline` store children in a `HashMap` | Child `execute()` order within a cycle is non-deterministic | Harmless under our read/compute/write split (§4.2) — only the *last* value written per actuator matters, and writes happen after all commands run. Do not rely on ordering |
| `Race.execute()` `break`s on the first finished child | Children later in iteration order miss one `execute()` on the terminating cycle | Never put a side-effecting final action in a `race` child; put it after the race |
| `Sequential.end(SUSPENDED)` and `Repeat.end(SUSPENDED)` call `commands.get(index)` unguarded | `IndexOutOfBoundsException` if suspended exactly when `index == size` | Do not set `InterruptedBehavior.SUSPEND` on a `sequential`/`repeat` group. Suspend leaf commands only |
| `Scheduler` state is static | Commands leak between OpModes | `Scheduler.reset()` first line of `runOpMode()` |

---

## 3. Layered structure

```
┌──────────────────────────────────────────────────────────────────┐
│ OpMode          AutoBlueLeft, AutoRedRight, TeleOp               │
│                 Owns nothing. Wires a Robot to a routine.        │
├──────────────────────────────────────────────────────────────────┤
│ Routines        Auto sequences: sequential/parallel/deadline      │
│                 trees of commands. Pure composition, no hardware. │
├──────────────────────────────────────────────────────────────────┤
│ Commands        Factory methods ON subsystems: lift.goTo(HIGH)    │
│                 Each declares .requiring(subsystem).              │
├──────────────────────────────────────────────────────────────────┤
│ Subsystems      Lift, Turret, Intake, Drive.                      │
│                 Own hardware. Expose read()/write() + factories.  │
│                 NEVER touch hardware outside read()/write().      │
├──────────────────────────────────────────────────────────────────┤
│ Robot           Subsystem registry + the loop contract.           │
│                 Bulk-cache control, read-all / write-all, timing. │
├──────────────────────────────────────────────────────────────────┤
│ Hardware        DcMotorEx, Servo, LynxModule, Follower, Limelight │
└──────────────────────────────────────────────────────────────────┘
```

The rule that makes latency analysable: **dependencies point downward only, and hardware
I/O happens exclusively in the `Robot` phase boundaries.** A command never calls
`motor.setPower()`; it sets a *setpoint field* on its subsystem, and the subsystem's
`write()` flushes setpoints to hardware once per loop.

---

## 4. The loop — where latency is won or lost

### 4.1 The contract

```java
// AutoOpMode.runOpMode(), after waitForStart()
while (opModeIsActive()) {
    robot.startCycle();      // 1. clearBulkCache() on every hub  — ~0.3 ms
    robot.read();            // 2. all sensor reads (cache hits)  — ~0.0 ms
    follower.update();       // 3. localization + drivetrain PIDF — ~0.5 ms
    Scheduler.execute();     // 4. ALL decision logic, no I/O     — ~0.1 ms
    robot.write();           // 5. gated actuator writes          — 0–4 ms
    robot.telemetry();       // 6. throttled to ~10 Hz            — ~0.0 ms amortised
}
```

Phases 2–5 are strictly ordered so that a command reading a sensor value in phase 4 sees
data captured in phase 2 *of the same iteration*, and an actuator written in phase 5
reflects a decision made microseconds earlier. **Sensor-to-actuator latency is one loop
period, bounded and known.** That is the property last year's code lacked.

`follower.update()` sits before `Scheduler.execute()` deliberately: aim solutions computed
from `follower.getPose()` in phase 4 then use a pose refreshed this cycle, not last cycle.
It also means Pedro's parametric path callbacks (§6.2) fire *before* the scheduler runs,
so a command scheduled by a callback starts on the same iteration.

### 4.2 Latency budget

Measured costs on a Control Hub, for calibration. The dominant term is not computation —
it is hardware round-trips.

| Source | Cost | Handling |
|---|---|---|
| Un-cached `getCurrentPosition()` / `getVelocity()` | **~2 ms each** | `BulkCachingMode.MANUAL`, one `clearBulkCache()` per hub per loop. 10 reads: 20 ms → 0.3 ms |
| `setPower()` / `setPosition()` on unchanged value | **~2 ms each** | Epsilon-gated writes (§4.4). Typically eliminates 70–90% of writes |
| Reads from the **second** hub (expansion) | ~2 ms + cost | Put latency-critical sensors (odometry, turret encoder) on the **Control Hub** |
| I2C sensor (colour, distance, non-Pinpoint IMU) | 3–8 ms | Never on the control thread. Poll on a background thread and publish through a `volatile` field, the way the Limelight driver does (§7.1) |
| `follower.update()` | ~0.3–0.8 ms | Once per loop, unconditionally |
| `Scheduler.execute()` at ~15 commands | < 0.2 ms | Not a concern |
| SDK `telemetry.update()` | 1–5 ms | Throttle to 10 Hz; keep Panels off the critical path |
| Limelight 3A result — `getLatestResult()` | **~0 ms** | A `volatile` field read. The camera's own processor ran the pipeline. §7 |
| Any other Limelight call (`pipelineSwitch`, `getStatus`, `updateRobotOrientation`) | 100 ms – **15 s** | Blocking HTTP. Init only, or a dedicated background thread. §7.3 |

**Budget: ≤ 6 ms median.** At 6 ms the loop runs at ~165 Hz and worst-case aim staleness is
6 ms of robot motion — under 1 cm at 1.5 m/s. At last year's effective rate (blocked for
hundreds of ms at a time) the same error was tens of centimetres. *That* is the aiming fix.

### 4.3 Bulk caching — mandatory

```java
// Robot constructor
hubs = hardwareMap.getAll(LynxModule.class);
for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

// Robot.startCycle(), once per loop
for (LynxModule hub : hubs) hub.clearBulkCache();
```

`MANUAL` (not `AUTO`) is deliberate: `AUTO` invalidates the cache on the *second* read of
the same register, silently reintroducing round-trips. `MANUAL` guarantees exactly one
bulk transfer per hub per loop, and every read in phase 2 is a memory access.

The corollary is a discipline, not just a setting: **all sensor reads happen in `read()`,
into fields.** A command that calls `lift.getMotor().getCurrentPosition()` in phase 4 still
hits cache, but it breaks the invariant that phase 4 is pure — and once one command does
it, ordering bugs become possible. Commands read `lift.position`, a field.

### 4.4 Write gating

Every hardware write is a USB/RS-485 transaction. Writing an unchanged value costs the same
as writing a new one, so we suppress no-ops:

```java
// In a subsystem's write()
protected void setPower(DcMotorEx motor, double power, double epsilon) {
    if (Math.abs(power - lastPower) > epsilon || (power == 0.0) != (lastPower == 0.0)) {
        motor.setPower(power);
        lastPower = power;
    }
}
```

Epsilons: **0.01–0.02** for motor power, **0.002–0.005** for servo position (below servo
resolution, so nothing is lost). The exact-zero check ensures a stop command always lands.

This also neutralises the non-deterministic child ordering inside Ivy's `Parallel`/`Race`
groups (§2.1): if two commands both set a setpoint in phase 4, whichever wrote last defines
the field, and phase 5 flushes exactly one value. Ordering can affect *which* value wins,
so overlapping ownership of one actuator remains a design error — prevented by requirements,
not by luck.

---

## 5. Subsystems

### 5.1 The contract

Ivy has **no subsystem abstraction and no periodic hook** — unlike WPILib or FTCLib, nothing
in the library will call your subsystem every loop. We supply that; it is the missing piece
that makes the read/compute/write split enforceable.

```java
public interface Subsystem {
    /** Phase 2: copy cached hardware state into fields. No logic. */
    void read();

    /** Phase 5: flush setpoint fields to hardware, gated. No logic. */
    void write();

    /** Phase 6: optional, throttled. */
    default void telemetry(TelemetryManager t) {}

    /** Safe state on OpMode stop. */
    default void stop() {}
}
```

`Robot` holds the registry and owns the phases:

```java
public class Robot {
    public final Follower follower;
    public final Drive drive;
    public final Lift lift;
    public final Turret turret;
    public final Intake intake;
    public final Vision vision;

    private final List<Subsystem> subsystems;
    private final List<LynxModule> hubs;

    public Robot(HardwareMap hw) {
        hubs = hw.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        follower = Constants.createFollower(hw);
        drive    = new Drive(follower);
        lift     = new Lift(hw);
        turret   = new Turret(hw);
        intake   = new Intake(hw);
        vision   = new Vision(hw);
        subsystems = Arrays.asList(drive, lift, turret, intake, vision);   // Java 8: no List.of
    }

    public void startCycle() { for (LynxModule h : hubs) h.clearBulkCache(); }
    public void read()       { for (int i = 0; i < subsystems.size(); i++) subsystems.get(i).read(); }
    public void write()      { for (int i = 0; i < subsystems.size(); i++) subsystems.get(i).write(); }
}
```

Indexed loops over `List` rather than enhanced-for: avoids an `Iterator` allocation per phase
per loop. Marginal, but free.

### 5.2 A subsystem, in full

`Lift` shows the whole pattern — closed-loop control living in `write()`, commands as
factories, hardware touched in exactly two places.

```java
public class Lift implements Subsystem {
    public enum Level { STOWED(0), LOW(450), HIGH(1180); public final int ticks; ... }

    private final DcMotorEx motor;
    // com.pedropathing.control.PIDFController — API is setTargetPosition/updatePosition/run()
    private final PIDFController controller = new PIDFController(COEFFS);

    // --- state, refreshed in read() ---
    private int position;
    private double velocity;

    // --- setpoint, mutated by commands in phase 4 ---
    private int target;

    private double lastPower = Double.NaN;

    @Override public void read() {
        position = motor.getCurrentPosition();   // bulk-cached: free
        velocity = motor.getVelocity();          // bulk-cached: free
    }

    @Override public void write() {
        controller.setTargetPosition(target);
        controller.updatePosition(position);
        double power = Range.clip(controller.run() + kG, -1.0, 1.0);   // kG = gravity feedforward
        if (Math.abs(power - lastPower) > 0.015 || (power == 0.0) != (lastPower == 0.0)) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    // --- queries: pure field reads, safe in phase 4 ---
    public int position()      { return position; }
    public boolean atTarget()  { return Math.abs(target - position) < TOLERANCE; }

    // --- commands ---

    /** Sets the target and returns immediately. Does not wait. */
    public CommandBuilder setTarget(Level level) {
        return Commands.instant(() -> target = level.ticks).requiring(this);
    }

    /** Sets the target and holds until the lift arrives. Times out rather than hanging. */
    public CommandBuilder goTo(Level level) {
        return Groups.sequential(
                Commands.instant(() -> target = level.ticks),
                Commands.waitUntil(this::atTarget)
            ).requiring(this)
             .until(Timeouts.after(1500))          // never blocks the routine forever
             .requiring(this);
    }
}
```

Three properties worth naming:

- **`goTo` never blocks.** `waitUntil` returns `done()==false` each cycle and yields. The
  robot's other 14 commands run in the same iteration.
- **The lift is under closed-loop control every single cycle**, including while a *different*
  command owns it, because control lives in `write()` — not in a command's `execute()`. A
  command that ends does not drop the lift.
- **Every path out is bounded.** The `.until(Timeouts.after(1500))` decorator means a jammed
  lift costs 1.5 s, not the match.

> `.requiring(this)` is applied *after* the group decorators because `Groups`/`until` return
> a fresh `CommandBuilder` whose requirements are the union of its children's — re-declaring
> at the outermost level keeps the token attached to the command that actually gets scheduled.

### 5.3 `Drive` — wrapping Pedro

Exists for one reason: Ivy's Pedro commands declare no requirements (§2.1), so without this
wrapper two concurrently scheduled paths silently fight over the drivetrain.

```java
public class Drive implements Subsystem {
    private final Follower follower;

    public Drive(Follower follower) { this.follower = follower; }

    // follower.update() is called by the OpMode loop, not here — it must run
    // between read() and Scheduler.execute(). read()/write() stay empty.
    @Override public void read()  {}
    @Override public void write() {}

    public CommandBuilder follow(PathChain path) {
        return PedroCommands.follow(follower, path).requiring(this);
    }

    public CommandBuilder follow(PathChain path, double maxPower) {
        return PedroCommands.follow(follower, path, maxPower).requiring(this);
    }

    public CommandBuilder turnTo(double radians) {
        return PedroCommands.turnTo(follower, radians).requiring(this);
    }

    public CommandBuilder hold(Pose pose) {
        return PedroCommands.hold(follower, pose).requiring(this);
    }

    /** Fires when the follower passes t along the current path. See §6.2. */
    public CommandBuilder waitForT(double t) {
        return Commands.waitUntil(() -> follower.getCurrentTValue() >= t);
    }

    public boolean stuck() { return follower.isRobotStuck() || follower.isLocalizationNAN(); }
}
```

### 5.4 `Fork` — fire-and-forget, correctly

`Command.proxy()` is broken in 1.0.0 (§2.1). This is the intended behaviour: schedule a
command into the scheduler as an independent citizen, and finish immediately, so the parent
sequence moves on while the forked work continues under its own requirements.

```java
public final class Fork {
    /** Schedules {@code inner} independently and completes in the same cycle. */
    public static CommandBuilder of(Command inner) {
        return Commands.instant(inner::schedule);
    }

    /** Schedules {@code inner} and completes once it leaves the scheduler. */
    public static CommandBuilder untilDone(Command inner) {
        return Command.build()
                .setStart(inner::schedule)
                .setDone(() -> !Scheduler.isScheduled(inner))     // note: negated
                .setEnd(end -> { if (end != EndCondition.NATURALLY) inner.cancel(); });
    }
}
```

Use `Fork.of` sparingly — a forked command escapes its parent group's lifecycle, so an
interrupted routine will not clean it up. Prefer `parallel`/`deadline`.

---

## 6. Concurrency — the actual fix for aiming

### 6.1 Composition patterns

| Intent | Composition |
|---|---|
| Drive there, then score | `sequential(drive.follow(p), lift.goTo(HIGH), claw.release())` |
| **Drive there while raising the lift** | `parallel(drive.follow(p), lift.goTo(HIGH))` — ends when both finish |
| Drive there, raising the lift on the way; don't wait for the lift | `deadline(drive.follow(p), lift.goTo(HIGH))` — ends when the path ends |
| Intake until a game element is detected, or 2 s | `race(intake.run(), Commands.waitUntil(intake::hasElement), Timeouts.command(2000))` |
| Aim continuously in the background | `Groups.loop(turret.trackGoal())` at low priority (§6.3) |

The last-season transcription becomes:

```java
// Before: ~4.0 s, drivetrain and mechanisms strictly serial, follower starved throughout
// After:  ~2.1 s, follower updated every 6 ms for the entire routine
Groups.sequential(
    Groups.deadline(
        drive.follow(toScoringPose),                 // deadline: the path defines the duration
        Groups.sequential(
            drive.waitForT(0.55),                    // wait until 55% along the path
            lift.goTo(Lift.Level.HIGH)               // then start the lift, still driving
        )
    ),
    claw.release(),
    Groups.deadline(
        drive.follow(toIntake),
        lift.goTo(Lift.Level.STOWED)                 // retract on the way back
    )
)
```

Nothing here blocks. `follower.update()` runs at full rate for the whole sequence, the lift
overlaps both drives, and the routine is ~half the wall-clock time.

### 6.2 Two ways to overlap, and when to use each

**(a) `waitForT` inside a group** — shown above. Ivy-native, composable, testable, and
visible in the routine's structure. **This is the default.**

**(b) Pedro path callbacks** — attached at path-build time:

```java
scoringPath = follower.pathBuilder()
        .curveThrough(...)
        .addParametricCallback(0.55, () -> lift.setTarget(HIGH).schedule())
        .addTemporalCallback(200, () -> intake.stop().schedule())
        .addPoseCallback(handoffPose, () -> claw.grip().schedule(), 2.0)
        .build();
```

Callbacks fire inside `follower.update()` — phase 3, *before* `Scheduler.execute()` — so the
scheduled command starts on the same iteration with zero extra latency. `addParametricCallback`
triggers on path parameter `t`, `addTemporalCallback` on milliseconds since path start,
`addPoseCallback` on proximity to a pose.

Trade-off: callbacks are one cycle tighter and expressed in path-space, but they hide
behaviour inside path construction and bypass the group's lifecycle (the callback's command
is not cancelled if the enclosing routine is interrupted). **Use (a) unless a specific
handoff is timing-critical enough to need the single cycle**, and never let a callback
schedule something that must be cleaned up on interrupt.

Both are safe with respect to threading: `follower.update()` runs on the control thread, so
callbacks calling `schedule()` obey §2's single-thread rule.

### 6.3 Continuous aiming as a background command

Aiming is not a step in a sequence — it is a control law that should be live from `start()`
to the end of the match. Model it as an infinite, low-priority command holding the turret:

```java
public CommandBuilder trackGoal() {
    return Commands.infinite(() -> {
            Pose robot = follower.getPose();                    // refreshed this cycle
            Vector vel  = follower.getVelocity();
            aimTarget   = AimSolver.solve(robot, vel, GOAL_POSE); // pure math, no I/O
        })
        .requiring(this)
        .setPriority(0)                                          // yields to everything
        .setInterruptedBehavior(InterruptedBehavior.SUSPEND);    // auto-resumes when free
}
```

Scheduled once, right after `waitForStart()`. Because it computes a *setpoint field* and
`write()` closes the loop, the turret tracks continuously — through paths, through scoring,
through everything. When a scoring routine needs the turret at a fixed angle it schedules
`turret.holdAt(angle)` at priority 10; Ivy interrupts the tracker (`SUSPEND`), and the
scheduler automatically resumes it the moment the turret requirement frees up. No manual
state machine, no re-scheduling logic.

The feed-forward on `follower.getVelocity()` is what closes the remaining gap: it aims at
where the robot *will* be one cycle from now, cancelling the last 6 ms of latency.

---

## 7. Vision — the Limelight 3A does the seeing

**Vision does not run on the Control Hub.** We use a **Limelight 3A** smart camera: it carries
its own processor, runs the AprilTag / neural pipeline on-board, and hands the Control Hub a
finished answer over Ethernet-over-USB. The 10–30 ms of frame processing that would otherwise
have to be hidden on a background thread — competing with our loop for the same four CPU
cores — happens on a different computer entirely.

That is a straight win for the loop budget. It also introduces exactly one new way to destroy
it, described in §7.3.

### 7.1 What the SDK driver already does for us

`com.qualcomm.hardware.limelightvision.Limelight3A` (SDK 11.2.1) is not a passive wrapper.
Verified in the SDK sources:

- `start()` schedules `updateLatestResult` on the driver's **own single-thread
  `ScheduledExecutorService`**, at `pollIntervalMs` — default **10 ms (100 Hz)**.
- That thread issues a blocking HTTP GET to `/results`, parses the JSON, and stores the
  outcome in a `volatile LLResult latestResult`.
- `getLatestResult()` **only reads that volatile field.** No HTTP, no lock, no allocation.

So the publish/consume split we would otherwise have hand-built with an `AtomicReference` is
already built, correctly, inside the driver. Two consequences:

1. **We do not create a vision thread.** Writing one would duplicate the driver's and add
   contention for nothing.
2. **§2's threading rule is satisfied for free.** The driver's poll thread never calls into
   our code — it sets a field. It cannot touch `Scheduler`, so it cannot violate the
   single-thread constraint.

`setPollRateHz(int)` must be called **before** `start()` — it early-returns while the poller is
RUNNING. The 100 Hz default already exceeds the camera's frame rate; leave it alone unless
profiling says otherwise.

### 7.2 The subsystem

`Vision` is an ordinary subsystem: it samples in `read()`, writes nothing, and exposes fields
that stay stable for the whole cycle.

```java
public class Vision implements Subsystem {
    private static final long MAX_STALENESS_MS = 100;

    private final Limelight3A limelight;

    // --- refreshed in read(), stable for the entire cycle ---
    private LLResult result;
    private boolean  usable;
    private Pose     fieldPose;

    public Vision(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);   // must precede start()
        limelight.pipelineSwitch(0);    // blocking HTTP — init only, never in the loop
        limelight.start();              // spins up the driver's own poll thread
    }

    /** Phase 2: one volatile read plus a few getters on an already-parsed object. No I/O. */
    @Override public void read() {
        result = limelight.getLatestResult();
        usable = result != null
              && result.isValid()
              && result.getStaleness() < MAX_STALENESS_MS
              && result.getBotposeTagCount() > 0;
        if (usable) fieldPose = toPedroPose(result.getBotpose());   // MegaTag1; see §7.3
    }

    @Override public void write() {}

    @Override public void stop() { limelight.stop(); }

    public boolean  hasFix()    { return usable; }
    public Pose     fieldPose() { return fieldPose; }
    public LLResult raw()       { return result; }
    /** True while the driver has had a reply within 250 ms. Local clock arithmetic, free. */
    public boolean  online()    { return limelight.isConnected(); }
}
```

Gate on all four conditions, not just `isValid()`. A stale-but-valid result is the exact
failure this architecture exists to eliminate, and `getStaleness()` is the cheapest guard
we have against it.

### 7.3 The blocking-call hazard — read this before enabling MegaTag2

`getLatestResult()` is free. **Almost every other method on `Limelight3A` is a synchronous
HTTP round-trip on the calling thread.**

| Method | Underlying call | Cost if called on the control thread |
|---|---|---|
| `getLatestResult()` | `volatile` field read | **~0 ms — safe.** Call once per cycle |
| `isConnected()`, `getTimeSinceLastUpdate()` | local clock arithmetic | **~0 ms — safe** |
| `pipelineSwitch(int)`, `reloadPipeline()` | HTTP GET | 100 ms connect timeout. **Init only** |
| `getStatus()` | HTTP GET | up to 100 ms read timeout. **Init or a throttled off-thread call** |
| `updateRobotOrientation(double)` | **HTTP POST** | 100 ms connect + **15 000 ms read timeout** |
| `captureSnapshot`, `uploadPipeline`, `uploadFieldmap`, `uploadPython` | HTTP POST | seconds. **Init or never** |

**`updateRobotOrientation()` on the control thread would be a worse blocking bug than anything
in last season's code.** Its read timeout is fifteen seconds: one unlucky packet stalls the
robot for over half the autonomous period. And it is required *per frame* for MegaTag2, so the
naive integration is exactly the catastrophic one.

Two acceptable options:

**(a) MegaTag1 — the default.** Use `getBotpose()` and gate on `getBotposeTagCount() >= 2`.
No yaw feed, no POST, no risk. Slightly worse single-tag accuracy, which our gating discards
anyway.

**(b) MegaTag2 with an off-thread yaw pusher.** Only if MT1 proves insufficient on the field:

```java
// Owned by Vision. Touches no subsystem and never calls Scheduler — see §2.
private final ScheduledExecutorService yawPusher =
        Executors.newSingleThreadScheduledExecutor();
private volatile double yawDeg;      // written in read(), read by the pusher

private void startYawPusher() {
    yawPusher.scheduleAtFixedRate(
            () -> limelight.updateRobotOrientation(yawDeg),
            0, 50, TimeUnit.MILLISECONDS);          // 20 Hz is plenty
}
```

`scheduleAtFixedRate` will not overlap runs, so a slow POST makes the pusher fall behind
rather than pile up — the correct failure mode. Shut it down in `stop()`.

### 7.4 Timestamps — the domain trap

A detection describes where the robot was when the *shutter* fired, which is tens of
milliseconds before we read it. Fusing it into a current pose without correcting for that
reintroduces exactly the staleness this whole design removes. Getting the correction right
means getting the clock right, and the SDK makes that easy to get wrong:

- `LLResult.getControlHubTimeStamp()` is `System.currentTimeMillis()` **captured when the
  driver's poll thread parsed the reply** — not when the frame was captured.
- `getControlHubTimeStampNanos()` is simply that value `* 1_000_000`. Nanosecond *units*,
  millisecond *resolution*, and still in the **wall-clock domain** — it is **not**
  `System.nanoTime()`. Comparing the two yields an offset of the device's uptime and a
  silently nonsensical correction.
- `getCaptureLatency()` and `getTargetingLatency()` are camera-side milliseconds, so:

```java
long   parsedAtMs = result.getControlHubTimeStamp();
double pipelineMs = result.getCaptureLatency() + result.getTargetingLatency();
long   captureMs  = parsedAtMs - (long) pipelineMs;   // wall-clock ms, ±1 poll interval
```

Pedro's `follower.getPoseHistory()` is still unusable for the lookup — in 2.1.2 `PoseHistory`
exposes only `getXPositionsArray()` / `getYPositionsArray()` for dashboard drawing, with no
timestamps. So we keep our own ring buffer, **keyed in the same wall-clock domain**:

```java
// PoseBuffer: fixed-size, allocation-free, ~200 entries ≈ 1.2 s at 6 ms.
// Keyed on currentTimeMillis to match the Limelight timestamp domain — NOT nanoTime.
poseBuffer.record(follower.getPose(), System.currentTimeMillis());   // in Robot.read()

// later, on the control thread
Pose atCapture = poseBuffer.poseAt(captureMs);
if (atCapture != null) {                                   // null ⇒ fell off the buffer
    Pose corrected = follower.getPose().plus(vision.fieldPose().minus(atCapture));
    follower.setPose(corrected);
}
```

If `poseAt` falls off the end of the buffer, **discard the detection** rather than fusing a
guess. Sizing at roughly twice the worst observed `getStaleness()` is enough.

### 7.5 What vision is and isn't allowed to do

- Vision corrects **localization**. The aim solver always runs on `follower.getPose()`, never
  on a raw detection — §6.3's feed-forward depends on a pose that is current, not one that is
  60 ms old with a correction bolted on.
- Vision **never schedules commands.** It exposes fields; routines and the aim solver read them.
- Fuse sparingly — gate on `hasFix()`, and prefer fusing while the robot is slow. Correcting
  aggressively mid-path fights Pedro's odometry and shows up as heading chatter.
- If `online()` goes false mid-match, the routine continues on odometry alone. Losing the
  camera must cost accuracy, not the match.

---

## 8. Auto routine structure

### 8.1 Build paths at init, never during the match

`PathBuilder.build()` runs Bezier and arc-length math and allocates. Doing it mid-routine
costs milliseconds inside a loop budgeted at six. Every `PathChain` is constructed in a
`Paths` class before `waitForStart()`.

```java
public final class Paths {
    public final PathChain startToScore, scoreToIntake, intakeToScore, park;

    public Paths(Follower f, Alliance alliance) {
        startToScore = f.pathBuilder().curveThrough(...).build();
        ...
    }
}
```

When a path genuinely depends on runtime state (which of three randomisation positions), build
**all candidates at init** and select with `Commands.branch` / `Commands.match`:

```java
EnumMap<Position, Command> cases = new EnumMap<>(Position.class);   // Java 8: no Map.of
cases.put(Position.LEFT,   drive.follow(paths.leftSpike));
cases.put(Position.CENTER, drive.follow(paths.centerSpike));
cases.put(Position.RIGHT,  drive.follow(paths.rightSpike));

Commands.match(() -> detectedPosition, cases)
```

Reserve `Commands.lazy` for paths whose geometry cannot be enumerated ahead of time (e.g.
built from a vision-derived pose); accept the build cost on that one cycle and keep it out
of tight sections.

### 8.2 Routines are static factories

Routines contain no hardware and no state — they are pure trees. This makes them readable
as a description of the match and trivial to reorder.

```java
public final class ScoringRoutines {
    public static Command scoreThenIntake(Robot r, Paths p) {
        return Groups.sequential(
            Groups.deadline(
                r.drive.follow(p.startToScore),
                Groups.sequential(r.drive.waitForT(0.55), r.lift.goTo(HIGH))
            ),
            r.claw.release(),
            Groups.deadline(
                r.drive.follow(p.scoreToIntake),
                Groups.sequential(r.lift.goTo(STOWED), r.intake.deploy())
            ),
            r.intake.collect().until(r.intake::hasElement)
        );
    }
}
```

### 8.3 The OpMode

```java
@Autonomous(name = "Blue Left", group = "Competition")
public class AutoBlueLeft extends LinearOpMode {
    @Override public void runOpMode() {
        Scheduler.reset();                                  // MANDATORY — static state

        Robot robot  = new Robot(hardwareMap);
        Paths paths  = new Paths(robot.follower, Alliance.BLUE);
        robot.follower.setStartingPose(START_POSE);

        Command routine = Groups.sequential(
                ScoringRoutines.scoreThenIntake(robot, paths),
                ScoringRoutines.cycle(robot, paths),
                robot.drive.follow(paths.park)
        );

        // Init loop: keep vision and telemetry live, keep the loop shape identical.
        while (opModeInInit()) {
            robot.startCycle();
            robot.read();
            robot.telemetry();
        }

        robot.follower.update();
        routine.schedule();
        robot.turret.trackGoal().schedule();                // background aim, priority 0

        while (opModeIsActive()) {
            robot.startCycle();
            robot.read();
            robot.follower.update();
            Scheduler.execute();
            robot.write();
            robot.telemetry();

            if (routine.isScheduled() == false && !parked) { /* fallback: park */ }
        }

        Scheduler.reset();
        robot.stop();
    }
}
```

TeleOp is the same loop with gamepad-triggered `schedule()` calls and
`follower.setTeleOpDrive(...)` in place of path commands — the subsystems, commands, and
write-gating are shared verbatim, so anything tuned in auto behaves identically in TeleOp.

### 8.4 Failure containment

An auto that hangs scores zero; one that gives up on a sub-goal usually still parks.

- **Every wait is bounded.** `waitUntil` alone can hang forever — always pair with
  `.until(Timeouts.after(ms))` or put it in a `race` with a timeout command.
- **Watch `follower.isRobotStuck()` and `isLocalizationNAN()`** (both exposed on `Follower`).
  A supervisor command running at high priority can cancel the routine and schedule a park.
- **Global auto deadline.** Wrap the whole routine in
  `Groups.race(routine, Timeouts.command(28_000))` so the last seconds are always available
  for parking.
- **`Subsystem.stop()`** is called after the loop exits, setting motors to zero and servos to
  a safe pose.

---

## 9. Instrumentation

You cannot hold a 6 ms budget you do not measure. `Robot` keeps a rolling loop timer, and
telemetry reports **median, p95, and max** — not mean. The mean hides exactly the periodic
spikes (GC, a stray un-cached read, a telemetry flush) that break aiming.

```java
// Reported at 10 Hz
loop: med 5.2ms  p95 7.8ms  max 14.1ms | cmds 6 | lift 1180/1176 | aim 12.4°
```

Per-phase timing (`read` / `update` / `execute` / `write`) behind a `DEBUG_TIMING` flag makes
regressions attributable in one run instead of a bisect. When `max` climbs, the near-universal
causes, in order of likelihood: a sensor read that escaped `read()`, an un-gated write, a
telemetry call outside the throttle, or a path being built mid-routine.

---

## 10. Implementation order

Each step is independently testable; nothing later is required to validate anything earlier.

1. **`Subsystem`, `Robot`, bulk caching, `LoopTimer`.** Ship with one trivial subsystem and
   confirm the loop holds ≤ 6 ms median with nothing else running. This is the foundation —
   do not proceed until the number is real.
2. **`Drive` wrapper + `Timeouts` + `Fork`.** Port the existing TeleOp onto the loop contract.
3. **One real subsystem end-to-end** (`Lift`): `read`/`write`, gated writes, PIDF in `write()`,
   `goTo` command with a timeout. Verify the loop budget is unchanged.
4. **Remaining subsystems** to the same pattern.
5. **A single auto routine** using `deadline` + `waitForT` overlap. Compare wall-clock and
   logged loop period against last season's equivalent — this is where the design pays out.
6. **Background aiming** (`trackGoal`, priority/SUSPEND) once the loop period is stable.
7. **Vision** last: `Limelight3A` sampled in `read()`, staleness gating, then the
   `PoseBuffer` latency correction. Confirm the loop period is *unchanged* — if it moved,
   a blocking Limelight call reached the control thread (§7.3).

---

## 11. Rules of thumb

1. **Never block.** No `sleep()`, no `while (motor.isBusy())`, no `Thread.sleep()` on the
   control thread. Ever.
2. **Hardware only in `read()` and `write()`.** Commands mutate setpoint fields.
3. **Every command declares `.requiring(subsystem)`** — including wrapped Pedro commands,
   which do not declare their own.
4. **Every wait has a timeout.**
5. **`Scheduler.reset()` first, `Scheduler` only from the control thread.**
6. **Default to `parallel`/`deadline`; `sequential` is a deliberate choice**, made when the
   mechanism physically cannot overlap.
7. **Build paths at init.**
8. **Measure the loop every run.** A number in telemetry, every match.
9. **On the control thread, the only Limelight calls are `getLatestResult()` and
   `isConnected()`.** Everything else on that class is blocking HTTP — up to 15 seconds.

---

## References

- Ivy docs — <https://pedropathing.com/docs/ivy>
  ([commands](https://pedropathing.com/docs/ivy/what-are-commands) ·
  [requirements & priorities](https://pedropathing.com/docs/ivy/requirements-and-priorities) ·
  [builder](https://pedropathing.com/docs/ivy/command-builder) ·
  [compositions](https://pedropathing.com/docs/ivy/command-compositions) ·
  [decorators](https://pedropathing.com/docs/ivy/utilities-and-decorators) ·
  [OpMode use](https://pedropathing.com/docs/ivy/creating-opmodes) ·
  [Pedro commands](https://pedropathing.com/docs/ivy/pedro-commands))
- Pedro Pathing docs — <https://pedropathing.com/docs>
- Bulk reads — <https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html>
- Limelight docs — <https://docs.limelightvision.io/>
  ([FTC setup](https://docs.limelightvision.io/docs/docs-limelight/getting-started/ftc) ·
  [MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2))
- FTC SDK — <https://github.com/FIRST-Tech-Challenge/FtcRobotController>

*Library behaviour in §2 was verified against the `com.pedropathing:ivy:1.0.0` and
`com.pedropathing:ftc:2.1.2` sources; the `Limelight3A` behaviour in §7 was verified
against the FTC SDK 11.2.1 `Hardware` sources. Re-check §2.1 and §7.3 when upgrading.*
