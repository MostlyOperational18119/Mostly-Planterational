# CLAUDE.md

## What this repo is

The **FTC SDK** (`FtcRobotController`) plus team code for **BIOBUZZ**, competing in the
**FIRST Tech Challenge 2026–27 season**.

`FtcRobotController/`, `build.common.gradle`, `libs/`, `doc/`, and the (very large)
`README.md` come from FIRST's upstream
[FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController) repo
(currently v12.0). **Don't edit them** unless a task is specifically about
upgrading the SDK — keeping them pristine is what makes SDK updates mergeable.

All team code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── Robot.java        Subsystem registry + the loop contract (startCycle/read/update/
│                     execute/write/telemetry). OpModes call robot.cycle().
├── subsystems/       Drive.java — Pedro Follower wrapped as a requirable subsystem
│                     Intake.java — one open-loop motor ("intake"), collect/eject/off
├── util/             Subsystem.java (the periodic hook Ivy lacks)
│                     HubManager.java / LynxHubs.java (bulk-cache control)
│                     LoopTimer.java, WriteGate.java, Timeouts.java, Fork.java
├── pedroPathing/     Constants.java (FollowerConstants, PathConstraints, createFollower)
│                     Tuning.java (Pedro's tuning OpModes)
└── teleop/           DriveTeleOp.java — driver control on the loop contract
                      CommandTeleOpDemo.java — upstream Ivy + Pedro reference, not an OpMode
```

`Drive` and `Intake` exist so far — step 3 of the architecture doc's implementation order
(§10) is under way. `commands/`, `routines/` and `auto/` appear when there is a routine to
write.

## Libraries

Declared in `build.dependencies.gradle` (Maven repo `https://mymaven.bylazar.com/releases`):

| Dependency | Version | Purpose |
|---|---|---|
| `com.pedropathing:ftc` | 2.1.2 | **Pedro Pathing** — path following, localization, drivetrain |
| `com.pedropathing:ivy` | 1.0.0 | **Ivy** — command/scheduler framework |
| `com.pedropathing:telemetry` | 1.0.0 | Pedro telemetry |
| `com.bylazar:fullpanels` | 1.0.12 | Panels dashboard (`PanelsTelemetry`) |

### Pedro Pathing

Path-following and localization library used for both autonomous and TeleOp driving.
Configured in `pedroPathing/Constants.java`; tuning OpModes are in `pedroPathing/Tuning.java`.

Docs: <https://pedropathing.com/docs>

### Limelight 3A (vision)

Computer vision runs **on the camera, not on the Control Hub.** The Limelight 3A executes the
AprilTag/neural pipeline on its own processor and returns a finished result over
Ethernet-over-USB. Driver: `com.qualcomm.hardware.limelightvision.Limelight3A`, shipped with
the SDK — no extra dependency.

Two things matter when writing code against it (both verified in the SDK 12.0 sources):

- The driver runs **its own polling thread** (`ScheduledExecutorService`, 100 Hz by default)
  and stores the result in a `volatile` field. `getLatestResult()` just reads that field, so
  it is free to call on the control thread. Do **not** write a second vision thread.
- **Every other method is a blocking HTTP call.** `pipelineSwitch()` and `getStatus()` block
  up to 100 ms; `updateRobotOrientation()` — required per frame by MegaTag2 — is a POST with a
  **15-second** read timeout. On the control thread that is a worse stall than anything in
  last season's code. Init-time only, or a dedicated background thread. See architecture §7.3.

Prefer MegaTag1 (`getBotpose()` gated on `getBotposeTagCount() >= 2`); MegaTag2 only with an
off-thread yaw pusher.

Docs: <https://docs.limelightvision.io/> ·
[FTC setup](https://docs.limelightvision.io/docs/docs-limelight/getting-started/ftc)

### Ivy

Pedro's command framework (`com.pedropathing.ivy`) — commands, command groups, and a
**static, single-threaded, cooperative** `Scheduler`. Key points when writing code against it:

- `Scheduler.reset()` **must** be the first line of `runOpMode()` — its state is static and
  leaks between OpModes.
- `Scheduler.execute()` is called once per loop iteration; it gives each running command one
  `execute()` turn. **Nothing may block** — a blocking command blocks the whole robot.
- `Scheduler` is **not thread-safe.** Never call `schedule()` / `cancel()` off the control
  thread (this matters for vision).
- Composition: `Groups.sequential/parallel/race/deadline/repeat/loop`,
  `Commands.instant/waitMs/waitUntil/infinite/branch/match/lazy`,
  `PedroCommands.follow/hold/turnTo`.
- Requirements are `Set<Object>` (use the subsystem instance) with `priority()` plus
  `InterruptedBehavior` / `ConflictBehavior` / `BlockedBehavior`.

Docs: <https://pedropathing.com/docs/ivy>

**Ivy has no subsystem abstraction and no periodic hook** — the read/compute/write runtime
around it is ours to build. See the architecture doc.

**Known bugs in Ivy 1.0.0** (verified in source — full list and mitigations in the
architecture doc, §2.1):
- `PedroCommands.follow/hold/turnTo` declare **no requirements**; always add
  `.requiring(driveSubsystem)` or two paths will fight over the drivetrain.
- `Command.proxy()` has an inverted `done()` condition — don't use it.

## Architecture

**[`docs/architecture/autonomous-command-architecture.md`](docs/architecture/autonomous-command-architecture.md)**
is the design document for the Ivy + Pedro autonomous stack. Read it before writing
subsystem, command, or OpMode code.

It specifies the non-blocking loop contract, the `Subsystem`/`Robot` layering, latency
budgets (bulk caching, write gating), how to overlap mechanism motion with path following,
continuous background aiming, Limelight 3A integration and vision-latency correction, and the
Ivy 1.0.0 caveats to work around.

The short version — these are hard rules for new code:

1. **Never block.** No `sleep()`, no `while (motor.isBusy())` on the control thread.
2. **Hardware I/O only in `Subsystem.read()` / `write()`.** Commands mutate setpoint fields.
3. **Every command declares `.requiring(subsystem)`.**
4. **Every wait has a timeout.**
5. **Default to `parallel` / `deadline`; `sequential` is a deliberate choice.**
6. **Build `PathChain`s at init, never mid-routine.**
7. **On the control thread, the only `Limelight3A` calls are `getLatestResult()` and
   `isConnected()`.** Everything else blocks.

Motivation: in the 2025–26 season, non-drivetrain mechanisms were driven synchronously,
which starved `follower.update()` and made aiming act on stale pose estimates. The whole
design exists to make sensor→actuator latency exactly one (short) loop period.

## Build

```bash
./gradlew :TeamCode:assembleDebug        # build the team code APK
./gradlew :TeamCode:compileDebugJavaWithJavac   # fast compile check
./gradlew installDebug                   # deploy to a connected Robot Controller
```

Android Gradle Plugin 9.4.0, Java 8 source/target, `minSdk 24`, `compileSdk 30`.
`minSdk 24` means `java.util.function` and `java.util.stream` (which Ivy uses heavily) are
available without desugaring.

Verify changes compile, then check the loop-timing telemetry on the hardware. See
**Testing** below for what can and cannot be tested off the robot.

## Testing

There is no automated test harness yet; today the robot is the test. This section records
what was measured about the obstacles, because the shape of the harness follows from it.

### Why the FTC SDK resists unit testing

The SDK ships as Android AARs, so anything that touches the Android runtime cannot load in a
JVM unit test. Verified against the SDK 12.0 sources:

| | Loads on a plain JVM? |
|---|---|
| `DcMotorEx`, `DcMotor`, `Servo`, `HardwareDevice`, `Telemetry` | **Yes** — plain interfaces, no `android.*` imports |
| Pedro `core` (`Follower`, `Localizer`, `Drivetrain`, `Pose`, `PathChain`) and all of Ivy | **Yes** — ordinary Java jars |
| `HardwareMap` | No — requires an `android.content.Context` and `com.qualcomm.robotcore.R` |
| `LynxModule` | No — concrete class over USB/RS-485 comms |
| `LinearOpMode` / `OpMode` | No — extends `OpModeInternal`, needs the app's event loop |

So the untestable surface is small and well-defined: **`HardwareMap`, the hubs, and the
OpMode base class.** Everything else our code is built from already runs headless.

### What the current code does about it

These are deliberate choices in the shell, not accidents — keep them when adding subsystems:

1. **`Robot` has a hardware-free constructor**, `Robot(HubManager, Follower, Intake)` —
   one parameter per hardware-backed subsystem — with
   `Robot.fromHardwareMap(hw)` as the competition path. A `HardwareMap` never reaches
   anything below that factory.
2. **New subsystems take their devices, never a `HardwareMap`** — `new Lift(DcMotorEx motor)`
   plus a static `fromHardwareMap` factory that does the lookups. `DcMotorEx` is an
   interface, so the constructor-injected form is testable and the factory is the only
   untestable line.
3. **Bulk caching sits behind `HubManager`**, so `LynxModule` is confined to `LynxHubs`;
   tests pass `HubManager.NONE`.
4. **The loop lives in `Robot.cycle()`, not in the OpMode.** OpModes are ~20 lines of glue.
   Anything that can call `cycle()` in a `for` loop can run the robot.
5. **Pure logic has zero SDK imports** — `LoopTimer` (clock injectable), `WriteGate`,
   `Timeouts`. These need nothing but JUnit.

Point 4 plus point 1 turns out to be enough for real tests today. `Follower`'s public
constructor takes `(FollowerConstants, Localizer, Drivetrain, PathConstraints)`, and
`Localizer` is an interface while `Drivetrain` is an abstract class — both from Pedro's plain
jar. A hand-written fake localizer and a recording drivetrain give a **simulated robot with no
Android and no mocking framework at all.** This was confirmed by spiking it: the team code
compiles with plain `javac`, and a headless harness scheduling `drive.teleopDrive(...)` and
calling `robot.cycle()` in a loop drives the fake drivetrain, gets preempted by
`drive.turnTo(0)`, and resumes driver control when the turn completes — the full
requirement/suspend/resume path, off the robot.

### The three tiers, in the order they should be built

1. **Pure logic** — `LoopTimer`, `WriteGate`, `Timeouts`, and any aim/geometry math. JUnit
   only. Nothing blocks this today.
2. **Simulated robot** — fake `Localizer` + recording `Drivetrain`, subsystems built from
   mocked SDK interfaces, routines stepped with `robot.cycle()`. Asserts the things that
   actually go wrong: a routine that never finishes, a command that ends without releasing a
   requirement, two commands fighting over an actuator, a missing timeout.
3. **Stub SDK** *(the later project)* — headless `HardwareMap`, hubs and `LinearOpMode` so
   OpModes themselves can run. Tier 2 covers everything below the OpMode, so this only buys
   the last glue layer — worth scoping against Robolectric, which solves the same problem by
   supplying an Android runtime instead of a stub. The hazard to design around is classpath
   collision: a stub sharing fully-qualified names with the real AARs must *replace* them on
   the test classpath, not sit alongside them.

Open questions for whoever picks up the harness ticket: JUnit 4.13.2 and (if mocks are
wanted) mockito-core 4.x, since the 5.x line requires Java 11 and this module is Java 8;
`testOptions.unitTests.returnDefaultValues = true` in `build.common.gradle`; tests in
`TeamCode/src/test/java`; and whether tier 3 is a stub SDK or Robolectric.

None of this replaces the loop-timing telemetry check on the hardware — a test can prove a
routine terminates, but only the robot can tell you the loop held 6 ms.

## Conventions

- OpModes are annotated `@Autonomous` / `@TeleOp` with an explicit `name` and `group`.
- Package by role: `subsystems/`, `commands/`, `routines/`, `auto/`, `teleop/`,
  `pedroPathing/`, `util/`.
- Field-unit convention follows Pedro (inches, radians) — heading in **radians** at API
  boundaries; convert only for display.
