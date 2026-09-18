# CLAUDE.md

## What this repo is

The **FTC SDK** (`FtcRobotController`) plus team code for **BIOBUZZ**, competing in the
**FIRST Tech Challenge 2026–27 season**.

`FtcRobotController/`, `build.common.gradle`, `libs/`, `doc/`, and the (very large)
`README.md` come from FIRST's upstream
[FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController) repo
(currently v11.2 / SDK 11.2.1). **Don't edit them** unless a task is specifically about
upgrading the SDK — keeping them pristine is what makes SDK updates mergeable.

All team code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── pedroPathing/     Constants.java (FollowerConstants, PathConstraints, createFollower)
│                     Tuning.java (Pedro's tuning OpModes)
└── teleop/           CommandTeleOpDemo.java — current Ivy + Pedro reference OpMode
```

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

Two things matter when writing code against it (both verified in the SDK 11.2.1 sources):

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

There is no test harness — the robot is the test. Verify changes compile, then check loop
timing telemetry on the hardware.

## Conventions

- OpModes are annotated `@Autonomous` / `@TeleOp` with an explicit `name` and `group`.
- Package by role: `subsystems/`, `commands/`, `routines/`, `auto/`, `teleop/`,
  `pedroPathing/`, `util/`.
- Field-unit convention follows Pedro (inches, radians) — heading in **radians** at API
  boundaries; convert only for display.
