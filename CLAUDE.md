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
│                     aimError() — vision's aim corrected for the turn since capture;
│                     shotDistance() — distance to the up CELL, last sighting held;
│                     DEBUG_TIMING per-phase timing line
├── subsystems/       Drive.java — Pedro Follower wrapped as a requirable subsystem
│                     Intake.java — one open-loop motor ("intake"), collect/eject/off
│                     Transfer.java — open-loop motor ("transfer") lifting pollen into the
│                     flywheel, feed/reverse/off
│                     Outtake.java — flywheel ("outtake"), velocity control in write();
│                     SHOT_POINTS distance→rpm table, voltage compensated.
│                     spin/spinFor/spinAt/off, atSpeed/readyToFeed
│                     Battery.java — battery voltage, sampled every 100 ms (not bulk data)
│                     Vision.java — HIVE CELL clusters relative to the robot; which CELL is up
├── vision/           VisionSource (interface) / LimelightSource (the only Limelight3A user)
│                     VisionSample — LLResult reduced to per-tag robot-frame positions
│                     ClusterFit — cluster centre from 2–4 of its tags; CellSighting
│                     FieldFrame — FTC field frame (m, centre) ↔ Pedro (in, corner)
├── field/            Cell.java — the four HIVE CELLS and their tag IDs (manual §9.9)
│                     Alliance.java
├── util/             Subsystem.java (the periodic hook Ivy lacks)
│                     HubManager.java / LynxHubs.java (bulk-cache control)
│                     LoopTimer.java, WriteGate.java, Timeouts.java, Fork.java
│                     Aim.java — bearing / turn / distance to a goal, pure math
│                     HeadingHistory.java — wall-clock-keyed headings for latency correction
│                     AimController.java — chassis aim law (P + kS, capped), Panels-tunable
│                     ShotMap.java — launch speed by distance, interpolated measured table
├── pedroPathing/     Constants.java — Pedro 3 configs (drivetrainConfig, localizerConfig,
│                     foresightConfig), createFollower, LOCALIZER_INSTALLED, FORESIGHT_TUNED
│                     StubLocalizer.java — fixed pose, for a chassis without odometry
│                     UntunedAlgorithm.java — stands in for Foresight when FORESIGHT_TUNED is false
│                     Tuning.java — @Tuner registrations for the AutoTune webpage
│                     procedures/ — AutoTune procedures, copied unchanged from the Quickstart
├── routines/         TestRoutines.java — routine trees as static factories (§8.2)
│                     ShootingRoutines.java — aimAndShoot: aim on vision, then feed
├── auto/             TestAuto.java — "Test: Out and Back", step 5 of §10
│                     AimAndShootAuto.java — "Test: Aim and Shoot", no localizer needed
│                     TestPaths.java — its Paths, built at init
└── teleop/           DriveTeleOp.java — driver control on the loop contract; hold-to-aim
                      ShotCalibration.java — measures the flywheel, records SHOT_POINTS
                      DriveMotorTest.java — one drive motor at a time (disabled)

TeamCode/src/test/java/org/firstinspires/ftc/teamcode/
├── sim/              SimRobot (real Robot + Follower over fakes), SimDrivetrain,
│                     SimLocalizer, SimForesight, FakeMotor, FakeVisionSource
├── util/             tier 1: WriteGate, LoopTimer, Timeouts, Aim, AimController,
│                     HeadingHistory, ShotMap
├── vision/           tier 1: FieldFrame, ClusterFit
├── field/            tier 1: CellTest (tag IDs against the manual)
├── subsystems/       tier 2: IntakeTest, TransferTest, OuttakeTest, DriveTest, VisionTest,
│                     BatteryTest
├── routines/         tier 2: TestRoutinesTest, ShootingRoutinesTest
└── RobotTest.java    tier 2: aimError latency correction, phase timing
```

`Drive`, `Intake`, `Transfer`, `Outtake` and `Vision` are the whole robot (plus `Battery`,
which only reads the voltage). Every step of the
architecture doc's implementation order (§10) has code; what is left needs the robot:

- **Step 1:** the ≤ 6 ms median loop has never been measured on hardware. Turn on
  `Robot.DEBUG_TIMING` (Panels) for the per-phase breakdown.
- **Step 3:** the flywheel is the closed-loop subsystem. The hood is fixed, so the launch
  speed is chosen by distance: `Outtake.spinFor` looks up `SHOT_POINTS` (a measured
  distance→rpm table, interpolated, ends held) with the Limelight's distance to the up CELL
  (`Robot.shotDistance`, last sighting held while out of view). Nothing is measured yet:
  `TICKS_PER_REV = 28` is inferred, `KV` assumes 6000 rpm free speed, `SHOT_POINTS` is empty
  and `TARGET_RPM` is 0 — so today it still runs open loop at `SPIN_POWER`. Its output is
  voltage compensated (× 12 V ÷ `Battery.volts()`), because intake current sagged the battery
  enough to drop shots; `SPIN_POWER`/`KV`/`KP` are therefore "at 12 V". The
  **"Calibrate: Flywheel and Shots"** OpMode (`ShotCalibration`) measures all of it, in the
  order in `Outtake`'s Javadoc. Planned: a Pinpoint fallback for the distance once the HIVE's
  field coordinates are known.
- **Step 5:** Pedro is tuned, so `TestAuto` can run; its on-robot run (wall-clock time and
  loop period) is still to do. Real routines need this season's field coordinates.
- **Step 6:** `AimAndShootAuto` runs without odometry; `ShootingRoutines.AIM_TIMEOUT_MS` and
  `FEED_MS` are placeholders.
- **Step 7:** the latency correction is live now the Pinpoint is in, but untested on the
  robot; once it is, `AimController`'s gains can go up.

## Libraries

Declared in `build.dependencies.gradle` (Maven Central, plus `https://mymaven.bylazar.com/releases`
for Panels and `https://repo.dairy.foundation/releases` for Sloth):

| Dependency | Version | Purpose |
|---|---|---|
| `com.pedropathing:revhub` | 3.0.2 | **Pedro Pathing 3** — follower, Foresight, Mecanum, Pinpoint (brings `com.pedropathing:core`) |
| `com.pedropathing:tuning` | 1.0.3-beta.1 | **AutoTune** — robot-hosted tuning webpage (needs Sloth from `repo.dairy.foundation`). 1.0.2's scanner NPEs on the first `@Tuner`, leaving the page empty |
| `com.pedropathing.ivy:core` | 1.1.1 | **Ivy** — command/scheduler framework |
| `com.bylazar:fullpanels` | 1.0.12 | Panels dashboard (`PanelsTelemetry`) |

### Pedro Pathing

Path-following and localization library used for both autonomous and TeleOp driving.
**Version 3** — a ground-up rewrite of 2.x, not an upgrade: `com.pedropathing.math.Pose`
(`x()`/`heading()`, heading normalised to [0, 2π)), `Path` built with
`Paths.line(a, b).constant(h)`, and a `Follower(Localizer, Drivetrain, Algorithm)` driven by
four mode changes — `follow`, `hold`, `manual`, `stop` — none of which touches hardware
(motors are written in `follower.update()`). Configured in `pedroPathing/Constants.java`.

Drive motors are `motorFL`/`motorFR`/`motorBL`/`motorBR` (`Mecanum`, directions verified on
the robot; its `CachedMotor`s gate writes). Localization is a **goBILDA Pinpoint** (I2C
`"pinpoint"`, two 4-Bar pods, both directions FORWARD). The pod offsets are the Pinpoint
Tuner's output — x pod −3.5 in, y pod −0.5 in — not the tape-measured −3.0 / 0.0. Building
the localizer recalibrates the Pinpoint IMU and sleeps 500 ms — init only, robot still.

**Tuning is AutoTune**, a webpage the Robot Controller hosts at
`http://192.168.43.1:10158` (starts with the app; Panels stays on 8001). `Tuning.java`
registers its procedures with `@Tuner`; each prints Java to paste into `Constants`. Order:
Mecanum Tuner → Pinpoint Tuner → Foresight Tuner → Tests. Re-run the affected tuners after
any drivetrain, pod or weight change.

**Pedro is tuned**: `drivetrainConfig`, `localizerConfig` and `foresightConfig` hold
AutoTune's output, and `Constants.FORESIGHT_TUNED = true`. Paths, `turnTo`/`hold`,
`TestAuto` and the TeleOp A-button snap are all live. The flag stays as a switch: Foresight
cannot be constructed without its measurements, so setting it `false` (say, after clearing
`foresightConfig` to re-tune) puts `UntunedAlgorithm` in its place — manual driving still
works, following/holding commands zero power, and anything checking
`Constants.canFollowPaths()` refuses.

`Constants.LOCALIZER_INSTALLED` (`true`) is the same kind of switch for odometry: `false`
puts `StubLocalizer` in, for a chassis without pods.

The Pinpoint is an I2C device, read inside `follower.update()` (phase 3) — the one I2C read
the loop budget accepts. Check the `update` figure in `Robot.DEBUG_TIMING`'s phase line.
The other non-bulk read is the battery voltage (`VoltageSensor.getVoltage()` is its own hub
transaction, `LynxGetADCCommand`): `Battery` samples it once per 100 ms in phase 2, so it
shows in the `read` max, not the median.

Docs: <https://pedropathing.com/docs/pathing> ·
[Quickstart](https://github.com/Pedro-Pathing/Quickstart) (source of `procedures/`)

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

**BIOBUZZ: no botpose (MegaTag1 or 2).** Every tag (IDs 30–45) is on a HIVE CELL, and each
HIVE is a seesaw that tips ±30° during the match (manual §9.6, §9.9). A field-map solve is
only right while every HIVE sits as the map assumes, and there are no fixed tags to fall back
on. So `LimelightSource` passes each tag's **robot-relative** position
(`getTargetPoseRobotSpace()`), `ClusterFit` fits a CELL's row of four tags (offsets ±2.75,
±6.5 in) into a cluster centre from ≥ 2 of them, and `Vision` exposes per-CELL
`CellSighting`s (bearing, floor distance, height) plus `upCell(alliance)` — the CELL to
launch into. Both CELLS' clusters face down in either state; the up one is ~15 in higher,
so height decides (`UP_CELL_MIN_HEIGHT_IN`, derived from the manual's drawings). Age is
measured from the shutter (`parsedAt − capture − targeting latency`). Vision does **not**
localize the robot; field pose comes from the Pinpoint alone.

**Aiming turns the whole robot — there is no turret.** The flywheel is fixed to the chassis.
`Vision.aimError(alliance)` is the turn (radians, CCW +) that puts the launcher's line of fire
through the alliance's up CELL; `Drive.aimedTeleopDrive` servos it to zero through
`AimController` while the driver keeps translation, falling back to the turn stick when the
target is out of view. It closes the loop on the vision measurement, so it needs no
odometry. In `DriveTeleOp`: alliance chosen in init (X blue, B red), right trigger held to
aim. The flywheel fires straight forward from the robot's centreline (confirmed from the
CAD: `AimController.LAUNCH_DIRECTION = 0`, `LAUNCHER_LEFT_OFFSET_IN = 0`), so aiming points
robot forward at the target. The gains are untuned starting points, live-editable in
Panels. Vision lag (~30–50 ms) limits the gains. `Robot.aimError` corrects for it by
subtracting the heading change since the frame's capture (`HeadingHistory`, wall-clock
keyed like the Limelight's timestamps). It depends on the Pinpoint's heading; on
`StubLocalizer` the change is always zero and the correction does nothing.

Camera-side setup the code assumes: an AprilTag pipeline at index 0 (36h11, full 3D, tag
size 82.55 mm), and the camera's mounting entered in the Limelight UI so robot-space z is
height above the floor. The camera must look **up** at the CELL undersides (clusters at
roughly 34 and 49 in). **Unverified on hardware:** the Limelight robot-space y sign
(`LimelightSource.LEFT_SIGN`) and the up-CELL height threshold — check both against a tape
measure on a real HIVE. `FieldFrame` copies Pedro 2's `FTCCoordinates.convertToPedro` (Pedro 3
has no FTC-frame conversion) and is also unverified.

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
  `Commands.instant/waitMs/waitUntil/infinite/branch/match/lazy`.
- Since 1.1.0 Ivy is split into `com.pedropathing.ivy:core` (Pedro-independent; what we use)
  and `:pedro` (`PedroCommands`, for Pedro 3). Ivy 1.0.0 must not be used with Pedro 3: its
  `Commands.waitMs` was built on Pedro 2's `Timer` and throws `NoClassDefFoundError`.
- Requirements are `Set<Object>` (use the subsystem instance) with `priority()` plus
  `InterruptedBehavior` / `ConflictBehavior` / `BlockedBehavior`.

Docs: <https://pedropathing.com/docs/ivy>

**Ivy has no subsystem abstraction and no periodic hook** — the read/compute/write runtime
around it is ours to build. See the architecture doc.

**Known bugs in Ivy 1.1.1** (verified in source; its core is 1.0.0's apart from `waitMs` —
full list and mitigations in the architecture doc, §2.1):
- `PedroCommands` (the `:pedro` module, not a dependency here) still declares **no
  requirements** and **no `end()`**, has no `turnTo`, and `hold(follower)` captures the pose
  when the command is *built*. `Drive` implements `follow`/`turnTo`/`hold` itself — always go
  through `Drive`'s factories.
- `Command.proxy()` has an inverted `done()` condition — don't use it.
- `Sequential` starts its next step one cycle after the previous one ends (it checks
  `done()` before `execute()`); each nesting level of groups adds a cycle at boundaries.

## Architecture

**[`docs/architecture/autonomous-command-architecture.md`](docs/architecture/autonomous-command-architecture.md)**
is the design document for the Ivy + Pedro autonomous stack. Read it before writing
subsystem, command, or OpMode code.

It specifies the non-blocking loop contract, the `Subsystem`/`Robot` layering, latency
budgets (bulk caching, write gating), how to overlap mechanism motion with path following,
continuous background aiming, Limelight 3A integration and vision-latency correction, and the
Ivy caveats to work around.

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

```bash
./gradlew :TeamCode:testDebugUnitTest      # tiers 1 and 2, ~10 s, no robot needed
```

Tiers 1 and 2 below exist; tier 3 does not. This section records what was measured about the
obstacles, because the shape of the harness follows from it.

### Why the FTC SDK resists unit testing

The SDK ships as Android AARs, so anything that touches the Android runtime cannot load in a
JVM unit test. Verified against the SDK 12.0 sources:

| | Loads on a plain JVM? |
|---|---|
| `DcMotorEx`, `DcMotor`, `Servo`, `HardwareDevice`, `Telemetry` | **Yes** — plain interfaces, no `android.*` imports |
| Pedro `core` (`Follower`, `Localizer`, `Drivetrain`, `Pose`, `Path`, Foresight) and Ivy `core` | **Yes** — ordinary Java jars |
| `HardwareMap` | No — requires an `android.content.Context` and `com.qualcomm.robotcore.R` |
| `LynxModule` | No — concrete class over USB/RS-485 comms |
| `LinearOpMode` / `OpMode` | No — extends `OpModeInternal`, needs the app's event loop |

So the untestable surface is small and well-defined: **`HardwareMap`, the hubs, and the
OpMode base class.** Everything else our code is built from already runs headless.

### What the current code does about it

These are deliberate choices in the shell, not accidents — keep them when adding subsystems:

1. **`Robot` has a hardware-free constructor**, `Robot(HubManager, Follower, Battery, Intake,
   Transfer, Outtake, Vision)` —
   one parameter per hardware-backed subsystem — with
   `Robot.fromHardwareMap(hw)` as the competition path. A `HardwareMap` never reaches
   anything below that factory.
2. **New subsystems take their devices, never a `HardwareMap`** — `new Lift(DcMotorEx motor)`
   plus a static `fromHardwareMap` factory that does the lookups. `DcMotorEx` is an
   interface, so the constructor-injected form is testable and the factory is the only
   untestable line.
3. **Bulk caching sits behind `HubManager`**, so `LynxModule` is confined to `LynxHubs`;
   tests pass `HubManager.NONE`. Likewise `Limelight3A`/`LLResult` (built on Android's
   `org.json`, a stub on the JVM) are confined to `LimelightSource` behind `VisionSource`;
   tests pass `VisionSource.NONE` or a `FakeVisionSource`.
4. **The loop lives in `Robot.cycle()`, not in the OpMode.** OpModes are ~20 lines of glue.
   Anything that can call `cycle()` in a `for` loop can run the robot.
5. **Pure logic has zero SDK imports** — `LoopTimer` (clock injectable), `WriteGate`,
   `Timeouts`. These need nothing but JUnit.

Point 4 plus point 1 turns out to be enough for real tests today. `Follower`'s public
constructor takes `(Localizer, Drivetrain, Algorithm)` — all three interfaces from Pedro's
plain `core` jar. A hand-written fake localizer and a recording drivetrain give a **simulated robot with no
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

How it is set up: JUnit 4.13.2, declared in `TeamCode/build.gradle` (not
`build.common.gradle`, which stays upstream-pristine). No mocking framework —
`sim/FakeMotor` fakes `DcMotorEx` with a `java.lang.reflect.Proxy`, and
`returnDefaultValues` has not been needed. `SimLocalizer` integrates over **real** time
because the follower times its own loop with `System.nanoTime()`, so tier-2 tests take as
long as the motion they simulate (a 24 in leg ≈ 1 s). The sim runs the real Foresight on
`SimForesight`'s config — numbers matched to the sim, not the robot. The simulated
drivetrain has no inertia: it proves routines terminate and hand the drivetrain over
correctly, not that Foresight is tuned.
When adding a subsystem, give it an `XTest` beside `IntakeTest` and a `FakeMotor` in
`SimRobot`. Still open: whether tier 3 is a stub SDK or Robolectric.

None of this replaces the loop-timing telemetry check on the hardware — a test can prove a
routine terminates, but only the robot can tell you the loop held 6 ms.

## Conventions

- OpModes are annotated `@Autonomous` / `@TeleOp` with an explicit `name` and `group`.
- Package by role: `subsystems/`, `commands/`, `routines/`, `auto/`, `teleop/`,
  `pedroPathing/`, `vision/`, `field/`, `util/`.
- Field-unit convention follows Pedro (inches, radians) — heading in **radians** at API
  boundaries; convert only for display.
