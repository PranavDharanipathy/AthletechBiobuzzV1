# AGENTS.md

Canonical guide for AI coding agents (and humans) working in this repository.
Read this first. Operational procedures live in [AGENTOPS.md](./AGENTOPS.md);
project-specific decisions, gotchas and caveats live in [DEV_NOTES.md](./DEV_NOTES.md).

## What this is

`AthletechBiobuzzV1` is a **FIRST Tech Challenge (FTC)** robot project for the
**BIOBUZZ** season (2026-2027), built on the **FTC SDK 12.0.0**. It is a standard
`FtcRobotController` Gradle project with an extra desktop **simulation** module (`Sim`)
added alongside the competition code.

## Modules

| Module | Type | Package / namespace | Purpose |
|--------|------|---------------------|---------|
| `FtcRobotController` | `com.android.library` | `com.qualcomm.ftcrobotcontroller` | Stock FTC SDK app shell. **Do not edit** — it is updated wholesale from upstream. |
| `TeamCode` | `com.android.application` | `org.firstinspires.ftc.teamcode` | The robot app we actually write. OpModes, subsystems, autonomous, tuning. This is what deploys to the Robot Controller / Control Hub. |
| `Sim` | plain JVM (`org.jetbrains.kotlin.jvm` + `application`) | `org.athletech.sim` | Desktop physics sims, prediction algorithms, and the **robot profile database** (`Sim/data/robot_profile.db`) that game simulators read. Never deployed. See [Sim/README.md](./Sim/README.md). |

### `TeamCode` source layout (`org.firstinspires.ftc.teamcode`)

```
Autonomous/        Autonomous OpModes
  Autos/           Six self-contained autos (Red/Blue Park, Preload + Park, 2 Tip Flowers + Park), nothing shared between them
  AutoSubsystems/  Auto-only helpers
Constants/         Robot + Pedro Pathing constants, hardware maps
Sim/               Bridge between robot code and the Sim module
  Adapters/        Adapt hardware interfaces to sim implementations
  Interfaces/      Abstractions shared by real + simulated hardware
Systems/           Robot subsystems (drive, mechanisms)
  Vision/          Vision guide (README.md) + LimelightBridge (hub -> Limelight port forwarding for tuning)
TeleOp/            Driver-controlled OpModes
Testing/           Throwaway / diagnostic OpModes
Tuners/            PID / path / sensor tuning OpModes (pair with FTC Dashboard)
Utility/           Shared helpers (MathUtil, BetterGamepad, RobotLogger, ...)
  Hardware/        Monitored* drop-in hardware wrappers (read caching + I/O stats)
  Monitoring/      HardwareMonitor: loop/power/hub/motor monitoring + on-robot database
                   + MonitorWebServer (web dashboard at :8080/hwmonitor)
  opmodes/         Utility OpModes (Hardware Monitor Viewer, Clear Database, example, Limelight Lab)
```

The match OpModes (Red/Blue TeleOp and their Easy / Temporal Log versions, the six autos) show a **Loop** telemetry line from
`Utility/TickRateChecker`: last loop, smoothed average and Hz, worst in the last 5 s. `tick()` runs once per loop
(`TeleOpBaseOpMode.clearCacheOfLynxModule`, `AthletechRobot.periodic`); the line only reads it.

### `Sim` layout (`org.athletech.sim`)

```
Sim/
  README.md          how to run sims and fill the profile
  PROFILE.md         data dictionary of the profile DB (generated: profile docs)
  data/robot_profile.db   THE robot profile (SQLite), tracked in git
  data/templates/    CSV templates for real shot / pose / disconnect logs
  out/               reports + scratch profiles (git-ignored)
  src/main/java/org/athletech/sim/
    Main.java        CLI: profile ..., sim ..., predict ...
    config/          Catalog (every profile entry), ProfileDb, ProfileSchema, RobotParams
    physics/         motor, battery, projectile models
    robot/           MecanumDrive (with wheel slip), Flywheel, Launcher, Turret, MatchDriver (full physics match)
    field/           Aperture (scoring openings)
    undeterminables/ error distributions, odometry drift, LoopTimingModel, static charge
                     (RobotCharge, TileChargeGrid, ChargeSim, Electrostatics), Disconnects
    predict/         AimSolver, ShotPredictor (Monte Carlo), DriveTimePredictor, LoopControlSim
    robotcode/       adapters that let TeamCode classes run on sim time and sim hardware (SimClock, SimFlywheelMotor)
    scenarios/       drive / launcher / localization / loop / esd sims that save results, plus code-in-the-loop
                     turret (TurretScenario) and flywheel (FlywheelCodeScenario) sims that run TeamCode's own classes
    input/           HardwareMonitor + shot/pose/disconnect log importers
    telemetry/ visualization/ utility/
```

## Toolchain & build config

- **Android Gradle Plugin** 8.13.2, **Gradle** 9.1.0
- **compileSdk / targetSdk** 36, **minSdk** 24
- **Java** 8 (`sourceCompatibility`/`targetCompatibility = 1.8`)
- **Kotlin** 2.4.10 (`org.jetbrains.kotlin.android`), `jvmTarget = JVM_1_8` to match Java
- **JDK** 25 runs the build — this is why Gradle must be **≥ 9.1.0** (first Gradle with Java 25 support)
- Shared config lives in `build.common.gradle`; shared dependencies/repos in
  `build.dependencies.gradle` (applied by both `FtcRobotController` and `TeamCode`).
- **Sim** is a desktop module, also **Java 8** source/target (Kotlin `JVM_1_8`), with its own
  dependencies in `Sim/build.gradle.kts` (`org.xerial:sqlite-jdbc`, commons-math3, JUnit 4). It must
  not depend on the FTC SDK or Android. It *does* compile a list of pure-Java TeamCode classes
  (`copyTeamCodeSources`) against the stand-ins in `Sim/src/ftcStubs`, so its sims run the robot's
  real control code. Keep those classes free of hardware calls beyond what the stand-ins have.
- **All Java in this repo is Java 8.** No `record`, `var`, `switch` arrows/expressions, text blocks
  (`"""`), `instanceof` patterns, `List.of`/`Map.of`/`Set.of`, `Path.of`, `Files.readString`/
  `writeString`, `String.isBlank`/`repeat`/`strip`, `Stream.toList()`, `Optional.orElseThrow()`
  (no-arg), `HexFormat`. Use `Arrays.asList`, `Collections.*`, `Paths.get`, `Files.readAllBytes`/
  `Files.write`, `collect(Collectors.toList())`, classic `switch` with `break`.

Three constraints that are easy to break — all explained in DEV_NOTES:

> 1. `targetSdk` may never exceed `compileSdk`. If you bump one, bump both.
> 2. The Kotlin plugin version is pinned by **both** the Gradle/AGP pair *and* the
>    newest `kotlin-stdlib` any dependency drags in. Pedro's `tuning` pulls Sloth,
>    which needs stdlib **2.4.10**, so the compiler is **2.4.10**. A compiler can only
>    read stdlib metadata one minor version ahead — a 2.2 compiler fails on 2.4.
> 3. Java and Kotlin JVM targets must match (both 1.8), or the build fails hard.

This project runs **Gradle 9**, which removed the Groovy space-assignment syntax
(`propName value`) for Gradle-generated properties. Use `propName = value`. This
is why `signingConfig`, `ndkVersion` and `jniLibs.useLegacyPackaging` are written
with `=`. Genuine AGP DSL methods (`keyAlias`, `pickFirst`, `abiFilters`,
`debuggable`, …) are unaffected and left alone.

## Key dependencies

| Library | Coordinate | Repo |
|---------|-----------|------|
| FTC SDK | `org.firstinspires.ftc:*:12.0.0` | Maven Central |
| FTC Dashboard (Sloth fork) | `com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0` | `https://repo.dairy.foundation/releases/` |
| Pedro Pathing 3 | `com.pedropathing:revhub:3.0.2` (pulls `core`) | Maven Central |
| Pedro tuning | `com.pedropathing:tuning:1.0.1` (pulls Sloth) | Maven Central + `https://repo.dairy.foundation/releases/` |
| NextFTC v2 | `dev.nextftc.v2:control` / `hardware` / `robot`, all `0.3.0` | Maven Central |
| Ivy Pedro commands | `com.pedropathing.ivy:pedro:1.1.1` (`PedroCommands`) | Maven Central |
| BlazeFTC (beta, **off**) | `dev.anygeneric:blazeftc:0.1.66` | `https://maven.anygeneric.dev/` |
| Apache Commons Math | `org.apache.commons:commons-math3:3.6.1` | Maven Central |
| Kotlin stdlib | `org.jetbrains.kotlin:kotlin-stdlib:2.4.10` | Maven Central |

FTC Dashboard must be **slothboard**, Sloth's fork, never the stock
`com.acmerobotics.dashboard:dashboard`. Pedro's `tuning` pulls Sloth, which hot-reloads
TeamCode into its own classloader; the stock dashboard can't see those classes, so its
Enable/Disable OpMode never registers and `@Config` edits don't reach the running OpMode.
The `0.3.2` prefix must match the Sloth version `tuning` pulls; bump them together. It is
declared with `exclude group: 'org.firstinspires.ftc'` so it can't drag in an older SDK.

`com.pedropathing:ftc` is **Pedro 2 only** (last release 2.1.2). Never add it alongside
`revhub`/`tuning` — it silently mixes `core` 2.x and 3.x on the classpath.

Autonomous command code uses NextFTC **v2**, with `PedroCommands` from Ivy for path following.
Never add NextFTC v1 (`dev.nextftc:ftc`, `dev.nextftc:hardware`) or `dev.nextftc.extensions:pedro`:
v1 clashes with v2, and the extension pulls `com.pedropathing:ftc:2.0.0`. Keep exactly one
`NextRobot` (`Systems/AthletechRobot`) with no hardware access in its constructor. Details are in DEV_NOTES.

**BlazeFTC** (`dev.anygeneric:blazeftc` 0.1.66) runs the hub link for every robot OpMode and every tuner except `Testing/`
testers and the utility OpModes. It's a beta Rust runtime with its own native threads. All of it goes through
`Utility/Blaze/BlazeSession`, switched by `Constants/BlazeConstants`. **`ENABLED` is false**: in the 2028-05-12 match runs
its hub and Pinpoint feeds died ~0.8 s after START with no fault (every loop fell back to SDK reads), loops went 28 -> 38 ms
median with 150-650 ms freezes about once a second (the "lag"), and `MOTOR_ACCELERATION` left the intake under half speed
the whole time (its `setPower` ignores RUN_USING_ENCODER). Don't turn it on again without a `blaze_check.py` run that passes.
Every TeleOp records `blaze.*` (hub data age and staleness, update counts, Pinpoint use, faults) and `pose.blaze_age_ms`;
`python -I tools/blaze_check.py [RUN]` judges a run against the newest Blaze-off TeleOp (feeds, faults, loop times,
command-to-motion lag, pose resets, turret, intake).

- **TeleOps and tuners:**
  - `TeleOpBaseOpMode` calls `begin` (first thing in `initializeComponents`), `engage` (in `setUpLynxModule`), `tick` (in
    `clearCacheOfLynxModule`) and `end` (in `stop`).
  - The four standalone tuners call the same four themselves.
  - Pedro's Foresight tuner and Tests get their Mecanum through `Tuning.blazeMecanum` (`begin` + `tick`, motors only, so a
    no-op while `MOTOR_ACCELERATION` is off).
- **Autos:** plain `NextOpMode`, no Blaze. As `BlazeOpMode`s, NextFTC started Blaze's proxy for them even with Blaze off, and
  the hubs retransmitted packets every auto (robot log 2028-05-08).
- **What Blaze does:**
  - motor `setPower` goes straight to Blaze (only with `MOTOR_ACCELERATION`, off);
  - each hub's bulk data is kept fresh in the background, and `tick` never clears a hub Blaze feeds;
  - `BlazePinpointLocalizer` gets Pedro its Pinpoint data from Blaze, falling back to an SDK read when Blaze's sample is
    stale. Its `setPose` is a software offset.
- **Clean-up:** `MonitorOpModeHook` calls `BlazeSession.end()` after every OpMode, crashed or not. That closes Blaze, puts the
  SDK motors back in the hardwareMap and sets the hubs back to the SDK's bulk mode.
- **Rules:**
  - Blaze callbacks only copy data in; never touch the follower or hardware from them.
  - Get hardware only after `begin`.
  - Don't use Blaze's Pedro add-on: its `setPose` throws and it writes the drive from Blaze's thread.
  - Bump Blaze and NextFTC v2 together.

Details and what's unproven are in DEV_NOTES.

**Temporal Log Red** (`TeleOp/TemporalLogRedTeleOp`) is the RED TeleOp exactly, with everything logged against time by
`Utility/TemporalLog`:

- every hardware read and write, with its value and duration (`DeviceTracker` and HardwareMonitor's hub samples);
- every variable of the TeleOp's object graph and of the Constants, whenever it changes (`VariableWatcher`);
- Blaze's background reads;
- loops and phases.

Files go to `/sdcard/FIRST/data/TemporalLog/<time>/`. Read them with `python tools/temporal_log.py` (pull, summary, csv,
decisions, keys). A new wrapper read or write must call `tracker.v(value)` before `tracker.read/write`, or its value logs as
NaN.

## Conventions for agents

- Format like Pranav: one statement per line (no hanging wraps; only his block lists, builder chains and array
  lists go multi-line), `else` on the line after `}`, blank line after `{` in multi-line methods. Run
  `python tools/pranavfmt.py <files>` on Java you write. See the code style rule in DEV_NOTES.
- Only comment when people literally won't get it, rough and short, `//blah`. See the code style rule in
  DEV_NOTES.
- Threads and loop code must suit the Control Hub: 4 slow A53 cores and little RAM, shared with the RC app,
  and one serial link per hub. At most one low-priority daemon background thread per job (`Thread.MIN_PRIORITY`).
  It does file/network I/O only, never hardware calls (hub transactions are serialized, so threads don't
  speed them up, they only add lock waits). It sleeps or blocks, never busy-waits, and stops when the OpMode
  stops. No thread pools, parallel streams or per-loop `new Thread`. No allocation per loop in hot paths:
  reuse buffers and arrays. See DEV_NOTES.
- Write new robot logic in `TeamCode` only. Never modify `FtcRobotController` —
  one deliberate exception: its `AndroidManifest.xml` declares `android:exported`,
  which `targetSdk` ≥ 31 requires (see DEV_NOTES). Re-apply after SDK upgrades.
- New OpModes go in the matching package (`TeleOp/`, `Autonomous/Autos/`, …) and
  are annotated `@TeleOp` / `@Autonomous`.
- Prefer existing subsystems in `Systems/` over inlining hardware calls in OpModes.
- Kotlin and Java may coexist in `TeamCode`; keep `jvmTarget`/Java at 1.8.
- Keep path/PID numbers in `Constants/` or `Tuners/`, not hard-coded in OpModes.
- Get hardware through the `Utility/Hardware` wrappers (`MonitoredDcMotorEx`,
  `MonitoredServo`, `MonitoredIMU`, ...) instead of `hardwareMap.get(...)` directly. They are
  drop-in (store them as `DcMotorEx`, `Servo`, `IMU`, ...); the constructor's boolean turns
  per-loop read caching on.
- Call `HardwareMonitor.get().update()` once at the top of every loop. It measures loop time,
  refreshes the read caches and samples battery/hub/motor data. Without it, cached reads only
  expire by age (50 ms) and no loop timing is recorded.
- Before adding a dependency, add it to `build.dependencies.gradle` (shared) unless
  it is sim-only, in which case it belongs in `Sim/build.gradle.kts`.
- The robot profile DB is a contract with game-sim authors. Add or change entries only in
  `Sim/.../config/Catalog.java` (naming `category.name_unit`), then run `profile init` and
  `profile docs` so the DB and `Sim/PROFILE.md` follow. Never edit the DB schema by hand.
- Sims must not overwrite better data: write results through `ProfileDb` with a source kind
  (`measured` > `manual` > `sim` > `placeholder`). Never use `--force` on the shared profile
  without the user asking.
- Never fill `Sim/data/robot_profile.db` with `--example` placeholders. Example runs use a
  scratch `--db out/...` file, or `--assume` (a filled copy in `out/`).
- Robot code that needs the time reads `Utility/Clock.seconds()` (same as `System.nanoTime()` on the robot),
  so the code-in-the-loop sims can run it on sim time.

## Hardware monitoring

`Utility/Monitoring/HardwareMonitor` records loop timing, hardware I/O per device, battery
voltage, hub current/rails/temperature, per-motor current and back-EMF, energy, heap and CPU.
Sessions open and save themselves when an OpMode inits and stops (`MonitorOpModeHook`), into
`/sdcard/FIRST/data/HardwareMonitor/` on the robot. Browse them with the **Hardware Monitor
Viewer** OpMode, the **web dashboard** at `http://192.168.43.1:8080/hwmonitor` (charts + sortable
tables + CSV/JSON export; page source `TeamCode/src/main/assets/hwmonitor/index.html`), or
`adb pull /sdcard/FIRST/data/HardwareMonitor` and open that folder in the same page offline;
delete them with the **Hardware Monitor: Clear Database** OpMode, the dashboard, or
`HardwareMonitor.clearDatabase()`. Design, costs and limits
are in [DEV_NOTES.md](./DEV_NOTES.md).

**Recording** (`Utility/Recording`: `Recorder.kt` + `RobotRecording.java`): every TeleOp, every tuner on `TeleOpBaseOpMode`
and every auto records everything every loop to `/sdcard/FIRST/data/RobotLog/<time>-<OpMode>/`, so a past match can be gone over
and fixed without the robot:

- `frames.bin` + `channels.tsv`: ~170 numbers per loop: loop and power, both gamepads (every stick, trigger, button), pose and
  velocity, the drive's command, wheels and pose hold, the turret and its controller (target, PID speed, every volt term,
  gates), the flywheel and its controller, the shooter's aim (raw and routed), cell, goal, distance, shot
  solution and gates, blocker, intake, FIArm, plus each tuner's own (autotune target and stage, ...).
- `events.tsv`: swings, cell switches, relocalizations, driver toggles, auto status/fault/shot-wait changes, autotune stages.
- `meta.tsv`: every Constants value at the start, every hardware device, the hubs, the OpMode, where it started from.

Channels read cached state only (no hardware calls) and are fixed at the first loop; one low-priority thread writes. A new
subsystem gets a method in `RobotRecording`; an OpMode adds its subsystems with `record(shooter, blocker, intake, drivetrain)`
and its own numbers with `recorder.channel(...)` / `recorder.watch(...)` in init; anything can drop an event with
`Recorder.note(...)`. Read runs with `python tools/robot_log.py` (pull, list, summary, events, meta, channels, csv); the build
also pulls them into `tuning-data/robot-logs` (git-ignored). TunerTeleOp differs only in also tuning live.

## Turret control and tuning

**`TurretConstants.TURRET_ENABLED`:** on. Turned off, `TurretBase.setVolts` sends the servos 0 in every OpMode (TeleOps, autos,
tuners), while the aim and the controller still run and are recorded, so the log shows what it would have done.

**Controller** (`Systems/Turret/TurretController`, pure Java, the Sim runs it): a position PID (far/close kp and kd, zoned ki
with smash, filtered d, hold feedforward) whose output is a speed (ticks/s), plus the aim rate from `Shooter` (which already turns
against the robot's spin, scaled by `TURRET_RATE_FEEDFORWARD_SCALE`). There's no velocity loop under it: that speed goes out
through `TURRET_GAIN` (speed per volt) plus a friction feedforward:

- signed by the commanded speed (`TURRET_FRICTION_FROM_VELOCITY`);
- the full breakaway push only while the encoder says it's still (Stribeck on the measured speed,
  `TURRET_STRIBECK_FROM_MEASURED`): with no velocity loop to wind up, it otherwise never starts small moves;
- nothing inside `TURRET_FRICTION_DEADBAND_DEGREES` while the aim is still.

**Pulse hold** (`TURRET_PULSE_*`, on): within `TURRET_PULSE_ZONE_DEGREES` of a still aim (and once the turret has slowed) the
PID is replaced by timed pushes `TURRET_PULSE_OVER_BREAKAWAY_VOLTS` over breakaway, a wait, and a look at where it landed; the push length is learned from that (no move:
longer, overshoot: shorter). Pushing until the encoder shows motion (~110 ms late) threw it ~10 deg past every time, while
50-100 ms pushes move it ~0.7 deg (logs 2028-05-12). Outward pushes near a travel end stay at the minimum. 8 pushes in a row
without a count trips the dead-encoder cutoff (pulses never stay over it long).
The pulses also learn friction during the match (`TURRET_FRICTION_LEARN_*`, `frictionTrim`, recorded as `turret.friction_trim`): a
full-length push that didn't move raises it, a shortest push that overshot lowers it, and it applies to the PID's friction push
too. That carries the hold through battery sag and friction drift (`TurretBatteryTest`: 12.6 -> 10.5 V, servos regulated or on
the battery, friction +-15%: ~0.5 deg, no lurches). `TURRET_VOLTAGE_COMPENSATION` is off: both servos are on hub servo ports,
a regulated supply, so dividing by the battery only pushed harder as it sagged.

It never heads for an end of the travel faster than it can still coast to a stop before it (`TURRET_END_DECEL_DEGREES_PER_S2`,
`TURRET_END_LAG_SECONDS`). `TurretBase` adds the wiring hard limit (past the travel plus `TURRET_HARD_LIMIT_MARGIN_DEGREES` it
only drives back) and the dead-encoder cutoff (`TURRET_FROZEN_ENCODER_*`: 0.5 V over breakaway for 0.3 s with no
counts; an unplugged encoder once spun the turret round. Relative, because breakaway went 1.16 -> ~2.3 -> 1.18 V). The servos ignore commands for ~1.1 s after an OpMode starts (every
log shows it), so the cutoff only counts after `TURRET_SERVO_WAKE_SECONDS` (counting from the start, it killed a working turret
0.7 s in); until the encoder has counted once the push is capped at
`TURRET_UNCOUNTED_OVER_BREAKAWAY_VOLTS` over breakaway. The cutoff latches for the OpMode and the Driver Station "Turret" line says so; "Turret encoder" shows its angle at INIT, so turn the turret by hand there.

**Turret Autotuner** (`Tuners/TurretAutotuner` + `TurretAutotuneEngine`, one OpMode, no hardware calls in the engine; sim-tested
in `TurretAutotuneTest`): robot still, turret straight at INIT. Stages:

1. after the servos wake (~1.1 s), a 4 V kick each way: no count is retried longer (0.15 / 0.22 / 0.3 s, alternating so a dead
   encoder can't carry it far); still no count after all three, or counts the wrong way, = fault;
2. breakaway ramps;
3. open-loop speed holds (speed per volt, sliding friction);
4. `ROUNDS` of step-trial knob tuning scored on settle time, overshoot, final error, holding jitter and volt flips (shake);
5. the **match course** (`COURSE`, same every trial): linear / quadratic / cubic ramps, a wobble, a jittery drift, a big step,
   0.5-2 deg nudges and plain holds, aim rate fed forward like the Shooter. Scored mostly on holding (error, swings, flips) plus
   tracking error, it tunes the close gains, deadband, pulse hold, friction ramp, rate feedforward scale and friction trims.
   B during it keeps what's tuned so far.

B stops it. It saves `/sdcard/FIRST/data/TurretAutotune/<time>/`; the build pulls it into `TunedConstants.TURRET_*` (gain,
friction and the gains), which `TurretConstants` points at (`TurretTuningPullTest`; older-format runs in that folder are skipped).
**Turret Manual Tuner** is for dashboard tweaking. `TurretControllerTest` sims the controller.

Offline: `python -I tools/turret_fit.py fit` fits the turret model to every turret log, `python -I tools/turret_fit.py export`
pulls real-match aim traces into `Sim/out/turret-traces`, and `TURRET_TUNE=1 ./gradlew :Sim:test --tests "*TurretTuning*"`
replays them on the measured turret, a stickier one and the log-fitted one and searches the knobs. The ~120 ms from command to
motion caps match tracking near 50% of loops within 2 deg; only less delay (faster loops) beats that.

The turret travels **±200° from home** (20° past straight back each way) (`ShooterConstants.MIN/MAX_TURRET_POSITION_IN_DEGREES`)
and the robot may spin forever. `TurretRoute` follows the aim round continuously. The match aim (`Shooter`) uses the whole ±200°
and swings a full turn only once the aim is past an end (`TURRET_SWING_MARGIN_DEGREES`).
In the match TeleOps (`Shooter.update`), **gamepad 1 Y** toggles a turret lock: held still at home (0 deg) instead of aiming.
The flywheel runs from START until the OpMode stops (no toggle), and the FIArm dropdown is down only while **gamepad 1 left
bumper** is held.

## Drivetrain pose hold

Every full TeleOp (match and tuning) drives through `Systems/Drivetrain/Drivetrain`. It shapes the sticks with the
`QuadraticCurve` (x², finer at every power under full) and moves a reference. `DriveController` (pure Java, the Sim runs
it) holds the robot to that reference in forward, strafe and turn, using feedforward plus PID from an identified model.
So shoves, pushes from behind and twists get pushed back against, and slow means slow.

- While the sticks slow it down, only speed is held, so stops are as short as open loop with no rollback.
- The reference never asks for more than traction allows.
- A leash stops it lunging after being pinned.
- Pinpoint relocalization jumps are carried over.
- The model and hold stiffness come from the **Drivetrain Autotuner** (SD card, build pull, `TunedConstants.DRIVE_*`).
- `DrivetrainConstants.DRIVE_*` holds the knobs. The hold is optional (`DRIVE_HOLD_ENABLED`, off by default for now); gamepad 1 back turns it on and off.
- Call `follower.update()` before `drivetrain.update()` every loop.

## Match autos

There are six NextFTC autos (group A_Match): Red and Blue each have Park, Preload + Park and 2 Tip Flowers + Park (classes still `*TwoTipGarden`). Each auto is
one self-contained class in `Autonomous/Autos/` with its own `Constants/Autos/<Auto>Constants` (poses and timing).

- **Command based (Ivy on NextFTC):** each auto builds its paths, then one Ivy sequence of the robot's commands inside
  `robot.autonomous(...)`:
  - `drive.follow(path)` (Ivy's `PedroCommands.follow`, done at the end within tolerance, with a timeout);
  - `robot.shoot(pose)`, `robot.collect()`, `robot.stow()`;
  - `shooter.spinUp()`, `waitMs(...)`.
- **The robot wraps the routine:** `autonomous()` adds the time-limit/fault watchdog (loop timing, localization, a path
  timeout) and makes everything safe at the end.
- **Commands live on the robot:** they're on the mechanisms (`Autonomous/AutoSubsystems`) and on `AthletechRobot` for the
  ones using several mechanisms, never in an auto.
- **Nothing shared between autos:** no base class, and no auto-to-auto code or constants. A change to one auto touches only
  its two files.

- **Frame:** all poses are in the turret's frame (`FieldConstants`), which is also the Pedro visualizer's BIOBUZZ field. No
  mirroring.
- **Goal:** the shooter aims at its own color's goal on the robot's half (the one in front). Preload + Park (1 tip) starts
  where 2 Tip Flowers does, in line with the upward CELL, and shoots the same first volley; both check their shot poses'
  halves at INIT.
- **FIArm:** every auto sends it to RETRACT at INIT.
- **Auto to TeleOp handoff** (`Utility/AutoHandoff`): from START every auto (`robot.handOffAs(name)`) keeps the robot's pose and
  the turret's ticks from home in statics and sets `infoFromAuto`. `TeleOpBaseOpMode` takes it once at INIT (and sets the
  flag false): the turret starts from where it really is instead of calling that home, and the follower from the auto's
  pose. Ignored if older than `AutoHandoff.MAX_AGE_SECONDS` or past the turret's travel. Telemetry "Start" says which.
- **Paths that can't quite finish:** `Drive.follow` also counts a path done once it's at the end within `AUTO_ARRIVE_INCHES` /
  `AUTO_ARRIVE_DEGREES` and stopped for `AUTO_STILL_SECONDS` (`DrivetrainConstants`), noting it in the log. Before, a robot pressed into
  a wall or POLLEN sat until the 8 s path timeout, which faulted and stopped the whole auto.
- **Visualizer projects:** `docs/autos/*.pp`.
- **Rules, routes and field checks:** `docs/AUTOS.md`.

## Hypertuning and system evolution

**SHT Stability Tuner** (`Tuners/ShooterHyperTuning`) is the full RED TeleOp. While it's played it keeps tuning, with
`OnlineTuner`:
- the flywheel: stability at speed, and shots, including a pre-kick just before each ball hits. `Flywheel` learns when balls
  come, from every contact: first one after the blocker opens (logs: 0.24 s), then the gap between them (0.34 s);
- the turret (`TurretHyperTuner`, live through `TurretBase.overrides`): holding (aim still: error, swings, flips, counted double
  while a shot is fed) and tracking (aim moving: error) are scored apart.
It logs everything to the SD card, and `findings.md` names principle candidates. Agents turn those into new or removed
control features by the protocol in
[SYSTEM_EVOLUTION.md](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Tuners/ShooterHyperTuning/SYSTEM_EVOLUTION.md):
evidence first, switchable, sim-tested, and the robot decides.

## Simulation and the robot profile

`Sim` simulates the robot (motors + battery sag, mecanum dynamics, flywheel launcher, ball flight,
turret, odometry drift) and runs predictions (shooting table, Monte Carlo make probability,
stop-to-stop move times). Its results, real-robot measurements (HardwareMonitor sessions, shot and
pose logs) and the robot's build numbers all live in **one SQLite file**,
`Sim/data/robot_profile.db`, which a separate game simulator reads. Every value records its source,
and `profile report` shows what is still missing. Commands are in [AGENTOPS.md](./AGENTOPS.md);
the data dictionary is [Sim/PROFILE.md](./Sim/PROFILE.md); model limits are in DEV_NOTES.

Two "undeterminables" have their own sims and profile categories:

- **Loop timing** (`sim loop`, `control.*`): a loop = your code + every hardware read/write + jitter
  + rare spikes. Controllers run once per loop against the physics (sensor read early, motor
  command late), so the profile gets turn/move settle times, overshoot, flywheel speed at the feed
  (fed into the shot Monte Carlo), odometry integration error, pose age, how late timers end and
  which short sensor pulses get missed. HardwareMonitor sessions fill the loop's parts.
- **Static electricity and disconnects** (`sim esd`, `env.*`, `esd.*`, `electrical.*`,
  `disconnect.*`, `event.*`): charge on four robots (frame + electronics each), the perimeter, metal
  structures and a tile-charge grid, from rolling, sliding, wheel slip, intaking and hits;
  humidity-dependent leakage and sideways creep; every discharge; brownouts, current surges/drops,
  impacts and motor noise; all turned into Control Hub / Expansion Hub / I2C / camera / Wi-Fi
  failures with outage times. The charge books must balance (a test checks it): keep it that way
  when changing `ChargeSim`.

## Version control

**Every new file you create in this repo gets added to git — don't leave work
untracked.** If you add a doc, a Gradle file, an OpMode, a subsystem or a resource,
stage it in the same pass that created it:

```bash
git add <the files you just created or changed>
```

Untracked files are invisible to teammates, get clobbered by branch switches, and
Android Studio flags them as "unversioned" — if you created it on purpose, track it.
This applies to all project sources and configuration: `*.md` docs, `*.gradle` /
`*.gradle.kts`, `*.java` / `*.kt`, manifests, and `res/` files. It also covers the robot
profile, `Sim/data/robot_profile.db` (binary, tracked on purpose), and `Sim/data/templates/`.
`Sim/out/` is ignored.

**Do not add:**

- Anything already matched by [`.gitignore`](./.gitignore) — `build/`, `.gradle/`,
  `.idea/`, `*.iml`, `*.apk`/`*.aar`/`*.dex`, `*.log`.
- `local.properties` — machine-specific SDK paths.
- Real secrets: release keystores, API keys/tokens, `google-services.json` with
  live credentials, anything injected via the `APK_SIGNING_*` environment
  variables. These never go in the repo.

> Note: `libs/ftc.debug.keystore` **is** tracked on purpose. It is the shared FTC
> debug key, not a secret — the `*.keystore` ignore rules are commented out in
> `.gitignore` for exactly this reason. Release signing stays in env vars.

If you are unsure whether something is a secret, ask before staging it. Staging is
cheap to undo (`git restore --staged <file>`); a committed credential is not.

## Where to go next

- **Build, deploy, run, verify** → [AGENTOPS.md](./AGENTOPS.md)
- **Decisions, caveats, gotchas** → [DEV_NOTES.md](./DEV_NOTES.md)
