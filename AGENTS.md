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
| BlazeFTC (beta, **off**) | `dev.anygeneric:blazeftc:0.1.66` + `blazeftc_pedro3:0.1.66` | `https://maven.anygeneric.dev/` |
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

**BlazeFTC** (`dev.anygeneric:blazeftc` + `blazeftc_pedro3`, 0.1.66) is used **the way its README says**
(github.com/owenpmckenna/blaze_ftc). Blaze reads the Pinpoint and the hubs on its own threads and calls you the moment new
data is in; the robot's control runs in those callbacks. Don't wrap it, buffer its data or poll it from the loop: its author
read our old 240-line wrapper (it only timestamped the callbacks and kept everything in the slow loop) and said exactly that.

- **All of it is `Utility/Blaze/BlazeSession`** (begin, drive, mechanisms, run, end) plus `BlazeLocalizer`. `BlazeLocalizer`
  is Blaze's `Pedro3SingleDataLocalizer` with `setPose` added; Blaze's own throws "not implemented", and the GARDEN
  relocalize needs it.
- **Built into every OpMode that drives or runs the shooter, and all of it is OFF** (`BlazeConstants.ENABLED = false`; the
  user doesn't want it used). With it off, every OpMode runs exactly as before, in the same order.
  - **Which:**
    - the TeleOps;
    - every `TeleOpBaseOpMode` tuner (SHT, Flywheel Interpolation, TunerTeleOp, Flywheel/Turret Load, Shot Gate, Turret
      and Drivetrain Autotuners);
    - the six autos (through `AthletechRobot`);
    - Flywheel Tuner, Flywheel Voltage Filter Tuner, GeneralVeloMotor Tuner, Turret Manual Tuner.
  - **Not:** Pedro's own tuners (they run `follower.update()` inside Pedro's procedures), `Testing/` testers and the
    utility OpModes.
  - **TeleOp base:** `usesBlaze()` opts in. `updateFollower()` and `runDrive()` do the loop's share; `driveUpdate()` /
    `mechanismUpdate()` are the callbacks.
  - **The autotuners:** the Turret Autotuner's engine runs in the Expansion Hub callback. The Drivetrain Autotuner's runs
    in the Pinpoint callback and drives through `follower.manual`.
  - **Autos:** `Drive` starts paths and holds under the lock, so the follower is only changed between Blaze's updates (its
    README warns about that). The auto Shooter/Intake have `control()` for the callback.
- **Where things run:**
  - `driveUpdate()`, on each Pinpoint sample through `Pedro3SingleDataLocalizer.setup`: the drive from the sticks, then
    `follower.update()`. With Blaze, Blaze owns `follower.update()` and nothing else may call it. The drive goes out through
    `follower.manual(...)`, the same wheel mixing as `DriveController`, and Blaze writes it after the callback (a direct
    motor write would be overwritten by the follower's stop).
  - `mechanismUpdate()`, on each Expansion Hub bulk read (`engageBulkReadAcceleration`, Hub.ExHub, where the flywheel,
    turret encoder and intake are): `flywheel.update()`, `turret.update()`, the intake's speed loop.
  - The loop: gamepads, buttons (once per press), blocker, aim and flywheel targets (`Shooter.update` with
    `mechanismsElsewhere`), telemetry, recording. The Control Hub cache is cleared each loop; the Expansion Hub's isn't,
    because Blaze fills it.
- **One lock** (`BlazeSession.LOCK`): both callbacks and every loop body (and every follower change in the autos) take
  turns on the robot's objects. Never `sleep` while holding it.
  `TemporalLog.record` is synchronized because hardware calls now come from Blaze's threads.
- **Lifecycle:**
  - `begin` (initialize, before any hardware) in `initializeComponents`;
  - the hooks in `setUpLynxModule`;
  - `run` on the first loop;
  - `end` (`closeBlazeFTC`, or the robot restarts) in `stop` and again in `MonitorOpModeHook` after every OpMode.
  - Blaze's motors stay in the hardwareMap: no swapping SDK motors back.
- **Switches:**
  - `BlazeConstants.ENABLED` is **false** until the new way passes `python -I tools/blaze_check.py` on the robot (callback
    rates, gaps, loop times, lag, pose, turret, intake). The old way froze loops 150-650 ms about every second
    (2028-05-12).
  - `MOTOR_ACCELERATION` is off: Blaze's motor writes skip RUN_USING_ENCODER, which the intake's speed loop sits on (it ran
    under half speed). Blaze's README calls faster writes secondary anyway.
- **Recorded:** `blaze.drives`, `blaze.feeds_eh`, `blaze.drive_updates`, `blaze.mechanism_updates`, faults.

Details and what's unproven are in DEV_NOTES.

**Temporal Log Red** (`TeleOp/TemporalLogRedTeleOp`) is the RED TeleOp exactly, with everything logged against time by
`Utility/TemporalLog`:

- every hardware read and write, with its value and duration (`DeviceTracker` and HardwareMonitor's hub samples);
- every variable of the TeleOp's object graph and of the Constants, whenever it changes (`VariableWatcher`);
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

**Match video review** (`tools/shot_video.py`, steps in [docs/VIDEO_SHOT_REVIEW.md](./docs/VIDEO_SHOT_REVIEW.md)): when the user
sends a phone video of a match with its logs, sync it to the runs (flywheel-dip balls against sound onsets, marked launches or
an anchor; one offset per run, frame timestamps not frame numbers), look at every ball's frames, mark score/short/long/...,
and `fit` turns every review's marks into a small `FLYWHEEL_VELOCITIES` change (max 25 ticks/s a point) or flywheel recovery
advice.

The first review (`tuning-data/video-reviews/sht-20280515`, the old SHT run) put `FLYWHEEL_VELOCITIES` 8 ticks/s under the
old table (1436 at 38 in, 1482 at 54): on-target balls arrived high on the CELL.
- **Exit gain:** how wide the window is in wheel speed (exit gain 0.3-1) isn't known yet, so the fit keeps every gain the
  balls allow.
- **Trust:** it never puts the table more than 10 outside the speeds seen to score. Its first cut (−23, assuming gain 1) was
  below every ball that scored.
- **Probes:** the SHT shoots some bursts off the table to measure the window.
- **Contacts:** newer logs list every ball the observer saw. The old dip detector missed later balls of a burst.

**Field measurements** are all in [docs/FIELD.md](./docs/FIELD.md), from the manual's figures. Re-extract them with
`tools/manual_figures.py`. The field CAD is Onshape and needs a login. The CELL opening is a 20 in wide, 14 in tall pentagon
(a peaked roof above 7.61 in sides) leaning back 30°. So the speed range that goes in depends on the approach angle, not
just the distance: from the side, the fast side shrinks. `tools/field_geometry.py` models it, and the video fit judges
every ball by its own window. Account for it whenever the flywheel table is tuned.

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
and swings a full turn only once the aim is past an end (`TURRET_SWING_MARGIN_DEGREES`). An aim that jumps over 90° in one
loop (a relocalization) is re-picked as the in-travel angle nearest the turret: following it round sent the turret to the far
end and then a full turn back (~1 s lost, half the logged relocalizations).
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

## Flywheel control

The wheel **can't be braked**: with the power cut it only coasts down (~100 ticks/s², 2028-05-15 logs). So any overshoot
lasts ~0.4 s, and the old controller overshot +30..+55 after every ball (its filter lags and runs past the end of a
recovery). Second balls came in a median +21 fast and went long, slow ones went short (old SHT video, 49 balls marked).

- **`FlywheelObserver`** (`FLYWHEEL_OBSERVER`, on) is what the controller runs on. It carries the hub's reading (a 50 ms count
  read ~35 ms late) to now with the volts sent and the coast-only model. Its disturbance is the load, and a reading far under
  the model while a ball can hit is a contact. It also dates each ball and learns how much speed one takes.
  - It always runs (contacts, logs, the gate), even with the switch off.
  - Sim (`FlywheelBurstTest`, real `Flywheel` on the logged wheel, windows at exit gain 0.6): 40 misses in 150 balls → 0, p95
    46 → 9 ticks/s. 0 misses with the wheel off from the model every way the logs allow (accel 120-210, coast, lag, kv ±5%,
    10 V, sags, 50 ms loops). Balls 0.2 s apart still miss: they're past the blocker before the dip shows.
  - **Not fixed by the wheel:** the old video's first balls missed as if balls varied ~14 ticks/s on their own. If that's
    real, the sim says ~4-5% still miss (`oldSystemMissesLikeTheVideo`).
- **`FeedGate`** (`FEED_GATE`, **off** in the match TeleOps, on in the SHT) holds a ball the wheel won't be inside the shot's
  window for when it arrives. Balls already past the blocker (`FEED_GATE_COMMIT`) count against the prediction.
  - The window is `CellGeometry`'s (the CELL pentagon, drag-free, `Shooter.cell`) for where the turret is, ×
    `FEED_GATE_WINDOW_SCALE` (0.6, the exit gain until the SHT probes measure it), kept in 25..80 ticks/s.
  - In normal shooting it holds a few times in 150 balls. At 10 V with sags and heavy balls: 22 misses → 0, 0.29 → 0.39 s
    a ball.
  - It never shuts for a later ball before the first ball of an opening is surely past the blocker (`firstGuard`). Doing
    that stopped both, every time, and a badly tuned wheel got almost no balls out (SHT sim: 9.4 s a ball → 0.42).
  - `Flywheel` learns the first ball's time only from openings with nothing still in flight.
  - `starved` counts trigger presses it let no ball out of. The SHT scores those against the knob value on trial.
  - Off in matches until an SHT run backs it: the old ShotGate stuttered the autos.
- **Gains from the logs** (docs/VIDEO_SHOT_REVIEW.md):
  - `tools/flywheel_fit.py` fits the wheel and the balls from the logged speed, power and supply into the Sim.
  - `FlywheelTuning` (`FLYWHEEL_TUNE=1`) tries small gain steps there.
  - The 2028-05-15 logs show balls **0.17 s apart**, and feeding stays that fast (the user's call). No gain, gate, kick, burst
    lift (`FLYWHEEL_BURST_LIFT_SHARE`) or continuous burst push (`FLYWHEEL_BURST_FEED_SHARE`) scores those reliably. Both are
    off, and the SHT tries them: the balls scatter (gaps 0.08-0.25 s, drops 40-180), and those only move the average.
  - The Sim's miss rate on those balls runs from 2% to 29%, depending on the window width the SHT probes will measure.
    At 0.30 s it would be 0.3%.
- **Aim point:** the top middle of the opening's rectangle (7.61 in up the face, `FieldConstants` goals), not its middle.
  `docs/FIELD.md`.

## Hypertuning and system evolution

**SHT Stability Tuner** (`Tuners/ShooterHyperTuning`) is the full RED TeleOp with the feed gate on. While it's played it
keeps tuning, with `OnlineTuner`:
- the flywheel. **Shots** (`kp_close`, `accel_per_volt`, `observer_lag`, `kp_far`, `observer_gain`, `max_p`): every ball the
  observer sees hit scores its speed error over that shot's window (1 = at the edge, past it = a predicted miss; later balls
  weigh more), plus the gate's holding and the overshoot after. **Stability** (`kd`, `observer_load_gain`): holding at speed,
  on the hub's raw reading. `Flywheel` learns when balls come (first after the blocker opens, then the gap);
  - telemetry "Balls" shows how many hit outside their window;
  - **paired trials:** each trial alternates the new and old value burst by burst (ABBA) and judges the differences, so the
    battery running down can't fake a better knob (the 2028-05-15 baseline drifted 9.1 → 11.3 while 6 changes were kept). It
    measures until clearly better or clearly not (1.5 sigma, noise learned across trials);
  - sessions only resume from ones with `format.txt` = `SHTStabilityTuner.FORMAT` (2), so older gains don't come back;
  - a trigger press the gate let no ball out of counts as a burst at the window's edge plus the wait, so a value the gate
    won't feed can't stay on unjudged;
  - **probes** (`PROBE`, on): some bursts shoot off the table (`PROBE_OFFSETS`, −35..+12) so the video measures the window
    (docs/VIDEO_SHOT_REVIEW.md). Some of those balls miss on purpose;
  - sim: `FlywheelBurstTest.shtTunerFixesABadModelInOneSession` plays 15 min from a bad start: every ball still goes, 3 of 540
    miss, and the gate's waiting falls 39 → 30 s a third;
- the turret (`TurretHyperTuner`, live through `TurretBase.overrides`): holding (aim still: error, swings, flips, counted double
  while a shot is fed) and tracking (aim moving: error) are scored apart.
It also keeps a **temporal log** (`TEMPORAL_LOG`, on): every read/write and every variable of the OpMode (shooter, flywheel,
turret, blocker, intake, pose, both tuners) plus the shooter Constants. Every 10 s it drops a `sync` mark in both the temporal
log and the robot log, as Temporal Log Red does, so `shot_video.py link` lines it up exactly with a video of the session.
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
