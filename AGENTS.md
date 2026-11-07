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
  Autos/           Full autonomous routines
  AutoSubsystems/  Auto-only helpers
Constants/         Robot + Pedro Pathing constants, hardware maps
Sim/               Bridge between robot code and the Sim module
  Adapters/        Adapt hardware interfaces to sim implementations
  Interfaces/      Abstractions shared by real + simulated hardware
Systems/           Robot subsystems (drive, mechanisms)
TeleOp/            Driver-controlled OpModes
Testing/           Throwaway / diagnostic OpModes
Tuners/            PID / path / sensor tuning OpModes (pair with FTC Dashboard)
Utility/           Shared helpers (MathUtil, BetterGamepad, RobotLogger, ...)
  Hardware/        Monitored* drop-in hardware wrappers (read caching + I/O stats)
  Monitoring/      HardwareMonitor: loop/power/hub/motor monitoring + on-robot database
                   + MonitorWebServer (web dashboard at :8080/hwmonitor)
  opmodes/         Utility OpModes (Hardware Monitor Viewer, Clear Database, example)
```

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
`NextRobot` (`Systems/AthletechRobot`) with no hardware access in its constructor, and don't use
`BlazeOpMode` (Blaze isn't packaged). Details are in DEV_NOTES.

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

## Turret control and tuning

`Systems/Turret/TurretController` is model based (speed per volt, lag, dead time, friction, and also top speed and
acceleration, because the servos run out of torque long before volts), not a hand-tuned PIDF. Its gains come from `TURRET_BANDWIDTH`. Tune it with the **Turret Autotuner** OpMode: run STATIONARY
first, with the robot still and the turret straight at INIT, then DRIVING (press A, then drive). Each run saves
`/sdcard/FIRST/data/TurretAutotune/...`, and the next build pulls it into `TunedConstants`. **Turret Manual
Tuner** is for dashboard tweaking. `TurretAutotuneEngine` has no hardware calls, so keep it that way and
test it against a simulated turret. Details are in DEV_NOTES.

The turret travels **±200° from home** (20° past straight back each way) (`ShooterConstants.MIN/MAX_TURRET_POSITION_IN_DEGREES`) and the robot may spin
forever. `TurretRoute` (used by `Shooter` and the autotuner's DRIVING mode) follows the aim round continuously and
swings it a full turn around before an end, early enough for the turret's stopping distance. `TurretController`'s plan
never heads for an end faster than it can stop (`setTravel`). Past the limit plus
`TURRET_HARD_LIMIT_MARGIN_DEGREES`, `TurretBase` will only drive back toward home, which protects the wiring.
Flywheel shake is handled by **vibration rejection** in `TurretController`: it measures shake from the error's
second difference and lowers stiffness while the turret shakes, down to `TURRET_MIN_BANDWIDTH`, which DRIVING
derives (`TURRET_SHAKE_BANDWIDTH`: the softest bandwidth that really shook less without holding or tracking worse; equal
to the bandwidth = never soften). The autotuner turns rejection off while it measures.
Holding, the friction push lets go early when the turret comes in much faster than the loop asked for (the friction
guess is too high: it slides on less than it takes to start), and the target rate gets a deadband sized to its measured
noise, so the Pinpoint's heading-rate jitter can't flip the push every loop.
**Lanyard compensation** (`LanyardMap`, `TURRET_LANYARDS_ENABLED`, off for now) learns the volts the lanyards need at
each angle (the Manual Tuner's X sweep) and adds them as feedforward. When it's off it must add exactly 0.

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
