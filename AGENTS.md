# AGENTS.md

Canonical guide for AI coding agents (and humans) working in this repository.
Read this first. Operational procedures live in [AGENTOPS.md](./AGENTOPS.md);
project-specific decisions, gotchas and caveats live in [DEV_NOTES.md](./DEV_NOTES.md).

## What this is

`AthletechBiobuzzV1` is a **FIRST Tech Challenge (FTC)** robot project built on the
**FTC SDK 12.0.0** (DECODE era). It is a standard `FtcRobotController` Gradle
project with an extra desktop/off-robot **simulation** module added alongside the
competition code.

## Modules

| Module | Type | Package / namespace | Purpose |
|--------|------|---------------------|---------|
| `FtcRobotController` | `com.android.library` | `com.qualcomm.ftcrobotcontroller` | Stock FTC SDK app shell. **Do not edit** — it is updated wholesale from upstream. |
| `TeamCode` | `com.android.application` | `org.firstinspires.ftc.teamcode` | The robot app we actually write. OpModes, subsystems, autonomous, tuning. This is what deploys to the Robot Controller / Control Hub. |
| `Sim` | `com.android.application` | `org.athletech.sim` | Standalone simulation app (Kotlin DSL build). Physics/field/visualization for testing logic off-robot. |

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

### `Sim` source layout (`org.athletech.sim`)

```
config/ field/ input/ physics/ robot/ telemetry/ undeterminables/ util/ visualization/
```

## Toolchain & build config

- **Android Gradle Plugin** 8.13.2, **Gradle** 9.1.0
- **compileSdk / targetSdk** 36, **minSdk** 24
- **Java** 8 (`sourceCompatibility`/`targetCompatibility = 1.8`)
- **Kotlin** 2.4.10 (`org.jetbrains.kotlin.android`), `jvmTarget = JVM_1_8` to match Java
- **JDK** 25 runs the build — this is why Gradle must be **≥ 9.1.0** (first Gradle with Java 25 support)
- Shared config lives in `build.common.gradle`; shared dependencies/repos in
  `build.dependencies.gradle` (applied by both `FtcRobotController` and `TeamCode`).

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
| FTC Dashboard | `com.acmerobotics.dashboard:dashboard:0.6.0` | `https://maven.brott.dev/` |
| Pedro Pathing 3 | `com.pedropathing:revhub:3.0.2` (pulls `core`) | Maven Central |
| Pedro tuning | `com.pedropathing:tuning:1.0.1` (pulls Sloth) | Maven Central + `https://repo.dairy.foundation/releases/` |
| Apache Commons Math | `org.apache.commons:commons-math3:3.6.1` | Maven Central |
| Kotlin stdlib | `org.jetbrains.kotlin:kotlin-stdlib:2.4.10` | Maven Central |

FTC Dashboard is declared with `exclude group: 'org.firstinspires.ftc'` because its
POM pulls the FTC SDK at **10.0.0**, which would clash with the pinned 12.0.0.
Everything it needs from the SDK is already declared in `build.dependencies.gradle`.

`com.pedropathing:ftc` is **Pedro 2 only** (last release 2.1.2). Never add it alongside
`revhub`/`tuning` — it silently mixes `core` 2.x and 3.x on the classpath.

## Conventions for agents

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
`*.gradle.kts`, `*.java` / `*.kt`, manifests, and `res/` files.

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
