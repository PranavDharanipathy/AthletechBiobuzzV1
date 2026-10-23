# DEV_NOTES.md

Project decisions, gotchas and caveats. Referenced by [AGENTS.md](./AGENTS.md).
Newest notes at the top.

## 2026-10-23 — Hardware monitor web dashboard

- `Utility/Monitoring/MonitorWebServer` registers pages on the Robot Controller's built-in web
  server (the one serving OnBotJava/Blocks) via `@WebHandlerRegistrar`, like FTC Dashboard.
  No extra app, module or run configuration. Open `http://192.168.43.1:8080/hwmonitor`
  (Control Hub) or `http://192.168.49.1:8080/hwmonitor` (phone RC) on the robot's Wi-Fi.
- The page is one self-contained file, `TeamCode/src/main/assets/hwmonitor/index.html`
  (no CDN: the robot's Wi-Fi has no internet). Opened straight from disk it switches to
  local mode: "Open folder…" on a copy from `adb pull /sdcard/FIRST/data/HardwareMonitor`.
- Tabs: Overview (KPIs + automatic findings), Timeline (system.csv, synced drag-zoom),
  Motors, Devices, Events (every read/write: rates, latency scatter + histogram, slowest
  calls), Tables (every number, sortable/filterable, CSV/JSON export), Compare (sessions).
- A recording session updates live every 2 s: the page polls `/api/live` and appends to the
  CSVs from a byte offset (`X-Next-Offset`), so nothing is re-downloaded.
- API: `GET /hwmonitor/api/sessions`, `GET /hwmonitor/api/file?session=&name=&offset=`,
  `GET /hwmonitor/api/live`, `POST /hwmonitor/api/delete?session=`, `POST /hwmonitor/api/clear`.
  File names are whitelisted and session ids validated (no path traversal). The recording
  session cannot be deleted.
- Verified in the container: real OpenRC `NanoHTTPD` serving `MonitorWebServer`, simulated
  sessions (incl. one live), driven by headless Chromium — all tabs, sort/filter/export,
  live updates, delete/clear, file:// mode; no console errors. **Not yet run on the robot.**
- If the page ever 404s on the robot, check the RC log for "HardwareMonitor dashboard
  available" — that line means the handlers registered.

## 2026-10-23 — Hardware monitor + Monitored* hardware wrappers

New code (26 files):
- `Utility/Hardware/` — drop-in wrappers: `MonitoredDcMotor`, `MonitoredDcMotorEx`,
  `MonitoredServo`, `MonitoredCRServo`, `MonitoredIMU`, `MonitoredDistanceSensor`,
  `MonitoredColorSensor`, `MonitoredTouchSensor`, `MonitoredDigitalChannel`,
  `MonitoredAnalogInput`, `MonitoredVoltageSensor`; plus `DeviceStats`, `DeviceTracker`,
  `HwOp`, `CacheSlot`, `MonitoredDevice`.
- `Utility/Monitoring/` — `HardwareMonitor` (singleton), `MonitorStore` (database),
  `MonitorOpModeHook` (lifecycle), `MotorElectricalModel`, `RunningStats`, `Histogram`,
  `EventBuffer`.
- `Utility/opmodes/` — `HardwareMonitorViewer`, `HardwareMonitorClear`,
  `HardwareMonitorExample` (`@Disabled`; device names must be edited before use).
- SDK 12.0 added `PwmControl.setPulseWidth(us)` / `getPulseWidth()`. `MonitoredServo` and
  `MonitoredCRServo` pass them through and track them like `setPosition`/`setPower`
  (repeated width = redundant write; on a CR servo it also clears the cached power).

### Read caching
- Constructor boolean (`new MonitoredDcMotorEx(hardwareMap, "lift", true)`) or
  `setCachingReads()` at runtime.
- A cached value is valid until the next `HardwareMonitor.get().update()` (one "cycle"), and
  never longer than `CONFIG.cacheMaxAgeMs` (50 ms) — the age limit is a safety net for code
  that forgets `update()`.
- Writes invalidate only the readings they can change (`setPower` → power + busy;
  `setMode`/`setDirection`/`setMotorType` → everything; PIDF → nothing). A read after a
  write in the same loop therefore always sees the new state.
- Current is cached in amps and distance in mm, so one transaction serves every unit.
  `fromMm` preserves `DistanceSensor.distanceOutOfRange`.
- Complements REV bulk caching rather than replacing it: bulk caching makes encoder,
  velocity, digital and analog reads cheap; IMU, color, distance, current and voltage reads
  are separate transactions that only this cache helps. `CONFIG.bulkCachingMode = MANUAL`
  makes the monitor set hubs to MANUAL and clear them in `update()`.

### What is measured, and what it costs
- Every wrapped call: count, cache hit / redundant write, latency (`System.nanoTime`),
  recent (1 s) and average frequency, first/last time, last 256 events in memory, and
  every event in `events.csv`.
- Counting is a few atomic increments: well under 1 µs per call, no allocation.
- The monitor's own sampling (battery, 6 metrics per hub, motor current + velocity) is
  limited to `CONFIG.maxSampleReadsPerUpdate` = **1 hardware read per `update()`**,
  most-overdue first, and reuses any value your code already read through a wrapper.
  Expect roughly +1–2 ms on about a quarter of loops; set it to 0 for zero sampling overhead.
  The cost is reported as `monitor.overheadMsPerLoop` in every summary.
- All file I/O runs on a low-priority background thread every 250 ms. `events.csv` stops at
  64 MB per session (`CONFIG.maxEventLogBytes`) so long practices cannot fill storage.

### Back-EMF
REV hubs cannot measure it, so it is estimated (`MotorElectricalModel`):
1. **From velocity** (primary, always on): `E = 12 V × velocity / free speed`, using
   `getMaxRPM()` and `getTicksPerRev()` from the motor type **in the robot configuration**.
   If the configured motor type is wrong, back-EMF is wrong by the same ratio.
2. **From current** (only after `motor.setWindingResistance(ohms)`): `E = duty×V − I·R`.
   Not reported at |power| < 0.02 or in velocity mode, where the duty cycle is unknown.
`bemfToBattery` near 1.0 means the motor has run out of voltage headroom.

### Sessions and the lifecycle hook
- `MonitorOpModeHook` registers through `@OnCreateEventLoop` and `OpModeManagerImpl`, the
  same mechanism FTC Dashboard uses. It opens a session when the first `Monitored*` device
  is created during INIT and saves it at stop — even after a crash or a Driver Station stop.
- The default `$Stop$Robot$` OpMode and OpModes that touch no monitored hardware (the
  viewer and clear OpModes included) never create a session.
- If the hook is not active (summary shows `"lifecycleHook": false`), call
  `HardwareMonitor.start(this)` first in init and `HardwareMonitor.get().stop()` at the end.
- `summary.json` is rewritten every 5 s and written atomically, so a power loss leaves at
  most 5 s of summary missing. A session without an index row shows as `*` (incomplete).

### Verification status
- Compiled with `javac --release 8` against stub SDK classes whose signatures were checked
  against the RobotCore/Hardware **12.0.0** javadocs and OpenRC sources.
- Exercised end-to-end in a simulation (fake motor/IMU/servo/hubs with realistic latencies;
  init → start → 400 loops → stop; viewer and clear OpModes driven by scripted gamepad):
  32/32 checks passed, including cache-hit counts, redundant writes, event-log
  completeness, energy integration, and back-EMF within 1% of the analytic value.
- **Not yet run on the real robot or built with Gradle.** First real build is the check.

### Known limits
- Not wrapped: goBILDA Pinpoint, SparkFun OTOS, Limelight, webcams. Their timing still shows
  up as loop time, just not attributed to a device.
- No write caching: redundant writes are *counted* (`redundantWrites`) but still sent. If
  that number is large for a device, write caching for it is an easy win.
- Servo `getPosition()` returns the last commanded position (servos have no feedback).
- Storage is under `/sdcard/FIRST/data`. With `targetSdk 36` on a phone Robot Controller
  running Android 11+, writes there may be refused (scoped storage). Failures are logged and
  reported as `monitor.lastWriteError` and never crash the OpMode; the Control Hub (Android
  7) is unaffected.

## 2026-10-22 (evening) — first real builds: manifest, Pedro 3, Kotlin 2.4.10

These supersede parts of the earlier entry below; where they disagree, this wins.

### Build failure 1 — `processDebugManifest` (`android:exported`)

`./gradlew clean :TeamCode:assembleDebug` failed in
`:FtcRobotController:processDebugManifest`: *"android:exported needs to be explicitly
specified … Apps targeting Android 12 and higher …"*. This is a direct consequence of
`targetSdk 36` (anything ≥ 31). The stock FTC manifest was written for `targetSdk 28`.

Fix: added `android:exported="true"` to the two activities with intent-filters in
`FtcRobotController/src/main/AndroidManifest.xml`:
- `PermissionValidatorWrapper` — LAUNCHER/MAIN, must be exported to be launchable.
- `FtcRobotControllerActivity` — `USB_DEVICE_ATTACHED`, must be exported to receive it.

(FTC had already set it on the `RunOnBoot` receiver.) **This is an edit to vendor
code** — an FTC SDK update will overwrite the manifest and the build will fail the same
way until it's re-applied.

### Build failure 2 — `compileDebugKotlin` (stdlib 2.4.10 vs compiler 2.2)

With the Kotlin plugin at 2.2.20, compilation failed: *"The binary version of its
metadata is 2.4.0, expected version is 2.2.0"*, cascading into `Unresolved reference
'ArrayList' / 'sin' / 'PI'` across `Systems/FIArm/*.kt`.

Root cause chain: `com.pedropathing:tuning:1.0.1` → `dev.frozenmilk.sinister:Sloth:0.3.2`
(runtime) → `kotlin-stdlib:2.4.10`. AGP aligns compile and runtime classpaths, so 2.4.10
becomes the compile stdlib too. A Kotlin compiler reads metadata at most one minor
version ahead (2.2 → 2.3), so 2.2.x can never compile against it.

Fix: Kotlin plugin **and** declared stdlib → **2.4.10**. KGP 2.4.10 supports Gradle
7.6.3–9.5.0 and AGP 8.5.2–9.1.0, covering this project's Gradle 9.1.0 + AGP 8.13.2.
(Pedro's own migration notes call the 2.4 metadata "lint-only noise" — true for
Java-only TeamCode, false once TeamCode contains `.kt` sources.)

### Pedro Pathing 2 → 3

`com.pedropathing:ftc` was **retired after 2.1.2**. Pedro 3 is `revhub` (pulls `core`)
+ `tuning`. The build had `ftc:2.1.2` *and* `revhub:3.0.2`, so Gradle resolved
`core` to 3.0.2 under the 2.x `ftc` artifact. Removed `ftc:2.1.2`.
Also converted the dairy.foundation repo line to `url = '…'` (Gradle 9 deprecation).

### Why AGP stays at 8.13.2 (not 8.7.0)

The build runs on **JDK 25**. Java 25 needs **Gradle ≥ 9.1.0** (Gradle 8.9 caps at
Java 22). AGP 8.7.0's min/default Gradle is 8.9 and it predates Gradle 9, and it was
only tested to `compileSdk 35`. The FTC-stock stack (Gradle 8.9 + AGP 8.7.0 + Kotlin
2.2.20) is only viable if the Gradle JDK is switched to 17 or 21 — *and* it would
still need Kotlin ≥ 2.3 because of Sloth's stdlib 2.4.10.

### Still open

Some `FIArm` errors may be real code bugs rather than stdlib fallout — e.g.
`Field.kt:42` (*expected List<Obstacle>, actual String*), `SectorPrism.kt:46–53`
(Double/String mismatches), `SectorPrism.kt:61` (*Unresolved compareTo*). Re-check
them once the stdlib error is gone.

## 2026-10-22 — SDK 36 bump, new deps, Kotlin, Gradle 9 cleanup

### SDK versions → 36

`compileSdk` and `targetSdk` are now **36** in all three places that set them:
- `build.common.gradle` (feeds `TeamCode` via `apply from`)
- `FtcRobotController/build.gradle`
- `Sim/build.gradle.kts`

All three had to move together because they were inconsistent (`compileSdk 30` /
`targetSdk 28` in the FTC modules, `30`/`30` in `Sim`) and **`targetSdk` may never
exceed `compileSdk`**. Raising only `targetSdk` to 36 would have failed outright.

While touching those lines they were also converted from the deprecated
`compileSdkVersion` / `minSdkVersion` / `targetSdkVersion` method form to the
`compileSdk = ` / `minSdk = ` / `targetSdk = ` property form (AGP 8's documented
form, and the only one that survives into AGP 9).

> **Control Hub caveat:** FTC historically kept `targetSdk 28` *on purpose*, because
> the Control Hub runs an older Android. Raising `targetSdk` to 36 opts the RC app
> into newer runtime behaviors (runtime permissions, background execution limits,
> scoped storage). It compiles and packages fine, but if odd permission or
> background-behavior problems show up on the Control Hub, **this is the first
> suspect**. The fix is to lower `targetSdk` back toward 28 — `compileSdk` can stay
> at 36 independently.

### Kotlin added — and why the version was 2.3.21 *(superseded: now 2.4.10, see above)*

`org.jetbrains.kotlin.android` **2.3.21** is declared `apply false` in the root
`build.gradle` and applied in `TeamCode/build.gradle` and `Sim/build.gradle.kts`.
`kotlin-stdlib:2.3.21` is in the shared dependencies file.

The version is **not** arbitrary. The Kotlin Gradle plugin has a hard
compatibility matrix against Gradle and AGP, and this project runs **Gradle 9.1.0 +
AGP 8.13.2**:

| Kotlin plugin | Supported Gradle | Supported AGP |
|---|---|---|
| 2.1.0–2.1.10 | 7.6.3 – 8.10 | 7.3.1 – 8.7.2 |
| 2.1.20–2.1.21 | 7.6.3 – 8.12.1 | 7.3.1 – 8.7.2 |
| 2.2.x | 7.6.3 – 8.14 | 7.3.1 – 8.11.1 |
| **2.3.20–2.3.21** | **7.6.3 – 9.3.0** | **8.2.2 – 9.0.0** |
| 2.4.20–2.4.21 | 7.6.3 – 9.7.0 | 8.5.2 – 9.3.1 |

Anything in the 2.1.x/2.2.x range is **below** this project's Gradle version, so
2.3.21 is the lowest version that fully supports Gradle 9.1.0 *and* AGP 8.13.2.
If you upgrade Gradle or AGP, re-check this table first.

Note that Pedro Pathing requests `kotlin-stdlib:2.1.20` transitively; the explicit
`2.3.21` declaration wins and keeps the compiler and stdlib on the same version
(a newer stdlib than compiler produces a noisy warning).

`jvmTarget` is set to `JVM_1_8` in both modules via the modern
`kotlin { compilerOptions { … } }` DSL (the old `kotlinOptions` block is
deprecated). This must stay aligned with the Java 8 `compileOptions` — a
Java/Kotlin JVM-target mismatch is a **hard build error**, not a warning.

### New dependencies

In `build.dependencies.gradle`, which is applied by both `FtcRobotController` and
`TeamCode`:

- **FTC Dashboard** `com.acmerobotics.dashboard:dashboard:0.6.0`, from
  `https://maven.brott.dev/` (that repo was added here; it 302-redirects to Azure
  blob storage, which Gradle follows fine). Declared with
  `exclude group: 'org.firstinspires.ftc'` because **its POM depends on the FTC SDK
  at 10.0.0** while this project pins 12.0.0. Without the exclusion you get two
  versions of RobotCore/Hardware/RobotServer/FtcCommon competing. All four are
  already declared here, so excluding the group loses nothing.
- *(superseded: Pedro 3 uses revhub + tuning, see above)* **Pedro Pathing** `com.pedropathing:ftc:2.1.2`, now on **Maven Central** — no
  custom repository needed (older guides point at JitPack; that's out of date).
  Its only dependencies are `com.pedropathing:core:2.1.2` and `kotlin-stdlib`. It
  does **not** bundle the FTC SDK or Dashboard, so it needs no exclusions.
- **Apache Commons Math** `org.apache.commons:commons-math3:3.6.1`, also added to
  `Sim` so sim and robot code share the same geometry/linear-algebra library.

### Gradle 9 deprecation cleanup

The project is on **Gradle 9.1.0**, which removed Groovy *space-assignment*
(`propName value`) for Gradle-generated properties — scheduled to become an error
in Gradle 10. The build's own problems report listed 10 warnings; 4 were in these
files and are now fixed with `=`:

| Where | Was | Now |
|---|---|---|
| `build.common.gradle` `defaultConfig` | `signingConfig signingConfigs.debug` | `signingConfig = …` |
| `build.common.gradle` `buildTypes.release` | `signingConfig signingConfigs.release` | `signingConfig = …` |
| `build.common.gradle` | `ndkVersion '21.3.6528147'` | `ndkVersion = '…'` |
| `TeamCode/build.gradle` | `jniLibs.useLegacyPackaging true` | `jniLibs.useLegacyPackaging = true` |

The other 6 warnings are **inside AGP itself** (`com.android.internal.library` /
`…internal.application` declaring `lint-gradle` and `aapt2` with multi-string
notation). They are not fixable from this repo — they go away with a future AGP.
Don't chase them.

Deliberately left alone: `keyAlias`, `keyPassword`, `storeFile`, `storePassword`,
`applicationId`, `versionCode`, `versionName`, `pickFirst`, `abiFilters`,
`debuggable`, `jniDebuggable`. These are genuine AGP DSL *methods*, not
Gradle-generated properties, so the space-assignment removal doesn't apply — the
problems report confirms it never flagged them.

### Other

- `gradle.properties`: raised `org.gradle.jvmargs` from `-Xmx1024M` to `-Xmx4096m`.
  1 GB is too small for Kotlin compilation once Pedro Pathing, Dashboard and
  commons-math are on the classpath, and tends to fail with "Java heap space".

## Standing notes / gotchas

- **`FtcRobotController` is vendor code.** It is replaced wholesale on SDK
  upgrades. Never put team logic there — it lives in `TeamCode`. Note that
  `build.common.gradle` is also upstream-owned, so the edits above will need
  re-applying after an FTC SDK bump.
- **Two application modules.** Both `TeamCode` (`com.qualcomm.ftcrobotcontroller`)
  and `Sim` (`org.athletech.sim`) are `com.android.application`. They build
  independent APKs; only `TeamCode` goes on the robot.
- **`Sim` uses the Kotlin Gradle DSL** (`build.gradle.kts`) with the newer
  `compileSdk { version = release(36) }` block; the FTC modules use the Groovy DSL
  with `compileSdk = 36`. Same result, different syntax — don't "fix" one to match
  the other.
- **`TeamCode/Sim/` is not the `Sim` module.** The `Sim` package inside `TeamCode`
  (`Adapters/`, `Interfaces/`) bridges robot code to the standalone `Sim`
  simulation app. Keep that boundary clean so robot code never hard-depends on the
  simulator.

## TODO / open questions

- Confirm the Pedro Pathing **2.x** follower/localizer API matches whatever lands
  in `Constants/` and `Systems/` — 2.x changed APIs relative to 1.x.
- `TeamCode` is currently near-empty scaffolding: every package exists but only
  `Utility/MathUtil.java` (a stub) has a file in it. The `Sim` module's packages
  (`physics/`, `field/`, `visualization/`, …) are likewise empty.
- Decide whether `Sim` should depend on shared robot interfaces directly or stay
  fully decoupled behind `TeamCode/Sim/Adapters`.
- `ndkVersion` is pinned to the old `21.3.6528147`. It's the FTC default and there
  is no native code in this repo, but it must be installed locally or the build
  will ask for it.
