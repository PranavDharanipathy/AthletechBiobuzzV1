# AGENTOPS.md

Operational playbook for building, deploying, running and verifying changes.
Referenced by [AGENTS.md](./AGENTS.md).

## Prerequisites

- Android Studio (current stable) with Android SDK **API 36** installed.
- NDK `21.3.6528147` (pinned in `build.common.gradle`).
- A Robot Controller device (Control Hub or an Android phone running the RC app)
  for on-robot deploys. Off-robot work can use the `Sim` module only.

## Build

From the repo root:

```bash
# Build the competition app
./gradlew :TeamCode:assembleDebug

# Build + test the desktop sim module (plain JVM, no device needed)
./gradlew :Sim:build

# All Android modules
./gradlew assembleDebug
```

Always invoke it as `./gradlew`, including on Windows. A bare `gradlew` fails in
PowerShell, which does not execute from the current directory without an explicit
`./` prefix. `./gradlew` works in PowerShell and Git Bash alike.

First build after the SDK/dependency bump will re-resolve Pedro Pathing, FTC
Dashboard and commons-math — it needs network access to Maven Central and
`https://maven.brott.dev/`.

## Deploy to the robot

- Preferred: Android Studio **Run** on the `TeamCode` configuration with the
  Robot Controller connected via USB (or Wi-Fi ADB to the Control Hub).
- CLI: `./gradlew :TeamCode:installDebug` with the device attached.
- The APK's `applicationId` is `com.qualcomm.ftcrobotcontroller`; it replaces the
  stock RC app on the device.

## Run & tune

- Select OpModes on the Driver Station from the `@TeleOp` / `@Autonomous` groups.
- **FTC Dashboard**: with the robot on its Wi-Fi, browse to
  `http://192.168.43.1:8080/dash` (Control Hub) to see live telemetry and edit
  `@Config` values. Tuning OpModes live in `TeamCode/.../Tuners`.
- **Pedro Pathing**: path/localizer tuners also surface through Dashboard; keep
  tuned constants in `Constants/`.
- **Drivetrain Autotuner** (tuning group): the robot drives itself for about a minute (forward/back, strafe, spin, then
  shoves itself while holding), with about 3 ft clear every way. Any stick or B stops it. The result goes into
  `TunedConstants.DRIVE_*` on the next build. Run it again after drivetrain changes or if the hold feels soft or twitchy.
- **SHT Stability Tuner** (tuning group): play it like a match. Flywheel and turret gains keep changing, and sessions land
  in `tuning-data/hypertuning/` on build. Read `findings.md`, then follow `Tuners/ShooterHyperTuning/SYSTEM_EVOLUTION.md`.
  Never change a match default from one session.
- **Temporal Log Red** (A_Match): the RED TeleOp with everything logged against time (every read/write with value and
  duration, every variable change, Blaze's background reads, decisions).
  - **Read a session:**
    1. `python tools/temporal_log.py pull`
    2. Then `summary latest`, `decisions latest`, or `csv latest --grep flywheel -o out.csv` (see `--help`).
  - Sessions are big (hundreds of KB/s) and go to `temporal-logs/`, which is git-ignored.
  - Its `temporal.watch_us` record is the logging's own cost per loop.
- **Blaze** runs the hub link for every robot OpMode except testers, utility OpModes and Pedro's tuners. The `Blaze`
  telemetry line says what's on.
  - To go back to the stock SDK for TeleOps and tuners, set `BlazeConstants.ENABLED = false` on the Dashboard before INIT.
  - The autos always go through NextFTC's `BlazeOpMode`.
  - If a motor won't move in a tester after a Blaze OpMode, restart the robot and tell an agent (see DEV_NOTES).
- **Limelight Lab** (live vision tuning; moves nothing): run the `Limelight Lab` OpMode
  (Diagnostics). While it runs, the hub forwards ports 5800-5809 to the Limelight, so a laptop
  on the robot's Wi-Fi gets the web UI at `http://192.168.43.1:5801`, the stream at `:5800` and
  REST at `:5807`. Drive it with `python tools/limelight.py ...` (find, watch, get, set, sweep,
  record, inputs, pull, push, snap). Pipelines and SnapScripts live in
  `TeamCode/src/main/assets/limelight/p<index>_<name>.json|.py`. `push` backs up the camera's
  pipelines into `tools/out/` first and asks before overwriting; hold A+B in the OpMode does the
  same upload from the APK. An agent with Claude in Chrome can open the web UI and tune with the
  user; see `Systems/Vision/README.md` Part 6. Never run `push --yes` or `set --save` without the
  user asking: both overwrite tuned pipelines on the camera.

## Sims and the robot profile (desktop, no robot needed)

All commands go through Gradle from the repo root; arguments go in `--args="..."`.
Paths given to the commands are relative to the `Sim/` folder. Full list:
`./gradlew :Sim:run --args="help"`.

```bash
./gradlew :Sim:run --args="profile report"              # what is filled / missing
./gradlew :Sim:run --args="profile set robot.mass_kg 13.4 --source measured"
./gradlew :Sim:run --args="profile import-monitor C:/path/to/HardwareMonitor"   # after adb pull
./gradlew :Sim:run --args="profile import-shots data/my_shots.csv"
./gradlew :Sim:run --args="profile import-disconnects data/my_disconnects.csv"   # one row per match / disconnect
./gradlew :Sim:run --args="sim all --save"              # drive, localization, loop, launcher, esd -> profile
./gradlew :Sim:run --args="sim loop"                    # loop timing only
./gradlew :Sim:run --args="sim esd"                     # static charge + disconnects only
./gradlew :Sim:run --args="predict shot --distance 2.0 --ball pollen"
./gradlew :Sim:run --args="predict loop --sensor-reads 3 --motor-writes 6"   # what-if on the loop
./gradlew :Sim:run --args="predict disconnect --humidity 20 --strap 0"
./gradlew :Sim:run --args="profile export --format json" # for engines without SQLite
./gradlew :Sim:run --args="profile import-autotune"     # newest Turret Autotuner run -> turret.* (measured)
./gradlew :Sim:run --args="sim turret --assume"         # robot's real turret code vs a sim turret
./gradlew :Sim:run --args="sim flywheel --assume"       # robot's real Flywheel + ShotGate vs sim flywheel
./gradlew :Sim:test                                     # unit tests
```

**Running sims yourself (agents):**

- Build the launcher once, then call it directly, no Gradle startup each time:
  `./gradlew :Sim:installDist`, then from `Sim/`:
  `JAVA_OPTS="--enable-native-access=ALL-UNNAMED" build/install/sim/bin/sim.bat sim turret --assume`
  (`bin/sim` on Linux/macOS). Rebuild after changing Sim or the TeamCode classes it uses.
- `--assume` makes every sim runnable today. It copies the shared profile to
  `out/assumed_profile.db`, fills only the missing entries with PLACEHOLDER examples, and runs
  there. Real values win, the shared profile isn't touched, and the output lists every placeholder used.
  Report those placeholders with any number you quote.
- `sim turret` / `sim flywheel` are **code in the loop**: they run the robot's real classes
  (`TurretController`, `TurretAutotuneEngine`, `Flywheel`, `ShotGate`, `Encoder`, `VoltageCompensator`,
  the `Constants` they read), copied from TeamCode at build time by `:Sim:copyTeamCodeSources`. They
  compile against `Sim/src/ftcStubs` (only the SDK methods that code calls). To try a robot-code
  change, edit TeamCode, rebuild Sim and rerun. `--loop-ms` / `--loop-jitter` set the robot loop time.
  Traces (CSV) go to `Sim/out/`.
- To bring a new TeamCode class into the sims, add it to `copyTeamCodeSources` in
  `Sim/build.gradle.kts`. It must not touch hardware beyond the stand-ins. Read time through
  `Utility/Clock`, not `System.nanoTime()`, so the sim can drive it.

- Results and HTML reports go to `Sim/out/` (ignored by git). `profile report --html` charts
  every curve in the profile.
- Trying things without real data: use a scratch profile,
  `--args="profile init --example --db out/example_profile.db"`, then pass the same `--db` to `sim`.
  Never run `--example` against `data/robot_profile.db`.
- After changing `Catalog.java`: `profile init` (adds new rows, keeps values) and
  `profile docs` (regenerates `Sim/PROFILE.md`). Commit both the code and the updated DB.
- `sim all` takes about 1-2 min; `sim esd` is most of it (`--trials` sets Monte Carlo trials per
  point, default 1000, and scales how many matches `sim esd` plays).
- Missing inputs stop a sim with a list of keys and how to set them. That is expected
  until the profile is filled in. Add `--assume` to run anyway on placeholders (see above).
- When you set a profile value, say where it came from: `profile set <key> <value> --source manual
  --note "CAD 10-31: ..."`. The note is stored with the value.

## Verify

- Compilation is **not** proof the robot behaves — an agent cannot confirm runtime
  behavior. State clearly what was built vs. what still needs a human to run.
- After dependency or SDK changes, do a clean build to catch resolution/merge
  issues: `./gradlew clean :TeamCode:assembleDebug`.
- Sim changes: `./gradlew :Sim:test`, then rerun the affected sim on a scratch profile and
  look at its `Sim/out/<sim>/report.html`. Sim numbers are only as good as the inputs: say
  which inputs were placeholders.
- Lint/merge problems surface in `build/reports/` — check the problems report if a
  configuration phase fails.

## Safety rules for agents

- Never deploy or run an OpMode on the physical robot autonomously — a human must
  be present and initiate it.
- Treat changes to drive constants, motor directions, and servo ranges as
  high-risk; confirm with the user before editing.
- Do not commit secrets. Signing uses the shared `libs/ftc.debug.keystore` for
  debug; release signing reads `APK_SIGNING_*` environment variables.

## Housekeeping

- `./gradlew clean` clears module `build/` output.
- Don't commit `.gradle/`, `build/`, or `local.properties` (already covered by
  `.gitignore`), or `Sim/out/` (covered by `Sim/.gitignore`).
