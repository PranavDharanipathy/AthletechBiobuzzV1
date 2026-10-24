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
./gradlew :Sim:test                                     # unit tests
```

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
  until the profile is filled in.

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
