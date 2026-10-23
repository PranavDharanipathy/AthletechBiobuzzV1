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

# Build the simulation app
./gradlew :Sim:assembleDebug

# Everything
./gradlew assembleDebug
```

Use `gradlew.bat` on Windows (this project is developed on Windows).

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

## Verify

- Compilation is **not** proof the robot behaves — an agent cannot confirm runtime
  behavior. State clearly what was built vs. what still needs a human to run.
- After dependency or SDK changes, do a clean build to catch resolution/merge
  issues: `./gradlew clean :TeamCode:assembleDebug`.
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
  `.gitignore`).
