# CLAUDE.md

This file guides Claude Code when working in this repository.

**The canonical project guide is [AGENTS.md](./AGENTS.md) — read it first.**

@AGENTS.md

Everything about modules, source layout, build configuration, dependencies and
conventions lives in AGENTS.md and the files it references
([AGENTOPS.md](./AGENTOPS.md) for operations, [DEV_NOTES.md](./DEV_NOTES.md) for
decisions and caveats). Do not duplicate that content here; update AGENTS.md
instead so every agent stays in sync.

## Quick reminders for Claude

- This is an **FTC robot project** (FTC SDK 12.0.0). Edit robot code in `TeamCode`
  only; treat `FtcRobotController` as read-only vendor code.
- Gradle builds target a real Android device / Control Hub. Do **not** assume a
  build or deploy succeeded without the user running it — flag anything you can't
  verify. See AGENTOPS.md.
- `compileSdk`/`targetSdk` are both 36; keep them equal. Kotlin and Java are both
  pinned to JVM target 1.8 — don't raise one without the other.
- Gradle is **9.1.0**: write `propName = value`, never `propName value`. The Kotlin
  plugin version (2.4.10) is pinned by the Gradle/AGP matrix *and* by Pedro's
  Sloth dependency (stdlib 2.4.10) — check DEV_NOTES.md before changing Gradle,
  AGP, Kotlin or Pedro versions.
- When unsure about a hardware-affecting change, ask before editing. A wrong
  constant or OpMode can damage the robot.
