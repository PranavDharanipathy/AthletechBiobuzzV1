# DEV_NOTES.md

Project decisions, gotchas and caveats. Referenced by [AGENTS.md](./AGENTS.md).
Newest notes at the top.

## 2026-11-07 — Autos rewritten command based (Ivy on NextFTC)

- **Why:** the user wanted command-based autos. The split version rebuilt drive, shoot, fault and telemetry plumbing inside
  every auto.
- **Mechanisms own their commands:**
  - `Drive.follow(path)` = Ivy's `PedroCommands.follow`, with `setDone` adding the end tolerance and a timeout that sets
    `drive.fault`. Plus `hold`, and the static `pose`/`line`/`curve`/`slow` path helpers.
  - `Intake.run/stop`, `FIArm.deploy/retract`, `Shooter.spinUp/spinDown`, `Blocker.block/clear`.
- **`AthletechRobot` has the commands that use several mechanisms:**
  - `shoot(pose)`: hold, ready, feed window; it includes `shotWillScore`.
  - `collect()`, `stow()`.
  - `autonomous(steps...)`: a `race` of the sequence against a watchdog (time limit, loop timing, non-finite pose/velocity,
    path timeout), then `stopAll`.
  - Settings come in through `configureAuto`, `configureShots` and `drive.configure`.
- **Each auto** is only: poses from its own constants (static import), `requireCell` checks, its paths, and one
  `robot.autonomous(...)` block. "Nothing shared" now means between autos; commands belong to the robot.
- **Behaviour** is the same as before. Builds; not run on the robot.

## 2026-11-07 — Shoot on the move with the flywheel in it (ShotSolver) + per-ball miss gate

- **The physics:** the robot's velocity only adds to the ball's horizontal velocity. How high it climbs, and so its flight
  time T, comes from the flywheel alone (fixed launch angle, no vertical robot motion). With speeds tuned standing still, a
  ball at the flywheel speed for still distance D flies T(D) and covers D relative to the turret, plus v·T.
- **So a moving shot needs** (`Systems/Flywheel/ShotSolver`):
  - **Flywheel:** the speed of the still distance **D\* = |g − v·T(D\*)|**. The old code effectively used |g| (all flight
    times were 0). At 75 in and 40 in/s: 1265 ticks/s driving at the goal, 1469 still, 1550 across, 1859 away. 1859 is past
    the fit, so that shot is gated.
  - **Turret:** aimed along g − v·T.
- **Flywheel lag**, handled two ways:
  1. **Set the target early:** the target is for where the turret will be in `FLYWHEEL_SOTM_LEAD_SECONDS` (0.15 s,
     estimated).
  2. **Aim and gate on the actual speed:** once the flywheel is within `SOTM_ACTUAL_SPEED_BAND` (150 ticks/s) of target, T
     and the aim come from the speed it's really at. `missInches` = reach(actual) − |g − v·T(actual)| is how far long/short a
     ball fired now lands along the aim.
- **The gate:** `Shooter.shotWillScore()` keeps the blocker shut unless |miss| ≤ `SHOT_MISS_TOLERANCE_INCHES` (4 in ≈ 29
  ticks/s). It's in the TeleOp `Blocker` and in the four shooting autos.
  - **What it catches:** a flywheel still catching up after a direction change, and each ball's dip in a burst (the next
    ball waits).
  - **What it blocks on purpose:** shots past the fit's range (NaN miss).
  - **Stale values:** the value is time-stamped, so an OpMode without a running `Shooter` (a tuner) is never gated by an
    old one.
- **Flight time T(D):**
  - `SHOT_TIMES_OF_FLIGHT` when it's filled (measured).
  - Otherwise a projectile at `SHOT_LAUNCH_ANGLE_DEGREES` = 72 climbing `SHOT_RISE_INCHES` = 48.05 (CELL opening center
    1.5126 m − exit 0.292 m, robot profile).
  - **Where 72° comes from:** it's fitted, not measured. Assuming exit speed ∝ flywheel speed, it fits the 8 tuned points
    at 13.6 ticks/s RMS. But 68–76° fit almost as well, so T is ±15% uncertain.
  - **Tuning it:** `SOTM_TIME_SCALE` on the field. Moving shots landing toward where the robot was heading = raise it.
- **Solving for D\*:** bracketed then bisected (`SOTM_ITERATIONS` removed). Plain fixed-point iteration oscillated when
  driving fast at a close goal (56 in, 60 in/s); the Sim test caught it.
- **Code changes:**
  - `FlywheelVelocityModel.distanceFor` inverts the speed fit.
  - Old `getVirtualGoalCoordinate`/`getTimeOfFlight`/`lookUp` are gone from `Shooter`.
  - New Shooter fields: `shotDistance`, `shotTime`, `shotMissInches` (the Temporal Log picks them up).
- **Tested:** `ShotSolverTest` against simulated ball flight (launcher matching the tuned table when still, robot velocity
  added, no drag):
  - every direction up to 60 in/s at 56–96 in lands < 0.05 in from the goal;
  - the flywheel goes down toward the goal and up away/across;
  - `missInches` matches the real long/short landing within 0.05 in for flywheels 60 ticks/s off, with no sideways miss.

  The full Sim suite passes. This proves the math against its own model, not the robot: no drag, and the angle is a fit.
  Not run on the robot.
- **Switches:** `SOTM_ENABLED` (off = still-robot behaviour, flywheel from |g|) and `SHOT_MISS_GATE`.

## 2026-11-07 — 2 Tip + Park autos removed

- The user asked to remove them. `RedTwoTipPark`, `BlueTwoTipPark`, their constants and `docs/autos/*2tip*.pp` are deleted;
  four autos are left (Red/Blue Park, Preload + Park).
- `AthletechRobot.collect()`/`stow()`, `Drive.curve`/`slow` and `FIArm.deploy`/`retract` stay: robot commands, unused by autos for now.

## 2026-11-07 — Autos split: each auto self-contained

- **Why:** the user asked for each autonomous to be its own auto, with nothing shared between them.
- **What changed:**
  - `BiobuzzAuto` (the shared base class), `AutoPathConstants`, `RedAutoConstants` and `BlueAutoConstants` are deleted.
  - Each of the six (`Red`/`Blue` × `Park`/`PreloadPark`/`TwoTipPark`; the 2 Tip pair was removed later) is one standalone `BlazeOpMode` holding only what it
    uses: its own routine, drive/shoot/collect commands, fault checks and telemetry.
  - Each reads only its own `Constants/Autos/<Auto>Constants` (poses plus its own copy of the timing/tolerance values,
    starting from the old shared defaults).
- **Shared on purpose:** only the robot (`AthletechRobot`, `Autonomous/AutoSubsystems`), because NextFTC wants exactly one
  `NextRobot`.
- **Consequence:** a fix to the shoot or drive logic now has to be made in each auto that has it. That's the trade the user
  chose.
- **Behaviour:** unchanged from the base-class version. Builds; not run on the robot.

## 2026-11-07 — Flywheel speed from distance again, as a fitted model (not a lookup)

- **What changed:** `Shooter` sets the flywheel target from the distance to the (virtual, SOTM) goal again. It uses
  `Systems/Flywheel/FlywheelVelocityModel`, a least-squares polynomial through `FLYWHEEL_DISTANCES`/`FLYWHEEL_VELOCITIES`,
  in place of `5317f7f`'s one speed everywhere and the older piecewise-linear table lookup.
  - **Smoothing:** it's smooth between points and evens out noisy ones.
  - **Refitting:** it refits by itself when the points or degree change on the Dashboard, and allocates nothing otherwise.
  - **Range:** it follows the fit `FLYWHEEL_FIT_EXTRAPOLATE_INCHES` (12) past the tuned distances, then holds.
  - **Fallback:** with no goal, no fit, or `FLYWHEEL_DISTANCE_FIT` off, it uses `FLYWHEEL_DEFAULT_SHOOT_VELOCITY`.
- **Degree 1 by default.** On the 2026-11 table, a line misses by 12.6 ticks/s RMS. With each point left out in turn, it
  predicts that point at 16 RMS, against 18 for a quadratic and 30 for a cubic (the cubic overfits and runs away past the
  ends). A line through those points is speed = 7.256 d + 925.2.
- **Time of flight:** SOTM time of flight still uses the table (`lookUp`); all its values are 0 for now.
- **Hypertuner:** `FlywheelHyperTuner` now restarts its steady window only when the target moves by more than 10 ticks/s.
  The target follows the distance now, so every tiny change would otherwise restart it.
- **FlywheelInterpolationTuner:** its dataset view (Y) shows the fit through the collected points next to the one in use.
- **Tested:** `FlywheelVelocityModelTest` (matches numpy, clamping, refits only on change, degrades safely) and the whole Sim
  suite pass. Not run on the robot.

## 2026-11-07 — Blaze everywhere (but testers) + Temporal Log Red

The user asked for Blaze on every robot OpMode except testers, with NextFTC's Blaze support, plus a TeleOp that logs
everything against time. This supersedes "opt-in only" in the entry below.

- **`Utility/Blaze/BlazeSession`:** one lifecycle, used by `TeleOpBaseOpMode` (all TeleOps and most tuners) and by
  FlywheelTuner, FlywheelVoltageFilterTuner, GeneralVeloMotorTuner and TurretManualTuner.
  1. `begin` runs `initializeBlazeFTC` and `engageMotorAccel`. It must come before any `hardwareMap.get`.
  2. `engage` turns on background bulk reads on each hub that's there (RS485 Expansion Hub: 1 packet) and the Pinpoint feed.
  3. `tick` calls `BlazeFTC.run(0)` on the first loop. From then on it clears only the hubs Blaze doesn't feed: Blaze
     replaces `LynxModule.lastBulkData` from its own thread, so clearing a fed hub would just force a blocking SDK read.
  4. `end` closes Blaze.
  - Every Blaze failure is caught and shown on the `Blaze` telemetry line, and the OpMode carries on without that part.
- **Clean-up after each OpMode:** `MonitorOpModeHook.onOpModePostStop` calls `end()` after every OpMode, even one that
  crashed. That matters for three reasons:
  - **Robot restart:** Blaze needs closing, or the author says the robot restarts.
  - **Motors:** `engageMotorAccel` swaps the motors in the robot's hardwareMap, which outlives the OpMode. `end()` puts the
    SDK motors back, so a stock tester afterwards doesn't send power into a closed Blaze.
  - **Bulk mode:** Blaze forced every hub to MANUAL. `end()` sets OFF again.
- **Autos:**
  - `BiobuzzAuto` and `AutoPath` extend NextFTC's `BlazeOpMode`, so NextFTC adds its BlazeHook (init, `run(0)` at start,
    close in a `finally`).
  - `BlazeHubHook` wraps NextFTC's `BlazeBulkReadHook` per hub, because Blaze throws on a missing hub.
  - The constructor calls `BlazeSession.beginNext` first, since NextFTC's hook only runs after the constructor has built
    the hardware.
  - `AthletechRobot` attaches the Pinpoint feed and skips clearing fed hubs.
  - `BlazeConstants.ENABLED` can't turn Blaze off for the autos (the class hierarchy decides). With it off, NextFTC still
    starts Blaze, but only as an SDK passthrough.
- **`BlazePinpointLocalizer` instead of Blaze's `Pedro3SingleDataLocalizer`:**
  - Blaze's version throws on `setPose` (our TeleOp relocalizes).
  - Blaze's version writes the drive from Blaze's thread.
  - Blaze's version feeds raw Pinpoint registers, which are mm, to Pedro as if they were inches.
  - Ours:
    - takes Blaze's sample under a seqlock;
    - runs `update()` on the loop thread;
    - picks mm or inches by checking the first sample against an SDK read at INIT, which needs the robot away from (0, 0);
      otherwise it uses `PINPOINT_RAW_MM`;
    - falls back to Pedro's SDK `PinpointLocalizer` when the sample is older than `PINPOINT_STALE_SECONDS` (always during
      INIT);
    - does `setPose` as a software offset, so nothing writes the Pinpoint while Blaze polls it.
  - The `blazeftc_pedro3` dependency was dropped.
- **Not on Blaze:**
  - `Testing/` OpModes;
  - the utility OpModes (HardwareMonitor Viewer and Clear, Limelight Lab);
  - Pedro's tuning procedures (`Tuners/pedro`).

  Once any Blaze OpMode has run, though, the hub streams stay proxied through Blaze until the app restarts. That's Blaze's
  design.
- **Unproven, check on the robot:**
  1. After a Blaze OpMode stops, does a stock tester (SpinMotor) still move a motor? This tests `end()`'s motor restore, and
     whether Blaze's proxy still passes SDK packets once closed.
  2. Do two Blaze OpModes in a row both start? A second `initializeBlazeFTC` only clears telemetry, and Blaze's
     `opened` flags never reset.
  3. Blaze's ExHub `find {}!!` with the real hub layout.
  4. Is the Pinpoint unit guess right? Compare the `blaze.pinpoint.*_raw` and pose values in a temporal log.
  5. Motor `setPower` during INIT, before `run(0)`, goes to Blaze's native code. Whatever it does with it is unknown.
- **Temporal log:**
  - **What's in it:**
    - `TeleOp/TemporalLogRedTeleOp` extends `RedTeleOp`, so its behaviour is identical, and wraps init, loop and stop with
      `Utility/TemporalLog`.
    - **Hardware:** every `Monitored*` read and write calls `tracker.v(value)` before `tracker.read/write`. `DeviceTracker`
      logs value, duration, and whether it was a cached read or a redundant write. Cache hits log the last real value.
    - **HardwareMonitor's own samples:** its hub reads (current, voltages, temperature) and its motor current/velocity
      samples are logged too.
  - **Variables:** `VariableWatcher` walks the TeleOp's fields once by reflection, through our classes, Pedro and NextFTC,
    and not the SDK, Java or the monitoring/logging internals.
    - **What it walks:** it follows each field chain again every sample, so a new object in a field (a Pose made each loop)
      is followed. A field that was null grows its subtree the first time it holds something.
    - **What it logs:** primitives, enums (with labels), boxed numbers and primitive arrays of up to 64 entries, plus the
      static fields of the Constants classes. Each one is logged when it changes, at the end of each loop.
  - **Decisions:** every boolean or enum change, plus phases. That covers:
    - shooter on/off, auto-aim;
    - intake IDLE/INTAKE/REVERSE, blocker, FIArm;
    - the turret route's flags;
    - ShotGate.
  - **Numbers:** drive powers, flywheel target/speed, turret target/error/position, etc. come in as variables and as the
    motor/servo writes.
  - **Format:** 24-byte records (int64 ns, int32 key, int32 aux, float64 value) in `events.bin`, keys in `keys.tsv`.
  - **Threading and memory:** the loop thread fills reused 64 KB buffers (48 of them), and one MIN_PRIORITY daemon thread
    writes them. If the pool runs dry, records are dropped and the count is logged as `temporal.dropped_records`. The
    watcher logs its own cost per loop as `temporal.watch_us`: that's the observer effect, so check it before trusting loop
    times.
  - **Blaze's thread:** its reads reach the log through per-source `BlazeRing`s (single producer, fixed arrays), drained at
    the top of each loop with their own timestamps.
  - **Tested:** a desktop smoke test of TemporalLog + VariableWatcher (late objects, boxed fields, static changes, enums,
    hidden field names, Pedro Pose) decodes correctly with `tools/temporal_log.py`. Not run on the robot. Expect a few
    hundred KB/s; check free space.
- **Build:**
  - `:TeamCode:assembleDebug` passes. `AutoPath` (the old movement-only route) and `docs/AUTO_PATH.md` were then deleted
    at the user's request, after they had already deleted its `AutoPathRoutes`.
  - Nothing has been run on the robot.

## 2026-11-07 — BlazeFTC packaged, opt-in only

- **What it is:** BlazeFTC (github.com/owenpmckenna/blaze_ftc, `dev.anygeneric`) is a Rust rewrite of the hub link (Lynx
  protocol) that runs inside the SDK through JNI.
  - The SDK normally waits about 2 ms for each reply before sending the next request. Blaze keeps several reads and writes
    in flight instead.
  - It claims bulk reads at 500–1000 Hz, Pinpoint at about 200 Hz and Pedro at 200+ Hz, against about 50 Hz on the stock
    SDK. Those are the author's numbers, from "lab conditions".
  - Our loop is 29 ms median and 49 ms p99 (`tuning-data/REPORT.md`), so there's room.
- **Added:**
  - `maven.anygeneric.dev`, `blazeftc:0.1.66` and `blazeftc_pedro3:0.1.66` in `build.dependencies.gradle`.
  - It pulls only `kotlin-stdlib` 2.0.0 (resolves to our 2.4.10) and appcompat 1.2.0. No SDK and no Pedro come with it.
  - It's built for minSdk 24 and Java 8 bytecode.
  - The AAR ships `libblaze_ftc_neutrino.so` for armeabi-v7a and arm64-v8a, our two `abiFilters`.
  - Checked: `:TeamCode:assembleDebug` passes, the APK has both `.so` files, and `BlazeDummyPlug`, `BlazeFTC` and
    `Pedro3SingleDataLocalizer` are in the dex. This supersedes the 2026-11-03 "Don't extend `BlazeOpMode`" note: that
    `NoClassDefFoundError` was only because Blaze wasn't packaged.
- **API pairing:** NextFTC v2 0.3.0 calls Blaze through `BlazeHook` and `BlazeBulkReadHook`:
  - `BlazeDummyPlug.initializeBlazeFTC(HardwareMap)`
  - `engageBulkReadAcceleration(hw, Hub, packets, () -> Unit)`
  - `closeBlazeFTC()`
  - `BlazeFTC.run(int)`, `BlazeFTC.load()`

  All of these exist in 0.1.66. NextFTC only compiles against Blaze and doesn't declare it, so a mismatch shows up only
  as `NoSuchMethodError` on the robot. Bump the two together and recheck with `javap`.
- **What NextFTC wires up:**
  - Extending `BlazeOpMode` instead of `NextOpMode` turns on `RobotController.blazeEnabled`, which routes `NextMotor`
    writes through Blaze.
  - `BlazeBulkReadHook(hub, fastMode)` runs bulk reads for one hub (`fastMode` = 2 packets in flight) and calls
    `onControlHubBulkData()` / `onExpansionHubBulkData()`. Don't use it alongside `BulkReadHook`.
  - `onLocalizationData()` exists but nothing calls it. Pinpoint/Pedro speed-up means calling
    `Pedro3SingleDataLocalizer.setup(follower, runnable)` yourself.
- **Why it's opt-in and match code stays on `NextOpMode`:**
  - **Maturity:** beta, one maintainer, started Dec 2025, and the repo has **no license**. The OTOS, other I2C and the
    Servo Hub aren't parallel or are unstable.
  - **Threads:**
    - Blaze runs several native threads and calls Java back from them. The author calls thread safety "up in the air".
    - `Pedro3SingleDataLocalizer` runs `follower.update()` on Blaze's thread. Our `Drive` mechanism runs the follower on
      the NextFTC loop, and `Shooter` reads its pose and velocity there every loop, so turning it on means rethinking who
      owns the follower.
    - This also breaks the Control Hub rule: one low-priority I/O thread, no hardware off the loop thread.
  - **Failure:** if `closeBlazeFTC()` isn't called (a crash outside NextFTC's `finally`), the author says the robot may
    need a restart. It honors the SDK's stop flag only every 60 ms.
  - **Our wrappers:** `engageMotorAccel` swaps the motors in the hardwareMap, so `Monitored*` wrappers and HardwareMonitor
    I/O timings no longer see the real I/O.
  - **Sim:** it can't run Blaze callbacks, so code-in-the-loop tests won't cover a Blaze OpMode.
  - **Our turret is on servos**, which Blaze doesn't speed up.
  - **Legality:** the author says "Competition Legal". It intercepts the hub link at runtime, so confirm on the official
    FTC Q&A before an event.
- **How to try it:**
  1. Measure first. HardwareMonitor (`:8080/hwmonitor`) shows per-device I/O time per loop. Blaze only pays if the loop
     is mostly bulk reads, the Pinpoint read or the RS485 Expansion Hub.
  2. Then write one `BlazeOpMode` in `Testing/` with `BlazeBulkReadHook(CONTROL_HUB, false)`. Compare loop time and how
     Pedro holds a path against the same OpMode on `NextOpMode`.
  3. Watch for Sloth hot reload clashing with the JNI library (a library loads into one classloader only), and for the
     robot needing a restart after a stop.
- **Not run on the robot yet.**

## 2026-11-07 — Auto: user's mirrored export, movement only; strafe flip removed

- The "X flipped" run was `16d5d0d`, whose route an earlier agent had shifted west (path 1 to (20, 17): left 11.4, up 8.8).
  The auto-only strafe flip that `33368b6` added was a misdiagnosis: TeleOp strafes correctly with the same wheel commands,
  and with the drawn points the flip makes Pedro run away sideways (Pedro sim). Removed `AutoMecanum` and `REVERSE_STRAFE`.
  `DriveMechanism`/`AthletechRobot` are back to main's `7c2fc70` (plain `createFollower`).
- `AutoPathRoutes` = the user's visualizer export exactly, through `PoseFactory.degrees().mirrorX(70.75)`
  (x → 141.5 − x, heading → 180 − heading), so the robot drives the drawing's mirror image (path 1 goes left).
  `AutoPath` is movement only: shooter/intake never started. Details: docs/AUTO_PATH.md.
- Lesson: before blaming hardware, check which commit actually ran and whether someone changed the user's points.
- Builds; Pedro sim finishes all 7 paths in about 10 s. Not run on the robot. `tools/AutoPathGeometryCheck.java` still uses
  the old route field names.

## 2026-11-06 — Evidence review of autonomous on d8649e7

- Reviewed autonomous, mechanism/shooter integration, installed NextFTC/Pedro implementation and read-only profile provenance. Findings and source references: docs/AUTO_REVIEW.md.
- Confirmed missing exception cleanup and non-immediate Follower.stop motor writes; actual Pedro stop probe reproduced this. Seven AutoShotWindow tests pass.
- Exact shot ranges 49.660/39.015/36.948 in all clamp below table minimum to 1300 ticks/s. Shot sides south/north/north, no cell-tip sensing. Partner push-through remains absent.
- Geometry checker inferred rear from start Y, despite a CAD back-plate extent in robot_profile.db; CAD-to-Pedro origin match itself remains unverified. Prior validation is not a physical guarantee.
- Corrected stale autonomous docs and shutdown claims. No control-code changes, commit, push or deployment during review; manual strafe override remains enabled as requested, with physical cause unresolved.

## 2026-11-06 — Merge autonomous correction into current NextFTC main

- Preserved main's NextFTC OpMode/mechanisms and latest shooter tuning. Auto passes the snapshotted reversal flag through AthletechRobot.init into DriveMechanism; other callers default to normal strafe.
- Restored exact coordinates and retained AutoMecanum output correction. Supersedes prior notes describing the old LinearOpMode implementation.
- Android assembly passes. Sim suite: 70 pass, 1 skipped, 1 fails (`holdDoesNotHuntWhenFrictionIsGuessedHigh`, hold RMS 1.9925214752183356). Confirmed identical failure on pristine main 7c2fc70 in a separate worktree; no turret tuning modified by this merge.

## 2026-11-06 — Manual auto strafe reversal

- User authorized fallback reversal after code review found no established cause for physical horizontal inversion.
- Auto-only AutoMecanum reverses the body strafe term before normalization, including maxScaling wheel-mix calls. Forward/yaw and supplied coordinates unchanged.
- AutoPathConstants.REVERSE_STRAFE defaults true, captured at INIT and displayed in telemetry. False plus re-INIT restores normal Pedro mixing. Global drive/localizer configuration and TeleOp untouched.
- Build and mixed-command algebra checks pass; does not establish actual motor response or fix wrong odometry signs. Physical diagnosis remains required.

## 2026-11-06 — Restored exact user route on auton

- Supersedes the clearance/range revision below: restored all 12 supplied coordinates without mirroring or scaling. Original headings and shooting sequence retained.
- Physical horizontal reversal is unresolved; verify manually observed pose-axis signs and heading before changing hardware directions. See docs/AUTO_PATH.md.
- Prior geometry PASS does not apply: restored wall approach violates deployed-intake clearance and shots need range review. Partner push-through at point 6 remains pending.

## 2026-11-06 — AutoPath clearance/range coordinate revision

## 2026-11-06 — Turret hunting and buzz on holds (`TurretController`)

- **Seen on the robot** (every DRIVING run, flywheel on the whole time): at DONE it swung ±4° at ~2 Hz forever, and holds
  buzzed with the ±1 V friction push flipping almost every loop. That "hold shake" was the controller, not the flywheel:
  with the turret unpushed the encoder barely moves (median 0–6 ticks a loop), so the sim's default 0.3° flywheel shake is
  far too strong for this robot.
- **Hunting:** it slides on ~0.75 V at speed (run `20280507-195213-driving`, 50°/s on 1.04 V) but STATIONARY measured
  1.01 V near rest. Pushing full friction all the way in overshoots, then it pushes back the same way. Now, holding, if it
  comes in faster than 3× what the loop asks for (+3°/s) it lets go once it gets there within a lead, and brakes if it'll
  run well past. With the friction guess right it never triggers, so STATIONARY behaves as before.
- **Buzz:** the Pinpoint heading rate jitters 1.1°/s sd with the robot still, and the plan's target-acceleration lead
  blew that up to ~5°/s of planned speed against a 1°/s "moving" line. Now the target rate gets a soft deadband of
  4× its measured noise (second difference, at least 2°/s, fading out by twice that), and the plan has to pass 4°/s to
  count as moving. The noise is only measured while the rate stays under 10°/s: real turning jerks around (and the sim's
  spin target clamps at ±50°), which counted as noise, grew the deadband and made spins lag.
- **Sim, the robot's turret with plant friction 0.75 V vs a 1.01 V guess:** DONE-style hold 2.6 → 0.05° (31 ms),
  3.4 → 1.9° (60 ms TeleOp). Still robot with 1.1°/s rate noise: 0.7–5° and 0.7–1.5 V of chatter → 0.00° and
  ≤0.05 V. Exact friction: steps unchanged. Tests `holdDoesNotHuntWhenFrictionIsGuessedHigh`,
  `noisyRateDoesNotBuzzAStillTurret`.
- **Costs (sim, exact friction):**
  - Spin tracking at bandwidth 3.2: 3.44 vs 3.35° (31 ms), 3.97 vs 3.94° (50 ms).
  - STATIONARY (16 seeds) never picks below 1.5 rad/s, same as before. At what it picks, spins are about 0.3° worse
    (3.33 vs 3.04° at 24 ms).
  - Under the sim's 0.3° flywheel shake, holds are 1.0–1.4° vs 0.5–0.6°: velocity noise trips the let-go. The real
    encoder doesn't shake like that.
- **Robot run `20280507-210857-driving` with this code:**
  - The buzz was mostly gone: the push flipped 2–4 times a second instead of 17.
  - It still hunted ±4–5° at DONE. It slid on ~0.65 V and reached 60°/s within 100 ms, so letting go a lead early came
    too late.
  - So the hold push strength is now learned (`holdScale`): each overshoot under its own push takes 10% off (down to
    0.4), and it creeps back to 1 over 20 s. Kicks still use the full breakaway.
  - An earlier version also raised it on every kick. Stuck turrets kick a lot, which put it straight back to 1.
  - STATIONARY doesn't learn it (`minHoldScale = 1`), the same way it turns rejection off.
  - Sim with plant friction 0.65 / 0.75 V: holds 0.0–0.32° (were 2.2–2.8°).
- **Not verified on a robot.**

## 2026-11-06 — Vibration rejection floor comes from DRIVING (`TURRET_SHAKE_BANDWIDTH`)

- **Problem:** rejection could never soften. Its floor `TURRET_MIN_BANDWIDTH` was a hand-set 7 rad/s from the PIDF
  days, and the tuned bandwidth is 3.2.
- **Lowering the floor makes it worse** (sim, real turret model from run `20280507-081142-driving`, mean of 6 seeds):

  | Loop | Bandwidth 3.2, rejection off | 2.4 | 1.6 | Rejection on, floor 1.6 |
  |------|------------------------------|-----|-----|-------------------------|
  | 20 ms | 0.63° | 4.6° (worst seed 8.8°) | 4.8° | 2.0° |
  | 35 ms | 0.63° | 0.58° | 3.4° | 1.9° |
  | 50 ms | 0.45° | 0.45° | 0.54° | 0.59° |

  The table is hold error with 0.3° of flywheel shake. Spinning tracking also gets 1.3–2× worse when softer. A soft
  loop lets friction creep the turret off target.
  - It isn't the rejection's moving gains: rejection 8 → 6.8 behaves exactly like a fixed 6.8.
  - Softer is just a trade: at 35/50 ms it holds 10–38% better but tracks a moving target 6–19% worse, and at
    20 ms it holds worse too.
- **Now DRIVING finds the floor** instead of lowering the bandwidth (`Parameters.shakeBandwidth`, result
  `shake_bandwidth`, `TunedConstants.TURRET_SHAKE_BANDWIDTH` → `TurretConstants.TURRET_MIN_BANDWIDTH`):
  - 2 shaky judged windows start a trial at 0.85 × the floor so far. After 2 judged windows it is accepted only if
    all three hold:
    - shake is < 0.8 × before (or the flips stopped);
    - the hold's aim error is lower;
    - tracking lag (error per speed, so how fast the driver turned cancels out) is ≤ 1.05 × before.
  - Accepted → that's the floor, and the next trial starts from it. Rejected, or it can't settle when softer → it
    stops trying.
  - The bandwidth always goes back to what STATIONARY found. The floor is never below max(2, 0.5 × bandwidth).
  - Holds count from the first time the turret catches up (within 2°) after the target has been still 0.5 s. A
    loop that wanders off after that still counts, so it can't look calm by drifting out of the 2° band.
  - The analyzer takes the floor only from a DRIVING run (capped at its bandwidth). A STATIONARY run sets it equal
    to the bandwidth, so there's no softening until DRIVING has shown it helps.
- **Tested** (sim, DRIVING with 0.3/0.6° flywheel shake, 3 seeds each):
  - From 3.2, 5 and 8 at 20/35/50 ms (54 runs): all finish, and the floor stays at the bandwidth every time.
  - From 14 at 35 ms (too stiff): it steps down to 10.1–11.9. With that floor, rejection holds 0.55 vs 0.62° and
    1.00 vs 1.14°, and spinning is 2.00 vs 3.04° and 2.41 vs 3.63°, so it finds real wins.
  - From 11: 3 of 12 runs accept a roughly neutral step (tracking within 5–10%).
  - The sim's DRIVING now shakes the turret like the flywheel. Its hold shake (27–40°/s) matches the robot's
    15–38°/s.
- **For this turret:** `TURRET_SHAKE_BANDWIDTH` = 3.2 = the bandwidth, so rejection stays out of the way (same
  behavior as before, now on purpose). Test: `drivingFindsTheRejectionFloor`.
- **Not verified on a robot.**

## 2026-11-06 — DRIVING shake check fixed; relocalize telemetry

- **What the robot showed** (runs `20280507-080929/081142-driving`): every window came out "calm", even with hold
  readings of 40–170°/s.
  - The bar was 1.6 × the first window, and that window was already bad.
  - "Hold" was any moment the target speed was under 3°/s. With nonstop driving, those moments are mostly the
    turret catching up after a whip, not shake.
  - The real holds (target still ≥ 0.5 s, turret within 2°) had 15–38°/s RMS while driving, against
    1.3–1.6°/s with the robot still. That's about 0.3–0.6° of shake from driving and the flywheel.
  - The 25° tracking error was the driving, not the tuning. The robot hit 9 rad/s and was past the turret's
    ~1390°/s² plan acceleration 10% of the time. The turret followed its own plan within 2–4°. The robot's spin
    acceleration doesn't push the turret off its plan either: the correlation was 0.00.
- **Now** (`TurretAutotuneEngine.drivingTune`):
  - Shake and flips count only in real holds: the target still for ≥ 0.5 s, the turret within 2°, and not
    swinging. Flips are per second of holding.
  - A window is judged only with ≥ 0.5 s of holds. Otherwise it's "no hold, not judged", which neither counts
    toward nor resets the 3 calm windows. Telemetry asks the driver to stop turning for a second now and then.
  - "Shaky" means > 3 flips/s, or shake > max(6°/s, 1.6 × the calmest window so far). It is no longer measured
    against the first window.
  - 2 shaky windows in a row start a **trial** at 0.85 × bandwidth (never below 2 rad/s). After 2 judged windows:
    - It keeps the lower bandwidth if the shake dropped below 0.8 × before (or the flips stopped).
    - Otherwise it puts the old bandwidth back and stops trying: the shake isn't the loop's. After that, shaky
      windows count as settled, since nothing is left to change.
- **Sim:** the autotune's driving now stops for 2 s every 6 s (`TurretScenario.drivingTime`). At 20 ms the trial
  kept 3.99 (hold 11–12 → 5–6°/s). At 50 ms the trial didn't help, so it put 3.34 back and finished.
- **Relocalize didn't seem to do anything.** The code path is right:
  - `Shooter.relocalization` → `follower.setPose`. In Pedro 3 that's `localizer.setPose` → Pinpoint
    `setPosition`.
  - `follower.update()` reads the pose back from the Pinpoint, and `follower.pose()` returns that.
  - Nothing else in the loop calls `getInformation()` a second time.
  - SDK 12 sends `guide` in the gamepad packet (`ps` is an alias), but only when the Driver Station's **Advanced
    Gamepad Features** is on (the default). With it off, Android can take the button as a home press.
- **Telemetry** (`TeleOpBaseOpMode.addDriverTelemetry`, Red/Blue TeleOp) shows the pose plus `Relocalize (gamepad 1
  PS/guide)`: button DOWN/up and how many times it relocalized.
  - If the count doesn't go up on a press, the button isn't reaching the robot.
  - The lines are made once (`setAutoClear(false)` + `Func`s) and filled in only when telemetry is sent.
- **Not verified on a robot.**

## 2026-11-06 — Turret: spin as much as you like (DRIVING autotune and match aim)

- **Why:** DRIVING used to clamp its test target at ±50° and asked the driver to stay within 50° of the start.
  The team spins freely, so that limit is gone.
- **`Systems/Turret/TurretRoute`** (pure Java; the Sim runs it). It picks which turn of the aim the turret goes to,
  and both `Shooter` (match aim) and the autotuner's DRIVING target use it.
  - It follows the aim continuously, so it never jumps a turn on its own.
  - Heading for an end, it swings a full turn around once the room left is less than the turret's stopping
    distance + `TURRET_SWING_MARGIN_DEGREES` (5). Stopping distance = speed·(dead time + 2 loops) +
    speed²/(2·0.8·coasting deceleration), the same stop the controller plans.
  - After a swing, the aim can still be past the other end. The turret then waits at that end with no
    feedforward until the aim comes in.
  - If the aim sits past an end and isn't coming back (more than the margin out, coming in slower than
    1 s per degree beyond that), it swings back. Small overshoots therefore don't flip back and forth.
  - Shooter's old `routeTurret` (nearest-to-turret, swap only at ±190) is gone. It swapped too late: at 200°/s
    the turret needs ~30° to stop, so it ran ~30° past 190.
- **`TurretController` travel (`setTravel`, set by `TurretBase` and the autotuner to ±190 from home):** the plan
  never heads for an end faster than it can still stop before it.
  - Without it, an aim faster than the turret (6 rad/s spins), coming round from behind, kept the plan moving
    forward (aim speed minus top speed) and ran it to 216°/243°.
  - It defaults to no limit, and normal moves are unchanged: the autotune results are identical with and
    without it.
- **DRIVING target:** stays `TRAVEL_INSET_DEGREES` (10) inside the travel, so the fault check at ±190 always
  has spare room. While it swings, waits, or settles for 0.5 s after catching up, nothing counts toward the
  tuning statistics, and telemetry says "swinging around the end".
- **Sim** (`sim turret --assume`, 20/35/50 ms loops):
  - DRIVING while spinning 1.5 rad/s on top of the back-and-forth finishes, with max travel 171/170/175°.
  - The new "match aim" line spins 1.5/3/6 rad/s one way, then back. Max travel is 182/184/190° at 20 ms;
    at 6 rad/s the aim outruns the turret, so it waits at the end.
  - Tests: `drivingAutotuneHandlesEndlessSpinning`, `matchAimNeverRunsPastTheTravel` (6 rad/s stays short of the
    hard-limit cut at 195).
- **Not verified on a robot.**

## 2026-11-06 — Relocalization at the GARDENS

- **Controls:** gamepad 1's main (PS/guide) button relocalizes to your alliance's GARDEN corner (`Field/Garden`).
  It replaces the single `RELOCALIZATION_POSE`.
- **Gamepad 2 dpad nudges the aim point** by `GOAL_NUDGE_INCHES` (1 in) per press, as the driver sees it: up = away
  from your drive team, right = their right. `Goal.nudgeFromDriver` turns that into field x/y:

| Alliance | Drive team stands | up | right |
|----------|-------------------|----|-------|
| Red | behind the x = 0 wall, looking +x | +x | −y |
| Blue | behind the x = 144 wall, looking −x | −x | +y |

  - Only the aim point for the robot's half moves (close and far now have separate offsets).
  - Turret and flywheel both aim at `goal.getCoordinate`, so they follow it.
  - Offsets reset at `shooter.start`.
  - Turret home nudge moved from dpad left/right to gamepad 2 X (left) / B (right).
- **Goal coordinates from the manual** (9.6, Figures 9-9/9-10; HIVE structure at the field center):

| Goal | x | y |
|------|---|---|
| Red CLOSE | 59.25 | 87.77 |
| Red FAR | 59.25 | 56.23 |
| Blue CLOSE | 84.75 | 87.77 |
| Blue FAR | 84.75 | 56.23 |

  - x: the HIVE centers are 25.5 in apart (red on the left). y: the center of a tipped-up CELL's opening.
  - The opening is the CELL's outer end face (14 in tall), the HIVE tips 30°, and the arm sits 1.38 in below the
    43.95 in pivot. That gives the manual's 53.5 / 65.6 in opening heights, and puts the opening center 15.77 in
    from the pivot and 59.5 in up.
  - Profile `target.cell.opening_center_height_m` = 1.5126 (manual).
  - The manual's tolerance is ±1 in; nudge on the field with the gamepad 2 dpad.
- **`FAR_ZONE_CLOSE_ZONE_BARRIER`** was −35, from DECODE's frame. In this frame every pose has y > −35, so it always
  picked CLOSE. It is now 72, mid-field: y > 72 (away from the audience) aims at the CLOSE point, below at FAR.
- **Where the gardens are** (Competition Manual 9.3, Figures 9-2/9-5):
  - Each GARDEN is a ~23 x 2 in strip along a wall, in opposite corners.
  - Red runs along the audience wall from the audience-left corner (tile A1).
  - Blue runs along the far wall from the opposite corner (F6).
- **Pose:** the robot points at its GARDEN (front on that wall) and is pushed fully into the corner (side on the other
  wall).
  - Red: (`ROBOT_CENTER_TO_SIDE`, `ROBOT_CENTER_TO_FRONT`, −90°).
  - Blue: (144 − side, 144 − front, +90°).
- **Field frame** (`FieldConstants`; nothing else had fixed it yet: `START_POSE` was 0 and the goals were NaN):
  pedro inches, (0, 0) at the corner on the audience's left, x toward blue, y away from the audience, heading 0 = +x,
  counterclockwise. **Goal coordinates and `START_POSE` must use the same frame.**
- **Robot outline** from CAD: front (bash bar) 9.72 in, sides 7.64 in from the pose point. This assumes:
  - Pedro's pose point (the Pinpoint offsets' center) is the CAD origin.
  - The bash bar is the robot's front.
  - The intake is up (down, it reaches 14.1 in).
- **Not checked on the robot:** relocalize, then drive to a measured spot and compare the pose.

## 2026-11-06 — Turret Autotuner, third fix: the servos coast, they don't brake

**What the robot showed** (run `tuning-data/turret-autotune/20280507-063450-stationary`, pulled over USB; "every
bandwidth overshot or shook"):
- The model came out right: 127°/s/V, 38 ms delay moving, 78 ms from rest, friction 0.98 / 1.15 V, 1777°/s².
- The 30° steps still overshot 15-45%.
- The trace shows how the servos slow down:

| Command while moving | Deceleration |
|----------------------|--------------|
| Forward but under friction (~1 V) | only coasts at ~500-830°/s² |
| 0 V | ~1500°/s² |
| Reversed | hard, 1300 to 7000+°/s² |

- The plan expected 0.8 × 1777 and kept a small forward command, so it rolled past every stop.

**What changed:**
- **`TURRET_MAX_DECELERATION`** (ticks/s², coasting; 0 = same as acceleration): the plan slows down at most
  0.8 × that, and its stopping curve uses it. The autotuner measures it on the small steps: they now coast to a stop
  at 0.85 × breakaway forward, then 0 V, instead of −9 V. The rate is 0.45 × speed / time(85% → 40%), so the delay
  before slowing doesn't count. It is clamped to [0.2, 1] × acceleration so a bad read can't leave it unable to stop.
  Plumbed like the others: `.deceleration()`, `max_deceleration`, the analyzer, `turret.coast_decel_dps2`.
- **Reverse to brake:** once the turret has run more than 2 × `frictionErrorWidth` (1°) past the reference, the
  friction term reverses. That's the only way these servos brake.
- **No plan swing:** a plan that crosses a still aim stops there instead of swinging back. A long loop carried it
  0.5° past, which flipped the push direction and braked the turret 6° short.
- **Kicks:** the extra kick volts grow by 0.08 per failed kick (was 0.05). On the robot the 1.15 V kicks failed
  and 1.26 V worked.
- **Tuning loop:**
  - `unstable()` is overshoot over 30% or more than 8 swings only. Friction hops while holding count against the
    score (0.1 × hold), not as instability, so it always keeps its closest try instead of faulting.
  - The search starts at 0.3 / (moving delay + loop).
  - The sine-based gain correction was dropped from STATIONARY and DRIVING: the measured gain is right, and the start
    delay at every reversal biased it down 20-30%.
  - DRIVING never raises the bandwidth any more, it only backs off. Raising chased that same lag into hunting.
- **Sim:**
  - `CrServoTurret` has `coastDecel` (forward-but-lower command coasts) and `neutralDecel` (|u| ≤ 0.1 V) added to
    the reverse braking. New catalog entries `turret.coast_decel_dps2` / `turret.neutral_decel_dps2`.
  - The profile `turret.*` now holds this run's numbers.
- **Seeded `TunedConstants`:** gain 8691 (127°/s/V), lag 0.03, dead time 0.038, start delay 0.08, friction 0.98 /
  1.2, accel 1777, decel 800°/s², top 360°/s.

**Sim** (copy of this turret: STATIONARY then DRIVING; no bandwidth passes every check, so it keeps the closest,
with a warning):

| Loop | Bandwidth | Overshoot on 30° | Settle | Error while spinning |
|------|-----------|------------------|--------|----------------------|
| 20 ms | 3.85 | 0.35° | 0.84 s | 2.8° |
| 35 ms | 6.9 | 0.01° | 0.60 s | 2.6° |
| 50 ms | 4.1 | 2.5° | 1.46 s | 3.4° |
| Pure 0.1 s delay servo | 4.9 | 0.5° | — | 3.1° |

- The seeded values are about as good: 20 ms gives 0.9° overshoot and 2.2° while spinning.
- The fault diagnostics still work.

**KNOWN GAP: lanyard learning is broken for these servos.**
- The learned map stays ~1 V off and holds are ~3° out.
- The coasting turret copy is ~20° out. Its feedback is a few tenths of a volt against loads of several volts.
- `CodeInTheLoopTest.lanyardSweepLearnsOverlappingLanyards` is `@Ignore`d with that reason.
- Lanyards are off (`TURRET_LANYARDS_ENABLED = false`); fix this before turning them on.
- Learning from the speed shortfall (velocity error / gain) was tried and didn't help.

**Not verified on the robot.**

## 2026-11-06 — Turret Autotuner, second fix: two delays, kicks that adapt, a search that goes the right way

**Robot screen after the first fix:**
- "couldn't find a stable bandwidth" again.
- Every test: hold 11 (never settled), overshoot 12-45%, and **"late -107 ms" (the turret ran 0.1 s early)**.
- Bandwidth was lowered every time: 3.3 → 0.6.

**In run 051944:**
- From rest, motion starts 90-150 ms after a command.
- While moving, the brake bites within one loop: the speed drops in the next sample and reverses within ~50 ms.
- So there are **two delays**. The controller used the slow one for everything, so it aimed 0.1 s into the future
  while the turret answered right away: early, overshoot, stuck past the target.

**What changed:**
- **Two delays:**
  - `TURRET_DEAD_TIME` is now the delay while moving. The autotuner measures it from braking: the time from the
    brake command until the speed drops, plus half a loop.
  - `TURRET_START_DELAY` is new: the delay from rest, from the full-power ramp start.
  - Only the friction kicks and the friction-ramp correction use the start delay.
  - Plumbed like `TURRET_MAX_SPEED`: Parameters `.startDelay()`, result `start_delay`, the analyzer,
    `turret.servo_start_delay_s` (new catalog entry) and import-autotune.
- **Adaptive kicks:**
  - A kick that didn't move the turret comes back twice as long, capped at the start delay, and +0.05 V, up to
    +0.6 V.
  - One that did comes back one loop shorter, with the extra volts halved.
  - So it settles on what the servo needs, whether its start delay is a plain delay or needs a sustained push.
  - The sim turret can now model the sustained-push kind: `startDelay`, past breakaway for that long before it moves.
- **Tuning loop:**
  - Overshoot over 8% or more than 3 swings means too stiff, so go down.
  - Anything else that fails (never settled, hops on friction while holding) means too soft, so go **up**.
  - It bisects between the soft and stiff bounds. If nothing passes, it keeps the closest non-unstable try with a
    warning instead of faulting.
  - The gain correction from the sine also runs on tests that didn't settle, at half steps, skipping
    overshooting or swinging tests.
- **Seeded `TunedConstants`:** dead time 0.02, start delay 0.12.

**Sim results** (`sim turret`; profile `turret.*` = the run's numbers: 150°/s/V, 20 ms moving / 120 ms from rest,
1500°/s², friction 1.30 / 1.40):
- STATIONARY settles on w 12.4.
- 30° steps overshoot 0.6-1.0°.
- Error while spinning is 1.6° (2.8° with the seeded values).

| Variant | Result |
|---------|--------|
| Pure 0.1 s delay servo | w 7.2, no overshoot |
| 35 ms loop | nothing passes every check; keeps w 8.5 with the warning; DRIVING → 1.4° |
| 50 ms loop | passes |

- The faults sim still catches a reversed encoder, fighting servos, a dead servo and a jam.
- With this turret, vibration rejection is worse in the sim (holding 0.49° vs 0.31° off). The shake model is a
  guess, so check it on the robot.
- **Not verified on the robot.**

## 2026-11-06 — Turret Autotuner fixed: the servos run out of torque, not volts

**What the robot's runs showed** (`tuning-data/turret-autotune/20280507-051944-stationary`, then "couldn't find a stable
bandwidth"):

| Step | Speed reached |
|------|---------------|
| 2.9 V | ~240°/s, then levels off |
| 4.4 V | still rising at the end |
| 6.4 V | still rising at the end |
| 8.9 V | still rising at the end |

- At 4.4 V and above, all the steps speed up at the same ~1500°/s² (torque limited) and reach ~340-377°/s.
- Motion starts 90-150 ms after the command at every voltage.
- Braking is about 3× faster than speeding up.
- At rest with 0 V the turret holds still; it doesn't drift.

**Why the old autotuner failed:**
- It fit one straight line through all the steps, so the torque-limited ones dragged the gain down to 33-54°/s per V.
  The real value is ~150. That made `kp` and the feedforward ~3-4× too strong, so every bandwidth failed, and lower
  bandwidths failed worse because the feedforward dominated.
- The controller had no speed or acceleration limit, so its plan asked for ~11000°/s².
- The gain formula needs a negative `kd` below ~1/τ. That got clamped to 0 and left the loop limp (`kp` 0.005 V/deg).
- Breakaway friction was held until the late velocity showed motion. With 0.1 s of dead time that hopped ~2° past
  small targets, so it hunted.
- The target-acceleration filter (time constant = lead) put ~2° of error into the plan on its own.
- The first "left servo pushes the wrong way" fault was false: the single-servo check started while the turret was
  still coasting from centering. **{REVERSE, REVERSE} was not fighting**, and flipping one servo to fix it was wrong.
  Pranav's {FORWARD, FORWARD} is right; the old encoder warning was real for {R, R}.

**What changed:**
- **Autotuner stage order:** kick, then single-servo checks, then friction ramps, then small steps. These all run
  open loop with `still()` (volts off until the turret stops) between them. Nothing runs closed loop until the
  turret has been measured.
- **Small steps** (breakaway + 0.3/0.6/1.0/1.6 V, 60° each way, alternating direction):
  - Speed per volt and kinetic friction come from the steady speeds.
  - The lag comes from the two slowest steps only.
- **Full-power steps** (9 V, lined up at ∓35°, 70° travel) give:
  - acceleration, from the straight ramp;
  - top speed (a lower bound if it was still speeding up);
  - dead time, from where the ramp starts: 101 ms vs the true 100 in the sim, where the old fit gave 115.
- **Friction reading:** the friction ramp reads high by 2 V/s × dead time, so that is taken off.
- **`TurretController.setCapability(maxSpeed, maxAcceleration)`** (`TURRET_MAX_SPEED`, `TURRET_MAX_ACCELERATION`, in
  ticks; 0 = unknown, which acts as before):
  - The plan's top speed is 0.8 × min(volts allow, measured top speed).
  - Its speed change per loop is min(half volts, 0.8 × acceleration × loop).
- **Gains:** `kp` ≥ w/gain and `ki` ≥ a·w²/gain, so the loop really crosses over at w.
- **Breakaway kicks:** when stuck, it applies breakaway for one loop, then kinetic for a lead time to see if it moved.
- **Target acceleration** is filtered at 0.05 s instead of over the lead.
- **Tuning loop:**
  - Friction scale only moves after passing tests and is capped at 1. Above 1, sliding friction alone beats
    breakaway and it creeps.
  - The final checks use the best passing settings instead of whatever the last failing test left behind.
  - Each test logs how many ms early or late it was on the sine. I tried correcting the dead time from that: it
    dragged the dead time from 101 down to 61 ms and the steps overshot, so it only logs.
- **Pipeline:** results add `max_speed` and `max_acceleration`. The analyzer writes them into `TunedConstants`
  (0 for older runs), and `profile import-autotune` writes `turret.max_speed_dps` / `max_accel_dps2`.
- **Seeded `TunedConstants`:** they were seeded by hand from that run (gain 10240 ticks/s/V = 150°/s/V, lag 50 ms,
  dead time 100 ms, friction 1.30 / 1.35 V, bandwidth 5, top 360°/s, 1500°/s²). The old defaults were ~6× off and
  overshot 20° in the sim. The next successful autotune replaces them.
- **Sim:** `CrServoTurret` has an acceleration cap (braking 3×) and a top speed. The profile `turret.*` servo values
  are now manual, from that run.

**Sim results** (that turret, 20 ms loop):

| Settings | 30° step overshoot | 30° step settle | Error while spinning | Hold |
|----------|--------------------|-----------------|----------------------|------|
| Old defaults | 20° | never | 20° | — |
| Seeded | 0 | — | 3.7° | — |
| After STATIONARY autotune (w 7.7) | 0 | 0.71 s | 3.4° | 0.08° |

- STATIONARY finishes in 225 s; its estimates are ±2% (lag 52 vs 50 ms).
- With a 35 ms loop it also finishes, at w 2.1, and DRIVING takes it to 3.1.
- Vibration rejection now reads slightly worse with this turret model (holding 0.56° vs 0.34° off). Check it on the
  robot with Manual Tuner B before trusting either.
- **Not verified on the robot.**

## 2026-11-06 — Turret lanyard compensation (off until lanyards go on)

- **Problem:** lanyards pull the turret by different amounts at different angles, their ranges overlap, and
  they drag differently going out vs coming back.
- **`Systems/Turret/LanyardMap`:** the pull is learned rather than modeled per lanyard.
  - It is a table of the extra volts needed at each angle: 5° bins over ±195° in encoder degrees from home,
    one table per direction.
  - Overlapping lanyards just add up in the same bins, so their number and where each starts don't matter.
- **How it learns (`TurretController`):**
  - It learns the feedback effort (p + i), averaged over 0.25 s at the averaged position. A single loop is
    spiky: going with a strong pull the turret stick-slips until the map has learned to brake it.
  - **Gliding:** the plan moves at steady speed (not starting or stopping) and the error is under 45°. This
    learns the plan direction's table.
  - **Stuck off target at a hold:** the error is under 15°. This learns both tables. The band is tight so a jam
    or a shove can't teach the map much.
  - The integral's share is handed to the map, so it isn't counted twice.
  - Lookup is at position + velocity·lead, where the turret will be when the command lands. Holds use the
    average of the two tables.
  - The map is clamped to ±`TURRET_LANYARD_MAX_VOLTS`.
- **Settings and storage:**
  - `TURRET_LANYARDS_ENABLED` (false) turns it on, live from the dashboard.
  - `TURRET_LANYARD_LEARNING` (false) keeps learning in every OpMode. The pit sweep learns either way.
  - The map is shared by every OpMode (static), saved to `FIRST/data/TurretLanyards/map.csv` at stop by
    TeleOp and the manual tuner, and resampled if the bin size or range changes.
  - Disabled, it adds exactly 0 and learns nothing; a sim test checks the controller output is identical.
- **Pit procedure:** Turret Manual Tuner, **X**.
  - It sweeps -185 → 185 at 30°/s, stopping 1.5 s at each end. The stops are what teach the ends.
  - Let it run about 8 sweeps (~4 min), then press X again to save. **back** clears the map.
  - Re-sweep whenever the lanyards change.
- **Sim** (`sim turret`, 4 made-up overlapping lanyards, up to 4.3 V of pull; the robot has none yet):

  | Loop | Case | Sweep | Holds | Spinning at 120° |
  |------|------|-------|-------|------------------|
  | 20 ms | no lanyards | 0.16° | 0.13° | 1.00° |
  | 20 ms | lanyards, compensation off | 3.52° | 3.99° | 5.62° |
  | 20 ms | after 3 sweeps | 0.50° | 0.16° | 0.93° |
  | 20 ms | after 8 sweeps | 0.39° | 0.06° | 1.03° |
  | 40 ms | lanyards, compensation off | 13.2° | 12.9° | 17.4° |
  | 40 ms | after 8 sweeps | 2.5° | 0.25° | 3.1° |

  With no lanyards and learning on, the map stays under 0.3 V. A slower loop needs more sweeps.
- **What was tried:**
  - An error band of 3° never learned, because the lanyards push the error past it.
  - Requiring smooth turret motion never learned going with the pull, because of the stick-slip.
  - Single-loop learning put spikes in the map.
  - Without the end stops, the ends of the travel stayed wrong.
- **Not verified on a robot.**

## 2026-11-06 — Turret: ±190° routing, wiring hard limit, vibration rejection, aim gate

- **Range:** the team says the turret travels 190° each way from home. Limits are now -190/+190, and profile
  `turret.range_deg` = 380 (manual).
- **Routing (`Shooter.routeTurret`, replaced by `TurretRoute`, see the spin note at the top):**
  - The aim angle comes out normalized (rawtt).
  - The candidates are rawtt-360, rawtt and rawtt+360.
  - Only candidates inside [MIN, MAX] count, and the one nearest the turret's current angle wins.
  - If none fits (impossible with 380° of travel), it clamps.
  - The 20° overlap (170..190 on each side) means a target crossing the back doesn't make the turret flip
    until it reaches 190.
  - Feedforward is used only when the chosen angle is rawtt plus a whole number of turns, which is always true
    unless it clamped.
- **Hard limit (`TurretBase.update`):**
  - Past MAX + `TURRET_HARD_LIMIT_MARGIN_DEGREES` (or below MIN - margin), it zeroes any volts that push further
    out. It can still drive back.
  - It uses encoder degrees from home, so **the turret must point straight at INIT**, the same rule as the
    autotuner.
- **Vibration: what was tried.** The flywheel shakes the turret, and a stiff loop chases the shake and adds
  servo-made shake on top.
  - Filtering the error in the loop added lag, made the turret hunt on stiction, and was worse on holds.
  - A high-pass shake detector fired on ordinary tracking error and softened calm tracking.
- **Vibration: what's used.**
  - Shake = RMS of the error's second difference / √6, averaged over 0.5 s. Slow tracking error barely moves
    a second difference.
  - It doesn't update while the plan itself accelerates.
  - Effective bandwidth = bw / (1 + ratio²), with ratio = max(0, shake - deadband) / reference.
  - That bandwidth has a floor of max(`TURRET_MIN_BANDWIDTH`, fraction·bw), capped at bw. Below about
    7 rad/s the hold sags on friction.
  - The gains are worked out again each loop from the effective bandwidth. This is pure arithmetic and
    allocates nothing.
- **Sim results** (`sim turret --assume --faults`, placeholder turret, 0.3° shake at the flywheel's rate plus
  an 8–14 Hz band):

  | Rejection | Hold, shaking | Servo-made shake | Volts change per loop | Spin |
  |-----------|---------------|------------------|-----------------------|------|
  | off | 0.81° | 0.75° | 0.83 V | 1.99° |
  | on | 0.33° | 0.15° | 0.40 V | 1.87° |

  With the flywheel off, both hold 0.22°. Across bandwidth / loop-time pairs, 14 / 40 ms went from 2.48 to
  0.51°. The limit is the shake itself: rejection stops the servos from adding to it but can't remove it.
- **Autotuner:** `TurretAutotuneEngine` turns rejection off so it measures the raw loop. Run DRIVING with the
  flywheel on, so the tuned bandwidth is one that copes with shake.
- **Aim gate (`TURRET_AIM_GATE_ENABLED`, off by default):**
  - The blocker opens only once the smoothed turret error (50 ms) has stayed inside 1° for 60 ms.
  - It stays allowed until the error passes 2°.
  - It goes through `Blocker.provideComponents(..., turret)`. Turn it on once real error numbers are known.
- **Tools:**
  - Turret Manual Tuner: B spins the flywheel at `FLYWHEEL_VELOCITY`. Telemetry shows the shake, the
    bandwidth now vs the base, and on target.
  - TunerTeleOp logs `tu_vib_deg`, `tu_bw` and `tu_on_target`.
  - Profile `turret.vibration_deg` is the measured shake RMS the sim uses.
- **Encoder reset (Through Bore):**
  - The hub keeps the encoder count between OpModes and only clears it on a power cycle. If the turret is
    moved in between, the old count is wrong.
  - `TurretBase` now calls `Encoder.reset` in its constructor. It does STOP_AND_RESET_ENCODER, then puts back
    the old run mode (RUN_WITHOUT_ENCODER if there wasn't one).
  - `new TurretBase(hardwareMap)` means the turret is at home now, so **it must point straight at INIT**.
  - `new TurretBase(hardwareMap, ticksFromHome)` resets and then makes the encoder read that value. Use it
    for the auto → TeleOp handoff.
  - `wrapIntoTravel` takes a turn off any value past the travel (350° → -10°).
  - Home (`startPosition`) is always 0 now, and the target starts where the turret is, so INIT never moves it.
  - The constructor clears every hub's bulk cache after the reset, because MANUAL bulk caching would still
    hand back the count from before the reset.
- **Not verified on the robot.** Everything above is from the sim with placeholder turret numbers.

## 2026-11-06 — Sim is usable: code-in-the-loop sims, --assume, real robot facts in the profile

- **Before:** every sim stopped with "missing inputs" (the profile had 27 of 361 entries). The Sim's
  turret was a speed limit and its flywheel had its own P controller, so neither said anything about
  the robot's code.
- **Code in the loop:** `:Sim:copyTeamCodeSources` copies the robot's pure-Java classes into the Sim build.
  It replaces `copyTeamCodeTuningSources`, and the stand-ins moved to `Sim/src/ftcStubs`. The classes:
  - `TurretController`, `TurretAutotuneEngine`;
  - `Flywheel`, `FlywheelPIDVSCoefficients`, `ShotGate`;
  - `Encoder`, `VoltageCompensator`, `MathUtil`, `DynamicTrapezoidalSum`, `InterpolationData`, `Clock`;
  - `TunedConstants`, `FlywheelConstants`, `ShooterConstants`, `TurretConstants`.

  They compile against `Sim/src/ftcStubs`: just the SDK interfaces that code calls, a `@Config`
  stand-in and a `HubVoltage` that returns the sim battery.
  - `sim turret`: the autotuner (stationary + driving) and the controller with the robot's constants,
    against `robot/CrServoTurret` (the plant validated for the autotuner). It covers steps, the robot
    spinning, time to turn, and `--faults`.
  - `sim flywheel`: the robot's `Flywheel` + `ShotGate`, against the sim flywheel and battery. It covers
    spin-up to the `FLYWHEEL_VELOCITIES` targets and 4-ball bursts with and without the gate, at 13.2 V
    and 12 V.
  - Both take `--loop-ms`/`--loop-jitter` and write CSV traces to `Sim/out/`.
- **TeamCode `Utility/Clock`:** `Flywheel.getSeconds()` and the no-argument `Encoder.updateFilteredVelocity()`
  now read `Clock.seconds()`. On the robot that's `System.nanoTime()` behind one null check. The sim
  points it at sim time. New robot code that's meant to run in a sim should read time the same way.
- **`--assume`** on any `sim`/`predict`: copies the profile to `out/assumed_profile.db`, fills only the
  gaps with PLACEHOLDER examples and runs there. All 7 sims run now (about 11 s for `sim all`).
- **`profile import-autotune [folder]`:** loads a finished Turret Autotuner run into `turret.*` as
  measured. New catalog entries: `turret.ticks_per_degree`, `servo_speed_dps_per_v`, `servo_time_constant_s`,
  `servo_dead_time_s`, `kinetic_friction_v`, `breakaway_friction_v`, `velocity_noise_dps`, plus derived
  `autotune_bandwidth_rad_s`, `step_overshoot_deg`, `spin_tracking_error_deg`.
- **`profile set ... --note "..."`** stores where a value came from.
- **Filled into `Sim/data/robot_profile.db`** (now 49 entries), each with a note:
  - flywheel motor from the goBILDA 5203 6000 rpm datasheet (1.47 kg·cm = 0.144 N·m stall, 9.2 A
    stall, 0.25 A free, 12 V);
  - from the code: max rpm 3568 (1665 ticks/s), gate tolerance, `wait_for_ready` 0 (gate off), voltage
    compensation on, velocity loop not on the hub, turret yes, turret range 180°, BRAKE, motor names, Pinpoint;
  - from CAD 10-31: launcher offset −0.0716 m, exit height 0.292 m (that hood pose; the hood is since
    removed from code), turret 68.27 ticks/deg, 4 balls held;
  - measured by Pedro's Foresight tuner on this robot: top speed 2.27 m/s (89.3 in/s), strafe
    efficiency 0.818 (73.0 / 89.3).
- **Still needed** (all sims say which): robot mass, size, wheelbase, track width, wheel diameter, drive
  motor and gear (CAD/build); flywheel MOI (CAD); battery voltage and resistance (HardwareMonitor); ball
  mass; exit speed ratio; feed interval; loop timing parts (HardwareMonitor); turret model (run the Turret
  Autotuner, then `profile import-autotune`). The launcher/shot sims still assume an adjustable hood
  (31–54°); with the hood gone, set `launcher.hood_min_deg` = `hood_max_deg` = the fixed angle once known.
- **First findings** (sim, placeholder plant, so trends only):
  - **Flywheel:** `FLYWHEEL_PIDVS_COEFFICIENTS` caps P at `maxP` = 3 V, so from rest the flywheel
    gets kv·target + 3 + 0.3 (I max) + 0.4 (S) V. That's about 9.9 V at 1300 ticks/s, not the full battery,
    and spin-up is slower than it could be. Raise `maxP` if spin-up time matters.
  - **Turret:** the untuned default turret constants (bandwidth 4) overshoot a 30° step by about 5.6°.
    After the autotuner it's about 0.4°.
- **Checked:**
  - `:Sim:test` passes: 57 tests, including 3 new code-in-the-loop tests.
  - `:TeamCode:assembleDebug` builds.
  - `sim all --assume` runs.

  Not run on the robot.

## 2026-11-06 — Turret: model-based control + Turret Autotuner (replaces the PIDF)

- **What was wrong:** the old `TurretBase` PIDF had kp/kd interpolated by position, two ki zones, an
  i "smash", a hold term, kV that learned itself, and a `reverse()` that flipped the shared static gain
  lists (calling it twice flipped them back). That's about 15 numbers tuned by hand, and none of them knew
  about dead time or loop time. It's gone, along with `TurretBasePIDFCoefficients` and `TurretBaseTuner`.
- **New control (`Systems/Turret/TurretController`, pure Java):** the turret is modeled as a speed per volt
  (`TURRET_GAIN`, ticks/s per V), a lag (`TURRET_TIME_CONSTANT`), a dead time, and kinetic + breakaway
  friction (volts).
  - A **plan** runs one lead time ahead (lead = dead time + one loop). It aims where the target will be,
    using its velocity and acceleration. It is speed and acceleration limited, and stops in whole loops.
  - **Feedforward** is the exact inverse of the lag over one held loop, plus friction by direction
    (breakaway until it actually slides). On 50 ms loops, the usual `(v + tau·a)/K` overdrove a 30° step
    by 17°.
  - **Feedback** compares the turret with the plan from one lead time ago, so it doesn't fight the delay.
  - **Gains** come from pole placement, `(s + a·w)(s² + 2ζw·s + w²)`, so the only tuning knob is
    `TURRET_BANDWIDTH` (plus damping, integral ratio and friction scale).
  - Hold deadband 0.15°. The integral only runs outside the deadband and doesn't wind up while saturated.
- `TurretBase` keeps its public API (`setPosition`, `setTargetVelocity`, `update`, getters, voltage
  compensation). New: `readSensors()` + `setVolts(left, right)` for driving the servos directly, and
  `getReference()`. `TurretConstants` is read every `update()`, so dashboard edits apply live.
  `TeleOpBaseOpMode` no longer sets coefficients.
- **Turret Autotuner** (`Tuners/TurretAutotuner`, engine in `TurretAutotuneEngine`, no hardware in it):
  - **STATIONARY**, the first tune: robot still, turret straight at INIT. It goes through:
    1. sensor noise;
    2. which way it turns (a reversed encoder is handled, with a warning);
    3. each servo alone (warns if one is dead, faults if it pushes the wrong way);
    4. 4 friction ramps;
    5. 8 open-loop speed steps, fit for gain, lag and dead time;
    6. closed-loop tests (±30° steps, ±2° steps, holds, a 25° 0.7 Hz sine) while it searches bandwidth
       up and down. It also nudges friction scale, integral and gain from the results;
    7. a final check.

    It faults (servos off) on a jam, fighting servos, no movement at 9 V, or passing the ±190° travel.
    About 2–3 min.
  - **DRIVING**, the retune: start, then press A. Until A, nothing else runs, so A doesn't also fire its
    normal action. Then drive like a match (drive, intake, blocker, LB = flywheel at the default speed),
    spinning as much as you like, while the turret holds a field-fixed direction (swinging around at the ends,
    see the spin note at the top). Stop turning for a second now and then: that's when it checks the shake, and
    2 shaky windows start a lower-bandwidth trial it keeps only if it helps (see the shake check note at the top).
    It never raises the bandwidth or changes the gain (see the autotuner fixes). It finishes on its own after ≥40 s, ≥20 s of turning, and
    3 calm windows.
  - Results show on telemetry and save to `/sdcard/FIRST/data/TurretAutotune/<stamp>-<mode>/`
    (`result.csv`, `samples.csv`). `pullTuningData` pulls them, and `analyzeTuning` writes the newest
    finished one into `TunedConstants` (`TURRET_*` model values, `TURRET_AUTOTUNE` = which run).
  - `PAD_LOOP_TO_MS` pads stationary loops to match TeleOp. The controller already adapts to loop time,
    so this is optional.
- **Turret Manual Tuner** (`Tuners/TurretManualTuner`): dpad steps, A toggles a sine. Edit
  `TurretConstants` on the dashboard. Shows p/i/d/f/v, kp/kd/ki, reference vs position. `LOOP_TIME`
  adds a sleep to mimic match loops.
- `TunerTeleOp`: the `tu_kv` column is now `tu_ref`, and the meta has the turret model instead of kV.
- **Checked off-robot only:** a sim turret (gain, lag, dead time, stick-slip friction, encoder noise,
  random loop times) ran against the real engine and controller. On 8–14 ms and 40–65 ms loops:
  - normal, reversed encoder, one dead servo and a slow sticky turret all finish;
  - fighting servos and a jam fault in 3.6 s;
  - spinning-robot tracking RMS: 7.4° → 1.1° (fast loops), 9.9° → 1.6° (slow loops);
  - driving mode finds a changed gain within 1% (2589 vs 2600).

  **Not run on the robot yet.** The default model in `TunedConstants` is a guess (bandwidth 4, soft) until
  the first STATIONARY run.

## 2026-11-05 — Shot gate, and comments cleaned up everywhere

- **Problem:** a ball that reaches the flywheel during a speed dip (from vibration or the ball before it)
  leaves slower and flies a different arc.
- **Shot gate** (`Systems/Flywheel/ShotGate`, used by the `Blocker` subsystem): left trigger opens the
  blocker only once the flywheel has been inside the enter band for `SHOT_GATE_DWELL`. It closes again when
  speed leaves the hold band (after `SHOT_GATE_MIN_OPEN`), or after `SHOT_GATE_PULSE` if metering is on.
  With the gate on and the flywheel off, the blocker stays shut. Off by default: `SHOT_GATE_ENABLED`.
- **Simulation** (off-robot, synthetic flywheel with 0.05 s servo, 0.08 s ball travel, 0.1 s ball spacing):
  - Ball speed error at contact went from −80 ± 54 to about 0 ± 25 ticks/s with vibration, and from
    −66 ± 39 to −3 ± 3 with ball dips only. Fire rate dropped from 9 to about 3 balls/s.
  - Under heavy vibration it only went from 106 to 81, because a ball already past the blocker can't be
    held back. Fix heavy vibration mechanically (balance, belt, mounts).
  - A gate that waits too long before closing (min open 0.15 s) barely helps, because the next ball is
    already through.
- **`ShotGateTuner`:**
  1. Spins to `TARGET_VELOCITY` and measures vibration for 3 s with the blocker shut.
  2. You shoot with the gate off and on (gamepad 2 A), then press BACK.
  3. It shows contact speed, fire rate, balls per opening, recovery time and how often P is maxed out, then
     suggests gate values and, if needed, a higher `P_MAX`.
  - Shots are counted from velocity dips while feeding (`ShotDipDetector`). Heavy vibration can be
    miscounted as shots, so compare the count with the balls you actually fired.
- **Comments:** a one-off script applied the code style rule to all `TeamCode` and `Sim` Java. It removed
  1505 comment lines and normalized 81 to `//blah`. Short `///` notes and commented-out code in team files
  were kept. It checked that each file's code is unchanged once comments and whitespace are ignored. The
  originals stay in git history.

## Code style (standing rule, from Pranav)

- Only comment when people literally won't get the code without it. Most code gets nothing.
- Keep it rough and to the point, like `//turret`, `//radians`, `//0 = auto`. Not clean, not polished.
- Use `//blah`, with no space. Short `///` notes are fine, like `/// Index 1 is the right crservo.`
- Reference: github.com/PranavDharanipathy/AthletechPremiers2526 has about 2% comment lines. Use a bit
  less than that.
- **Format** (same reference):
  - 4 spaces, no tabs. Blank line after a class's `{`.
  - **One statement = one line**, however long it gets. No hanging wraps (args continued on the next line with the `);` stuck to the last one).
  - The only multi-line forms are Pranav's:
    - block lists: the line ends with `(`, then one item per line (8 extra spaces), then `)` / `);` / `) {`
      on its own line at the statement's indent. Used for long constructor/argument lists and big `if (` conditions;
    - builder chains with one `.call()` per line;
    - array initializers.
  - `}` then `else {` / `else if (...) {` on the **next line**. `} catch` stays on the same line.
  - Methods/constructors: blank line after `{` when the body is 2+ lines, no blank line for one-liners
    (getters, `this(...)`).
  - Spaces: `if (`, `for (`, `new double[] {a, b}`. Short `if (x) y;` on one line, no braces.
  - Fields sit next to the methods that use them, in small groups separated by blank lines.
  - `python tools/pranavfmt.py <files>` does the line-joining/else/blank-line part. It was run on all of
    TeamCode (except `Tuners/pedro`, Pedro's own code) and Sim on 2026-11-06. `javap -c -p` showed
    identical bytecode for all 316 classes, so formatting only.

## 2026-11-05 — Turret aim feedforward

- Instead of `-kv * robotAngularVelocity`: `Shooter.getTurretAimRate` gives the exact rate (relative to the
  robot) the turret must turn to stay on the goal. It uses the robot's vx, vy and ω (Pinpoint velocity is
  field-frame) and the turret offset, so strafing past the goal is covered too, not just spinning. It was
  checked against a numeric derivative (error 2e-5 rad/s).
- `TurretBase` feeds that rate to `TurretController` as the target velocity (the old learned `TURRET_KV`
  is gone, the model gain does that job now, see the turret control note above).
- No feedforward when the aim is clamped at a travel limit. `TURRET_AIM_LEAD_TIME` (0 = off) aims from
  the pose predicted ahead.

## Threading on the Control Hub (standing rule, from Pranav)

- The Control Hub is an RK3328: 4× Cortex-A53, Android 7. The RC app, DS comms, the web server and Sloth all
  share it, and every REV hub call goes over one serial link per hub.
- Background work: one daemon thread per job at `Thread.MIN_PRIORITY`, sleeping between batches (250 ms is
  plenty). File I/O only, no hardware. Stop it when the OpMode stops. `CsvLogger` and the HardwareMonitor
  writer follow this.
- Loop thread: no per-loop allocation in hot code. Reuse arrays and StringBuilders, no `String.format`,
  boxed numbers or varargs in the loop. GC pauses on the Hub show up as loop spikes.
- Android API 24 (minSdk): no `String.join` or other API 26+ calls without desugaring.

## 2026-11-06 — TunerTeleOp + automatic hypertuning

- `TeleOp/TunerTeleOp`: the match TeleOp (alliance picked in init with gamepad 1 X/B) that logs every loop to
  `/sdcard/FIRST/data/TunerTeleOp/<yyyyMMdd-HHmmss>/loop.csv`, plus `meta.csv` with the settings in use.
  - Columns: loop time, the extra read time logging added, flywheel raw/filtered velocity, filter Hz, the
    PIDVS terms and both voltages, turret position/error, raw/filtered velocity, aim velocity, PIDF+V terms
    and kV, pose and velocity, and shooter/aim/blocker/gate/intake state.
  - Hub voltage is read every loop for the voltage fit. Its cost is logged and subtracted.
- On every TeamCode build, `:TeamCode:pullTuningData` runs `adb pull` on any new or grown sessions into
  `tuning-data/sessions/` (git-ignored). It's quiet when no robot is on adb. Then `:Sim:analyzeTuning`
  writes `Constants/TunedConstants.java` and `tuning-data/REPORT.md`, but only when something changed.
- The analyzer (`Sim/.../tuning/TuningAnalyzer`) uses the robot's own `Encoder`, `VoltageCompensator` and
  `FlywheelVoltageFilterFit` sources, copied in at build time with a 1-line `DcMotorEx` stand-in in
  `Sim/src/tuningStubs`. It uses the last 5 sessions with at least 300 rows. It sets:
  - loop time (median, which also sets `FlywheelTuner` `LOOP_TIME`; p95 too);
  - flywheel and turret voltage filters (replay fit on the real hub voltage);
  - flywheel and turret velocity filter settings (bandwidth range and score memory, by smallest next-reading
    prediction error over 36 combinations, kept unless at least 2% better; the stale time is 2.5× the hub's
    update period);
  - shot gate bands and dwell (from flywheel vibration while settled and not feeding);
  - turret vibration, and the turret model from the newest finished Turret Autotuner run.
  Gains are never changed. Each value is kept when there isn't enough data.
- `FlywheelConstants`/`TurretConstants` read those values from `TunedConstants`.
- Turret: `TurretBase` now runs the `Encoder` velocity filter, and D uses `kd * (targetVelocity - velocity) / 1000`,
  the same scale as the old per-ms error difference. That means no derivative kick on target jumps, and less
  noise. Recheck kd.
- Checked off-robot: a synthetic 90 s session gave loop 10 ms, vibration 11 ticks/s at 9.2 Hz (bands 22/44),
  hub update period, voltage sag and learned kV. The generated file compiles. `CsvLogger`'s number
  formatter was checked against edge cases. Not run on the robot yet.

## 2026-11-05 — Flywheel velocity filter (Encoder)

- `Encoder.updateFilteredVelocity()` replaces the old 1D Kalman filter
  (`setupVelocityKalmanFilter`/`runVelocityCalculation`). `Flywheel` uses it for `getCurrentVelocity()`.
  `getRawVelocity()` is the hub value.
- How it works: 7 velocity+acceleration Kalman filters (0.5 to 32 Hz). Hub noise is estimated live (median
  of second differences, quantization floor). Each filter gets a likelihood score, and the output is the
  weighted mix. Filters that fall far behind snap to the best one. There is nothing to tune.
- Off-robot test on synthetic hub data: steady noise went from 11.6 to 3.4 RMS, about what a tuned low-pass
  gets. On spin-up it did 14.8 against 71.1 for the low-pass. It kept ball dips (lowest 1297 vs true 1303).
- The PID now sees a smoother signal, so recheck the flywheel gains. `FlywheelTuner` shows `Raw Velocity`
  and `Filter Hz`.

## 2026-11-05 — Shooter subsystem

- `Systems/Shooter` (a `Subsystem`, based on the DECODE one) owns the flywheel and turret in TeleOp. Both
  TeleOps call `shooter.start(alliance)` and `shooter.update()` after `follower.update()`.
- Controls: gamepad 1 left bumper turns the flywheel on and off (off at start, where TeleOp used to spin up
  right away). Gamepad 2: right bumper toggles auto aim. *(Superseded: see "Relocalization at the GARDENS" above for
  the main button, the dpad and the turret home buttons.)*
- Goal switching: `Field/Goal` (RED, BLUE) picks the close or far aim point from the robot pose
  (`y > FAR_ZONE_CLOSE_ZONE_BARRIER` means close). The coordinates live in `FieldConstants` and are NaN
  until measured. While they're NaN, the turret holds home and the flywheel uses
  `FLYWHEEL_DEFAULT_SHOOT_VELOCITY`. Fill all eight goal values (x and y, close and far, per alliance) in the
  same frame as `START_POSE`.
- `FieldConstants.START_POSE` is (0, 0, 0), only so Pedro has a pose. Real aiming needs a real start pose,
  or relocalization at a known spot.
- `TURRET_AIM_DIRECTION` was -1 (from DECODE's `heading - angle`) and wrong. On the robot (2026-11-06) the turret turned
  the same way as the robot's spin, twice as fast. It's +1 now, which also mirrored the DRIVING autotune's targets.
  Not re-checked after the flip. `TURRET_OFFSET_X = -2.82` is from CAD. Shot lead (`SHOT_TIME_OF_FLIGHT`) and future-pose speed
  (`FLYWHEEL_SPEED_ADJUSTMENT_T`) are 0 (off) and use Pedro's `follower.velocity()`. There's no camera or
  pose estimator yet. The turret gains are still zero, so it won't move until tuned.

## 2026-11-05 — Flywheel overspeeds when the whole robot runs

Symptom: with everything running, the shooter twitches and shoots high. Not yet confirmed on the robot
(adb was offline), but the code had two problems that fit:

- **Velocity came from loop timing.** `GeneralConstants.BULK_CACHING_MODE` is MANUAL, so the encoder
  position is the bulk read from the top of the loop. `Flywheel.update()` timed it later, after
  `follower.update()` and `intake.update()`, and every 20 ms after its own voltage read. So dt varied with
  the rest of the robot's work, and velocity spiked both ways. P is clamped to -2/+3 V, so spikes that
  saturate it average out positive: a net push that makes the flywheel run fast. `FlywheelTuner` sleeps
  60 ms per loop, which hid this. **Fix:** control now uses `getVelocity()` (the hub's own
  counts-per-second, part of the bulk read). `getVelocityEstimate()` still returns the old value, for
  comparison only.
- **Voltage came from whichever hub was listed first.** **Fix:** `Flywheel.voltageSensorForMotor`
  matches the motor's connection info (`"...; module N; port P"`) to its hub's (`"...; module N"`).
- **`getCurrent()` isn't used for control.** Bus voltage is already the right thing to compensate for.
  Current is a separate hub transaction per read, and adding current feedback would mean retuning
  everything. HardwareMonitor sessions already log every motor's current for diagnosis.
- **New `FlywheelLoadTuner`:** drive normally with the match controls while the flywheel holds
  `TARGET_VELOCITY`, then press BACK on gamepad 1. It reports speed error under load (average, RMS,
  worst over/under, % time too fast), RMS noise of the old measurement, loop times, and a voltage-filter
  fit from this run, using the flywheel hub's voltage read every loop. `FlywheelVoltageFilterTuner` now
  reads the same hub.
- **Retune check:** gain units are unchanged (volts per tick/s), but P/D now see a much cleaner signal.
  Recheck `FlywheelTuner` and `FlywheelLoadTuner` before trusting the old gains.
- **Turret:** `TurretBase` now uses the left turret servo's hub, through the shared
  `Utility/HubVoltage.forDevice` (the flywheel uses it too). Caveat: hub servo ports are powered from a
  regulated rail, not straight from the battery, so scaling the turret command by battery voltage may
  overcorrect when the battery sags. If the turret gets twitchier under load, that's the likely
  reason. Not changed yet: it needs a test with the turret holding position under load.
- **Turret compensation switch:** `TurretConstants.TURRET_VOLTAGE_COMPENSATION` (default true, the old
  behavior), live as `VOLTAGE_COMPENSATION` in `TurretManualTuner` (was `TurretBaseTuner`). Off skips the voltage reads and divides
  by `TURRET_NOMINAL_VOLTS`, so gains keep their units. To compare, hold one target under load (drive
  plus intake) with it on, then off. The manual tuner doesn't run the drive, so that load comes from
  TeleOp, or from `TurretLoadTuner`. Retune with the setting you keep.
- **`TurretLoadTuner`:** match controls on gamepad 1, turret holding `TARGET_OFFSET_DEGREES` from its
  position at init (kept inside the software travel limits). Gamepad 2 A switches compensation, BACK
  finishes. It reports ON vs OFF: RMS and worst error, % time off target, RMS turret speed (shake), power
  reversals/s, RMS power, and hub voltage. It skips `SETTLE_SECONDS` after each switch. Degrees use
  `TURRET_TICKS_PER_DEGREE` (65.738, unverified).

## 2026-11-04 — Flywheel voltage filter is tunable, with an automatic tuner

- All five flywheel terms were already compensated: `toPower(p + i + d + v + s)` divides the sum by
  filtered battery volts. Don't compensate individual terms too, or it divides twice.
- `FLYWHEEL_VOLTAGE_SAMPLE_PERIOD` and `FLYWHEEL_VOLTAGE_FILTER_TAU` are real tuning values, now live
  in `FlywheelTuner`. `FLYWHEEL_NOMINAL_VOLTS` is deliberately not: it is only used until the first
  valid sample, which is normally taken in the first `update()`, and it is not a gain.
- **`FlywheelVoltageFilterTuner`** runs by itself: 4 cycles of spin to the top of
  `FLYWHEEL_VELOCITIES` for 3 s, then coast for 2.5 s, recording raw voltage. It stops early above
  130% of target or if the flywheel isn't at 25% of target after 1.5 s, both measured over 100 ms.
  Results go to telemetry and the robot log; copy them into `FlywheelConstants` by hand.
- **Single-loop flywheel velocity is noisy.** The first robot run stopped on a false-looking
  2200 ticks/s reading from `Flywheel.getCurrentVelocity()`, which is one loop's position change over
  dt, unfiltered. Don't use it for safety checks. The controller's P and D use the same value, and
  `FlywheelTuner` sleeps 60 ms per loop, so gains are tuned on a less noisy signal than TeleOp sees.
- **`FlywheelVoltageFilterFit`:** the period is the shortest that keeps reads under 10% of loop time.
  Tau is the candidate with the lowest RMS error, replaying the recording through the real
  `VoltageCompensator` against a 60 ms centered average.
- **Limits:** the run fires no balls, so it sees spin-up sag rather than shot sag, and the 60 ms
  reference window influences the tau it picks. Each read costs loop time, so judge a new period by
  loop time as well as tracking error. The turret has its own constants and doesn't use this.
- **Verified off-robot only:** the fit ran on synthetic voltage traces (noise, sag speed, slow loop,
  slow read, too little data). Results moved the expected way. **Not yet run on the robot.**

## 2026-11-04 — Hood removed from TeamCode

- **Removed:** `Systems/Hood/` (`Hood`, `HoodAngler`), `Constants/HoodConstants`, the
  `HoodAnglerTesting` tuner, the hood fields and setup in `TeleOpBaseOpMode`, and the `HOOD_*` values
  in `ShooterConstants` and `DeviceConstants`. All recoverable from git history. Older notes below
  that mention `HoodAngler` or a second hood servo are history, not open work.
- **Kept and renamed:** `HOOD_DISTANCES` is now `FLYWHEEL_DISTANCES`, because it is the distance key
  for `FLYWHEEL_VELOCITIES`. `HoodInterpolationTuner` is now `FlywheelInterpolationTuner`, because
  it also collects that table.
- **Not changed:** `Sim`, `Sim/data/robot_profile.db`, `docs/` and `tools/` still model a hood and
  hood-angle aiming, so sim shot predictions don't reflect the robot without one yet.

## 2026-11-03 — NextFTC v2 + Ivy for Pedro 3 autonomous commands

- **Installed:** `dev.nextftc.v2:control`, `hardware`, `robot` 0.3.0 and
  `com.pedropathing.ivy:pedro:1.1.1`, in `build.dependencies.gradle`.
- **Why not NextFTC v1:** `dev.nextftc.extensions:pedro:1.0.0` (its only release, 2025-09) pulls
  `com.pedropathing:ftc:2.0.0`, the artifact that mixes `core` 2.x and 3.x. Its source also calls
  Pedro 2 methods that Pedro 3's `Follower` no longer has (`followPath`, `breakFollowing`,
  `isTurning`, `turnTo`). v1 and v2 share class names, so they can't be installed together.
- **Why v2 fits:** it pulls Sloth 0.3.2 (same as `tuning`, and the `slothboard` prefix), stdlib
  2.3.0 (resolved up to our 2.4.10), minSdk 24 and Java 8 bytecode. No FTC SDK comes in transitively.
- **Path following:** NextFTC v2 has no Pedro integration of its own. Its scheduler is Ivy, and
  `PedroCommands.follow(follower, path)` / `hold(...)` use the Pedro 3 `Follower` API. The artifact
  declares no Pedro dependency, so it runs against whichever Pedro we declare. Nothing calls
  `follower.update()` for you: do it in `periodic()`.
- **`Systems/AthletechRobot` is required.** At startup NextFTC's `RobotScanner` sets a Driver
  Station global error if TeamCode has no `NextRobot`, and warns and picks none if there are several.
  This applies even if no NextFTC OpMode exists. It builds the class during app startup and keeps
  the instance, so its constructor must not touch hardware.
- **Don't extend `BlazeOpMode`.** `dev.anygeneric.blazeftc` was compile-only for NextFTC and isn't
  in our APK (checked with dexdump), so it would throw `NoClassDefFoundError`. `NextMotor` only
  reaches Blaze when `blazeEnabled` is true, which plain `NextOpMode` never sets.
- **`PedroCommands.hold(follower)` reads the pose when the command is built**, not when it runs.
  Inside a sequence, use `hold(follower, pose)` or wrap it in `Commands.lazy`.
- **Version pairing:** if `tuning` moves to a new Sloth version, check NextFTC v2's Sloth at the same
  time. Sloth, `slothboard` and NextFTC v2 all have to agree.
- **Verified off-robot only:** `clean :TeamCode:assembleDebug` passes, the runtime classpath has
  no `com.pedropathing:ftc` or NextFTC v1, and an example `NextAutonomous` using `PedroCommands`
  compiled (then deleted). **Not yet run on the robot:** scanner registration under Sloth, including
  `kotlin-reflect` 2.3.0 next to stdlib 2.4.10, is unproven until it boots on a Control Hub.

## 2026-10-31 — Merged `shooter-turret-v1`; pruned unrelated code

- Merged `origin/shooter-turret-v1` (servo-free pollen shot experiment) into `main`.
- Kept: turret, flywheel, hood, intake, drive, `shooter/` v1 package, Turret Diagnostic, Pollen
  Shot Experiment, HardwareMonitor + `Monitored*` read caches, `RobotLogger`, Pedro tuners, the
  whole `Sim` module (including its tests), and the turret/shot docs and tools.
- Removed: the whole `TeamCode/src/test` folder (JUnit dependency, `tools/run_tests.sh` and
  `tools/run_utility_tests.sh` with it); the unused FIArm/Field collision geometry (all of
  TeamCode's `.kt` sources); unused `Calculations`, `DriverConstants`, `FieldConstants`,
  `VisionConstants`, `DriverKeybinds`, `Resistor`, `DigitalDistanceSensor`, `TickrateChecker`;
  and `docs/CHECKPOINT.md` / `docs/CODE_AUDIT.md`. All recoverable from git history.
- TeamCode has no Kotlin sources now. The Kotlin plugin stays so `.kt` files can return without
  touching the version matrix below.
- TeamCode controller changes now have no automated tests; only `:Sim:test` runs off-robot.
- **Pollen Shot Experiment** is now an automatic flywheel-only sweep (voltage steps -> kv/ks fit,
  then RPM setpoints -> rise/settle/ripple, optional feed shots -> dip/recovery). It ends on a
  telemetry **Done!** summary and saves CSVs. `ShotGate`, `PollenPhysics` and
  `tools/summarize_pollen_shots.py` were removed. See `docs/POLLEN_SHOT_EXPERIMENT.md`.
  Ranges: flywheel 500–2100 RPM, battery 12–14.5 V. Voltage steps stay at or below 12 V (can't
  command more than the battery), and the sweep stops once it passes 2100 RPM.

## 2026-10-31 — Turret diagnostic and characterization OpMode

- New OpMode **Turret Diagnostic** (`Testing/TurretDiagnostics/`). It collects data and diagnoses
  the turret shake/overshoot; it is not a controller replacement. Procedure and interpretation:
  `docs/TURRET_DIAGNOSTIC.md`.
- **Modes** are operator-selected and labeled in every file and row:
  - passive observation;
  - the unmodified baseline `TurretBase`, with its P/I/D/hold terms, gains and own dt recorded;
  - bounded open-loop pulses and holds (`+,+,-,-` per level, neutral between), or a jog.
  - A per-step summary separates friction (breakaway, dead time), play (reversal minus restart
    dead time), inertia (coast decel) and loop timing.
- **Safety:**
  - Starts passive. Init never writes the servos, and arming is only possible after START.
  - Arming needs: operator-confirmed scale, reference, bounds, margin, command limit, assumed
    decel and max speed (all NaN by default); deadman released; still turret inside the safe zone.
  - Commands flow only while RB is held. Both servos go neutral on any of: deadman release,
    cancel, stale/failed/implausible feedback, out-of-bounds, predicted stop beyond the margin,
    overspeed, frozen encoder under command, command opposing motion, settings edit, full buffer,
    OpMode stop, exception.
  - Every command passes through `GuardedCRServo` (clamp + gate), including the baseline
    controller's, via its device-injection constructor.
  - Software bounds are operator inputs and do not guarantee wiring safety.
- **Changes nothing:** no constants, servo/encoder directions, gearing values or travel limits.
  `ShooterConstants.TURRET_TICKS_PER_DEGREE` stays 65.738; the diagnostic's own confirmed scale is
  used only for its targets and recordings.
- **No chassis drive** in this OpMode: `DrivetrainConstants` (lf/lb REVERSE) and
  `TeleOpBaseOpMode` (rf/rb REVERSE) disagree on drive directions. Driving-induced shake needs
  TeleOp integration after the directions are verified.
- **Verified off-robot only:** 45 new JUnit tests (every stop path, schedules, guard clamping of
  the real `TurretBase`, step analysis, file output). `:TeamCode:testDebugUnitTest`,
  `:TeamCode:assembleDebug` and `:Sim:build` pass; lint adds no warnings. Not yet run on the robot.

## 2026-10-31 — Ongoing code audit and pollen-only simulation support

- Andrew requested a checkpoint before a larger issue. Cross-agent handoff:
  `docs/CHECKPOINT.md`. Latest verification: 54 Sim and 20 TeamCode tests pass;
  Sim build and Android debug assembly pass. Audit remains incomplete.

- Findings and verification scope are tracked in `docs/CODE_AUDIT.md`; the audit
  is incomplete and no physical robot validation has been performed.
- Baseline flywheel/turret arithmetic, reset/readiness checks, voltage fault handling,
  and storage backlog/retry behavior now have direct TeamCode regression tests:
  `./gradlew :TeamCode:testDebugUnitTest`. Legacy desktop shooter checks still use
  `bash tools/run_tests.sh`; they do not prove baseline robot performance.
- Sim profiles now require binary `launcher.pollen.enabled` and
  `launcher.nectar.enabled`. The shared profile is 1/0 (manual, October 31 hardware
  notes). Pollen simulations no longer require invented nectar fit data. Disabled
  shot types are excluded and old simulated derived results cleared through normal
  priority-aware writes; historical measurements are retained.
- Recorded the documented 72 mm flywheel, one motor, 1:1 gearing, 6000 rpm free
  speed, and estimated 31–54° launch range in the shared profile as manual values.
  These launch angles are geometric estimates, not calibrated servo endpoints.
  Inertia, losses, ball masses, and most real uncertainty/timing measurements remain
  missing. No example physics were written to the shared database.

## 2026-10-31 — Shooter/turret hardware from CAD + team decisions

Source: Onshape doc `231d0d2ccd57c7da2e3d018d`, read-only via API on 10-21 and again on 10-31.
**Frames:** CAD −Y = robot forward, +X = robot left, +Z = up; floor at CAD z = −0.37 in; drive
center = CAD (−4.82, 1.14). The Shooter Assy sits unrotated in Main at offset (−5.6, 2.03, 13.53) in.
The CAD changed between 10-22 and 10-29. On 10-21 the turret drive read as 3:1 with 3 servos.

**Team decisions (Pranav, 10-31):**
- Pranav's `Flywheel` / `TurretBase` / `Hood` are the baseline. PID and third-party libraries are fine.
- The turret runs CR servos in **continuous mode**, because loop dt is too long for positional control.
- The turret encoder is **quadrature through a motor port** (`turretEncoder`).
- Turret travel has **no hard stop**; it is limited in software.
- **Two hood servos, mirrored**, matching the CAD (see the hood note below).
- Keep the 10-21 `teamcode/shooter/` v1 package for now; reconcile it with `main`'s subsystems later.
- **Accept a longer minimum shot distance** instead of redesigning the hood.
- **4 balls held**; fire them as fast and as accurately as possible.
- Use `Sim/` and the robot profile, not the 10-21 standalone shooter test suite.

**Game pieces:**
- POLLEN ≈ 2.80 in (manual 71 mm; a real ball may be ~2.86–2.9 in, like a 3 in wiffle ball).
- NECTAR ≈ 3.6 in (manual 91 mm).

**Turret:**
- Axis at CAD (−4.82, 3.96): 2.82 in behind drive center, centered left/right.
- Drive: 2× Axon Max. Each goes servo → 60T → **20T** pinion (3× up), then that shaft's **24T → 72T**
  turret gear (3× down). **Net servo : turret = 1 : 1.** The CAD gear relation only models the 24T→72T
  stage.
- Encoder: REV Absolute Encoder on its own **24T meshing the 72T**, so it is **geared 3:1**.
  - Quadrature at 8192 CPR → **`TURRET_TICKS_PER_DEGREE = 8192·3/360 ≈ 68.27`**. The current
    `ShooterConstants` value, 65.738, is last season's.
  - The absolute output repeats every 120° of turret.
- No motion limits are modelled on any turret mate.

**Flywheel:**
- 1× goBILDA 6000 rpm bare motor, 20T→20T GT2 belt (1:1).
- 72 mm Rhino + 2× 72 mm SWYFT wheels, r = 1.42 in.
- Axle **9.30 in above the floor**.

**Hood: it rotates about the flywheel axle.**
- The inner shooter plates pivot on bearings concentric with the flywheel shaft, and the hood is fixed
  to them. The hood surface is an arc at r = 4.14 in.
- **Gap = 2.72 in**, so POLLEN squeezes about 2–5 mm. **NECTAR (3.6 in) can't pass: the shooter is
  POLLEN-only.**
- Linkage: servo → Part 6 crank (1.185 in) → Part 7 coupler (1.345 in) → plate (4.608 in from the axle).
  The ground link is 6.348 in.
- It is non-Grashof, so the servo has toggle ends.
- **Launch elevation ≈ 31°–54° usable** (60° at toggle). That is about 0.26° of launch per servo degree
  near the current 39.4° pose.
  - This is a geometric estimate: tangent exit at the hood tip, collisions ignored.
  - Re-run with `python3 tools/hood_fourbar.py`.
- **Two hood servos (confirmed 10-31), mirrored and rigidly locked to one hood.** Their axes face −X
  and +X. They must always get mirror-image commands or they will fight, stall and heat up.
  `HoodAngler` currently drives only one (`hood`), so it needs a second servo with the opposite
  direction.
- **Exit point** (current hood pose, turret 0): ball center ≈ 11.5 in above the floor, 0.78 in ahead of
  the turret axis, ≈ 0.6 in right of it. It moves on an r = 2.82 in arc about the axle with the hood.

**Limelight 3A:**
- Chassis-mounted, on the Inner Transfer Plate, so it does **not** turn with the turret.
- Lens 3.78 in forward of drive center, centered, 9.18 in above the floor.
- **Pitched 50° up**, with 0° yaw and 0° roll.
- With an ≈42° vertical FOV it sees only 29°–71° of elevation: a target Δh above the lens is visible
  only within ≈ 1.8·Δh. The team says to ignore this for now (10-31).

**Open:**
- `HoodAngler`: add the second, mirrored hood servo.
- Software turret travel limits (the wiring range).
- Fill the `launcher.*` / `robot.*` inputs in the robot profile from these numbers through
  `Catalog`/`ProfileDb`, as source `manual`.
- Reconcile the 10-21 `teamcode/shooter/` v1 package (ESO/ADRC controllers, ballistics/aim solver,
  plain-javac tests via `tools/run_tests.sh`) with `main`'s subsystems. It still assumes 3:1
  servo:turret and ±200° travel, so update its constants or retire it.

## 2026-10-23 — Sim undeterminables: loop timing, static charge, disconnects

- **Loop timing** (`LoopTimingModel`, `LoopControlSim`, `sim loop`): a loop = logic + uncached motor
  reads + I2C sensor reads + other reads + motor/servo writes (each call lognormal around its mean),
  plus jitter and rare spikes. Sensors are read at the end of the read phase, motors commanded mid
  write phase; IMU / odometry computer are sample-and-hold at their own rates. PD turn and drive
  moves and the flywheel's ready-check-and-feed run once per loop against the full physics. Also:
  odometry integration error (straight vs arc steps), pose age, timer overshoot
  (E[T^2] / 2E[T]) and the chance of missing a sensor pulse of length w (E[(T - w)+] / E[T]).
  Loop-timing flywheel error feeds `ShotPredictor` unless `launcher.rpm_error` is measured.
  `MonitorImport` fills the loop's parts from HardwareMonitor `system.csv` / `devices.csv` /
  `events.csv` (cache hits and repeated writes do not count as hub calls).
- **Static charge** (`RobotCharge`, `TileChargeGrid`, `ChargeSim`, `Electrostatics`, `sim esd`):
  each robot is two conductors (frame, electronics) with a Maxwell capacitance matrix, leakage
  integrated implicitly (stable for the 470 ohm strap's sub-microsecond time constant). Charging per
  meter rolled, x factor when sliding, x factor per meter of wheel slip; humidity scales charging
  (e-fold per `esd.charge_humidity_scale_pct`) and leakage (decades per 10% RH). Tile charge sits on
  a 24x24 grid, leaks to the floor and creeps sideways (D = 1 / (rho_s * eps0 * eps_r / t)), and
  induces charge on a robot above it. Discharges bring two conductors to one potential, releasing
  1/2 dV dq. Field charge carries over between matches (competition day). Every coulomb is booked:
  `ChargeSim.Ledger` for this robot and a whole-field balance per match; a test holds both to 1e-9.
- **Wheel slip** in `MecanumDrive` (quasi-static): a wheel whose motor out-pushes traction spins or
  skids to the speed where motor torque = traction limit and draws that current. This lowered the
  example robot's peak drive current from 33 A to 17 A (it was overstated before; motion is
  unchanged because the force was already capped). Slip feeds static charging, which is how current
  draw reaches ESD.
- **Disconnects** (`Disconnects`, `MatchDriver`): one full-physics match gives bus voltage and current
  every 1 ms and every hit (wall rams with restitution, pushes with stalled motors, robot contacts).
  Each mode (Control Hub reboot, Expansion Hub drop, I2C lockup, USB camera drop, Wi-Fi drop) has
  lognormal fragility in discharge energy, impact g, current rise/drop rate; a dip under its reset
  voltage for the hold time always resets it; motor noise ~ I^2 and random rates add Poisson events.
  Exact P(at least one) per match plus a sampled timeline with outages.
- Real numbers used as examples: the REV Resistive Grounding Strap is 470 ohm (REV-31-1269 product
  page); REV specifies 8 V minimum operating input for the hubs (actual reset point unpublished).
  FIRST's ESD guide: sliding wheels charge the frame faster, wheels end up positive and tiles negative,
  ESD on the Control Hub's USB 2.0 port can drop Wi-Fi, I2C ports can be hit through sensor cables.
  Every other ESD/disconnect coefficient is a placeholder until measured (field meter + disconnect
  log, `profile import-disconnects`).
- Profile schema is now version 2 (`disconnect_observations`); older files upgrade on open.
- The `util` package was renamed `utility` (in Android Studio, before this work).

## 2026-10-23 — Sim module: robot physics sims + robot profile database

**Decisions (asked the user):** `Sim` is now a plain JVM desktop module (was an empty Android
app). The game-sim data file is SQLite. The game engine is undecided, so the profile stays
engine-neutral, with JSON/CSV export. The drivetrain is mecanum.

- Build: `Sim/build.gradle.kts` uses `org.jetbrains.kotlin.jvm` (2.4.10, declared in the root
  `build.gradle` next to the Android Kotlin plugin) + `application`, Java/Kotlin target 8 (was 17;
  see the Java 8 note below),
  `org.xerial:sqlite-jdbc:3.50.3.0`. `run` and `test` pass `--enable-native-access=ALL-UNNAMED`
  because sqlite-jdbc loads a native library and JDK 24+ warns otherwise.
- The game is **BIOBUZZ** (FTC 2026-27): POLLEN (2.8 in) and NECTAR (3.6 in) balls are launched into
  the upward-facing HIVE CELL (20 x 14 in opening, 12 in deep). FLOWERS (4 in top opening, 21.5 in up)
  are scored by placing. These numbers are pre-filled in the profile as `game_manual`. Not in the manual
  text, so measure them: the CELL opening's height and tilt, ball masses, and how many balls tip a HIVE.
- **Profile DB** (`Sim/data/robot_profile.db`, tracked): `Catalog.java` lists every entry (181 at first, 359 since the undeterminables)
  (parameters, curves, 2-D surfaces, error distributions, timed actions) plus observation tables
  for real shots and pose checks. It started with only the manual values filled. Each value records
  its source. Priority is measured = game_manual > manual > sim > placeholder, enforced in
  `ProfileDb`. Sim output computed from any placeholder input is saved as placeholder, so example
  numbers can never pass for real data.
- **Physics:**
  - Motors use the linear DC model from datasheet points: R = V/I_stall, Kt = T_stall/(I_stall - I_free),
    Kv from free speed, friction from free current.
  - The battery is Voc - I*R, solved together with every motor load each step. That is what makes speed
    and acceleration fall when current draw sags the bus.
  - Mecanum uses the standard Jacobian, strafe efficiency, a traction cap of mu*N/sqrt(2) per wheel, and
    rolling resistance.
  - The flywheel uses a feedforward + P controller, and each shot takes energy from the wheel.
  - Balls fly by RK4 with drag and Magnus. Scoring openings (`Aperture`) can face any way.
- **Predictions:**
  - `AimSolver` picks hood + speed by maximizing the estimated make chance under the profile's speed and
    angle error sizes. Choosing on speed tolerance alone flipped to flat shots that were very sensitive to
    angle error.
  - `AimSolver.solveTable` builds the whole shooting table at once (dynamic programming over 1 deg hood
    steps, penalising hood jumps) so neighbouring distances use the same arc. Where the best arc really
    switches (> 6 deg), it adds two points 1 mm apart so interpolation never blends a lob with a flat shot.
  - `ShotPredictor` is Monte Carlo over every error distribution, including localization error.
  - `DriveTimePredictor` uses bang-bang moves (full power, then full reverse) with the switch point found
    by bisection.
- **Verified in the container** (javac + the real sqlite-jdbc jar; Gradle cannot run there):
  - 32 JUnit tests pass. They cover the motor datasheet points, vacuum range = v^2/g, mecanum top
    speed/strafe/traction limit, the shot energy balance, DB priorities, imports and export, the
    shooting table's arc consistency, and stale-result clearing.
  - `sim all` with example inputs gives sane numbers: 1.80 m/s top speed vs 1.70 m/s free speed at
    12 V (the example battery is 13.2 V); traction-limited 5.85 m/s^2.
  - The HardwareMonitor import was run on real monitor session files.
  - **Not yet built with Gradle on the PC or fed real robot data.**
- **Model limits** (also in `Sim/README.md`):
  - Wheels never slip, so current while traction-limited is overstated.
  - Bang-bang braking by reversing power draws about twice stall current per motor. Real paths are
    slower; scale by `drive.path_overhead_factor`.
  - Exit speed ratio and energy loss factor must be measured (slow-motion video).
  - Bounce-outs are not modelled.
  - `TeamCode/Sim/` (Adapters/Interfaces) is still empty. Running real OpMode code against the sim is a
    later step.
- **Review fixes (independent review of the first version):**
  - Real shot/pose logs now write `*.measured_*` curves instead of overwriting the sim's curves; both are
    kept so a game sim can prefer real data where it has enough of it.
  - A ball coming up through the underside of the CELL no longer counts as a make.
  - A sim result that can't be produced (NaN, no points) is saved as empty, so stale numbers from an
    older run never survive next to new ones.
  - Make-probability curves run 0.5 m to the field diagonal, with unreachable distances stored as 0.
    `max_range_m` is found by bisection between grid points.
  - Heading error from the localization sim is saved (`localization.heading_error_deg`) and used as aim
    error in the shot Monte Carlo.
  - Catalog sync only runs when `Catalog.java` changes (fingerprint in `meta`); SQLite busy timeout 5 s.
  - `profile clear` works for curves, surfaces, distributions and actions too; `predict shot --battery V`
    also prints the burst make rate on that battery.
- **Java 8 (user request):** Android Studio showed hundreds of errors in Sim because the first
  version used Java 9-17 features. Sim is now plain Java 8 like TeamCode: records became final
  classes (same accessor names, `equals`/`hashCode`/`toString` kept, so the catalog fingerprint is
  unchanged), arrow switches became classic ones, text blocks became concatenated strings, and
  `List.of`/`Path.of`/`Files.readString`/`Stream.toList()`/`HexFormat` were swapped for Java 8 APIs.
  `Rng.gaussian()` now uses the Marsaglia polar method (`SplittableRandom.nextGaussian` is Java 17+),
  so Monte Carlo numbers differ slightly from before for the same seed. Checked with
  `javac --release 8`; the help text, DB schema, catalog and PROFILE.md came out identical.
- Removed the Android template leftovers from `Sim/` (`AndroidManifest.xml`, `res/`, `androidTest/`,
  `proguard-rules.pro`, `ExampleUnitTest.java`); they went to the Windows Recycle Bin.
- `./gradlew :Sim:build` passed on the PC (32 tests).
- Sim performance: about 20 s for `sim all` on 2 cores. The aim solver dominates (about 6000 flights
  per distance); Monte Carlo runs in parallel over distances with per-point seeds, so results don't
  depend on the thread count.

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

- **FTC Dashboard** is now `com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0` from
  the Dairy repo. A stock `dashboard:0.5.1` had been added to `TeamCode/build.gradle`;
  with Sloth on the classpath its Enable/Disable OpMode was missing and `@Config` edits
  had no effect, because Sloth runs TeamCode from its own classloader. The slothboard
  version prefix must equal the Sloth version (`tuning` 1.0.1 pulls Sloth 0.3.2).
  *(historical)* It was originally `com.acmerobotics.dashboard:dashboard:0.6.0`, from
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
