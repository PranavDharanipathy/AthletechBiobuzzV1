# Standing Goal: Shooter & Turret Control (Team 31666 Athletech, BIOBUZZ 2026-27)

This file is the **continuous goal prompt** for every session in this repo. Re-read it at the start of each session, together with `DEV_NOTES.md` (robot specs, CAD data, decision log).

## Mission

Build shooter and turret software that is **bulletproof, precise and fast**. The robot is mechanically simple on purpose, so the advantage has to come from the code. Every shot should land from any legal position, while the robot is moving, at any battery voltage, from the first match of the day to the last.

## Hard constraints (from the team; do not violate)

1. **No PID controllers.** That means no P/I/D loop on error, in any subsystem covered here. Find alternatives that are better suited to the physics.
   - Some candidates also contain an error-proportional term, for example LQR state feedback, ADRC's feedback gain, or sliding-mode surfaces. Never slip these in quietly. Name it, justify it in the decision log, and **ask the user** before adopting it.
2. **No bang-bang on the flywheel.**
3. **Voltage must not matter.** Performance has to hold from about 14.0 V down to sag under load (≤ 11 V). Compensate explicitly and verify it.
4. **No new third-party libraries for now.** Use the FTC SDK only. Write lean, optimized code ourselves. Propose a library only with strong justification.
5. **Scope is shooter + turret (+ hood + aiming)** until the user widens it.
6. **The CAD is read-only.** Never create, edit, move, rename or delete anything in Onshape.

## Method: a scientific engineering process

Every design decision goes through these steps, in order:

1. **Requirements.** State measurable targets with numbers. Examples: flywheel recovery time after a shot, steady-state speed error, turret settle time, tracking error while the robot spins at X°/s, shot-to-shot dispersion, loop time. If a target is unknown, propose one and mark it ❓ for the user.
2. **Physics model.** Derive the plant from first principles: motor constants (Kv, Kt, R), inertia, friction (static, Coulomb, viscous), gear ratios, backlash, servo internals, sensor resolution and latency, and ball-contact energy loss. Write the equations down.
3. **Alternatives.** List **at least 3** viable approaches. Include at least one unconventional, "outside the box" idea. Debate them honestly: performance, robustness to model error, tuning effort, CPU/IO cost, failure modes and what happens at the edges.
4. **Decision record.** Add an entry to the Decision Log in `DEV_NOTES.md`: context, options, the choice, why the others lost, and what evidence would make us revisit it.
5. **System identification plan.** Write a tuning or characterization OpMode that measures the model parameters on the real robot and logs the raw data. Never guess a constant that can be measured.
6. **Implement.** Keep the pure-Java math (controllers, observers, models, kinematics) **separate from the hardware calls**, so it can be unit-tested and simulated off-robot.
7. **Verify.** Unit tests plus a simple plant simulation, including noise, quantization, voltage sag, delay and disturbances. State the expected on-robot results before testing and compare them afterwards.
8. **Iterate.** Update `DEV_NOTES.md` with what was learned.

When something is uncertain, **don't guess.** Check the CAD, check the code or datasheets, or ask the user. Mark every assumption as an assumption.

## Things to consider at minimum (check off or justify skipping)

### Flywheel (1× goBILDA 6000 RPM bare motor, 1:1 belt, 72 mm Rhino + 2× SWYFT)
- Control alternatives to weigh, including:
  - model-based feedforward (kS + kV·ω + kA·α) with voltage scaling
  - feedforward + disturbance observer / ESO
  - ADRC
  - model predictive control (MPC; cheap for a first-order plant)
  - online-adaptive feedforward (RLS parameter estimation)
  - a minimum-time spin-up profile within current limits
  - shot-anticipation feedforward: we *know* when a ball is fed, so we can pre-inject the expected energy loss
- **Velocity sensing:** 28 CPR at ~2800 tps is coarse. Consider hub velocity vs. position differencing, quantization, latency, filtering (Kalman / α-β), and how sensing interacts with loop time.
- **Voltage:** read the battery voltage sensor sparingly (it costs I/O) and filter it. Model sag under load, brownout risk and current limits.
- **Readiness:** "ready to shoot" criteria based on predicted velocity at the time the ball makes contact, not just current velocity.
- Motor heating and drift in kV over a match. Behaviour across different batteries.

### Turret (Axon Max servos → 24T pinion → 72T gear, 3:1; REV absolute encoder; axis 2.82 in behind drive center)
- **Servo mode decision:** use the Axon internal position control or continuous-rotation mode under our own control. At 3:1, positional range limits turret travel, so analyze this.
- Control alternatives without PID: motion profiling (trapezoidal / S-curve / jerk-limited) + feedforward (kS, kV, kA), model-based inversion, ADRC, MPC, sliding mode (ask the user first), time-optimal-ish profiles.
- **Robot-rotation feedforward:** counter-rotate using the gyro/odometry yaw rate so the turret holds field heading even when the robot spins.
- **Backlash:** the two pinions on one ring gear could be **preloaded against each other** to eliminate backlash. Evaluate this.
- **Servo deadband**, PWM resolution, servo speed and torque limits, and differences between the two servos.
- **Encoder:** confirm the model, how it's mounted (geared or direct) and how it's read (quadrature vs. absolute / analog / PWM). Use the absolute reading for homing at startup, and handle wrap-around and cable or travel limits.
- Sensor fusion: Axon analog feedback wire + external encoder.

### Hood (2× Axon Max, linkage)
- Map servo position → hood angle using linkage kinematics from the CAD. Account for servo travel time and settling, keep the two mirrored servos in sync, and allow for backlash.

### Aiming & ballistics
- Choose between a physics projectile model (with drag) plus empirical correction, and a pure lookup table. Debate both.
- **Shoot on the move:** robot velocity, time of flight, lead compensation and total latency (sensor → compute → actuation → ball exit).
- Distinguish the turret pose from the robot pose (offset). Account for the exit point and height.
- Look up BIOBUZZ game rules and goal geometry in the game manual. **Do not invent field dimensions.**

### System / software
- Loop time and its jitter. LynxModule bulk caching (MANUAL). Minimize hub I/O: skip redundant writes, cache reads.
- No allocations in the loop (GC pauses). Use `System.nanoTime()`. All math is dt-aware.
- Fail safes: sensor disconnect, NaN/Inf guards, encoder sanity checks, stall detection, graceful degradation.
- Telemetry and CSV logging designed for system identification and post-match analysis.
- Testability: pure-Java core + JUnit + plant simulator.

## CAD access (when unsure about anything mechanical)

- Onshape document: `https://cad.onshape.com/documents/231d0d2ccd57c7da2e3d018d/w/26c896210c8c2048c69eab92/e/73ff5307c757b8d38dede3bf`
- Method: Claude in Chrome (the user's logged-in session) → **GET-only** requests to `/api/v6/...` via the page's fetch, plus viewing and screenshots. Never use POST/DELETE, never click into features or sketches, never drag parts.
- Useful element ids: Main assembly `73ff5307c757b8d38dede3bf`, Shooter Assy `41e5bcf280796d53e5626342`, Shooter Parts `e9e655e25cde5c707fd8f97e`, Turret Servo Shaft `8f757bd88f3c77601d6e2295`.
- CAD frame → robot frame: CAD −Y = forward, CAD +X = left, CAD +Z = up. The floor is at CAD z ≈ −0.37 in.
- If the Chrome extension isn't connected, ask the user. Don't guess.

## Current state (keep updated)

- v1 implemented on 2026-10-21 (see `DEV_NOTES.md` changelog). Code: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/shooter/` — `core/` (pure Java), `hardware/` (SDK I/O, `ShooterConfig` = all constants), `opmode/` (characterization, calibration, tuning).
- Verify every change with `tools/run_tests.sh` (all checks must pass) and `./gradlew :TeamCode:assembleDebug`.
- Next real-robot steps: `docs/SHOOTER_DESIGN.md` §5 (calibration order) and §9 (open ❓).

## Reference

- Last season's code is in `../AthletechPremiers2526`. Mine it for ideas (its flywheel feedforward, Kalman velocity filter, friction model, turret routing, latency/pose prediction), but it used PID. **Do not copy its control approach.**

## Working style

- Debate thoroughly, then commit to a recommendation. Present trade-offs briefly to the user and ask when a choice is genuinely theirs.
- Don't commit or push to git unless asked.
- Keep `DEV_NOTES.md` current: specs, ❓ items, decision log and changelog.
