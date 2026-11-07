# System evolution: hypertuning that changes the control system, not just its numbers

Hypertuning OpModes (the first is **SHT Stability Tuner**) tune gains while the robot plays. Numbers only get you so far:
when a knob keeps winning at its bound, or no knob moves the cost at all, the **structure** of the controller is the limit.
This file is how that evidence becomes a new **principle**: a feature added to the control system, or one replaced or
removed. An AI agent (Claude Code, Codex) does that work.

Every hypertuning session can hand the agent a new principle. The agent never acts on a hunch: every change starts from
evidence in the logs, is tried in the sim, and is switchable. The robot then decides whether it stays.

## The loop

1. **Play.** Run a hypertuning OpMode like a match. SHT Stability Tuner is the full RED TeleOp. It writes
   `/sdcard/FIRST/data/ShooterHyperTuning/<time>/`, as described in "What a session leaves" below.
2. **Pull.** Plug in and build. `:TeamCode:pullTuningData` copies new sessions to `tuning-data/hypertuning/<time>/`. Commit
   them: they are the evidence.
3. **Read.** The agent reads the newest `findings.md`, then the windows behind it in `loop.csv` and `events.csv`.
4. **Write the principle** in the register below, with status `proposed`. Write it as one sentence on the mechanism, the
   evidence (session, numbers), and what it predicts will improve.
5. **Build it switchable.**
   - Put it behind a constant (`TurretConstants`, `FlywheelConstants`, `DrivetrainConstants`) defaulting to *off*, or to a
     value that changes nothing.
   - Keep the logic in a pure class the Sim can run.
   - Add a code-in-the-loop test in `Sim/src/test` that reproduces the evidence, and show the principle fixes it there.
   - Status: `trial`.
6. **Give the hypertuner a knob for it.** That's an `OnlineTuner.Knob`: continuous, or `0..1` with step 1 for an on/off A/B.
   If the new principle is a cost, add it to the cost too. Add a `Findings` hint for its knob.
7. **Play again.** The tuner keeps the principle only if it scores better on the robot. Two or more sessions where it's
   kept: status `adopted`, and make it the default. It loses, or its knob never moves the cost: status `removed`. Delete the
   code, the knob and the constant, and keep the register line as a record.

## Rules for agents

- **Evidence first.** Quote the session and the numbers. "It should be better" isn't evidence. A sim run that reproduces
  the robot's symptom is.
- **The sim isn't the robot.**
  - The landing brake lost in the sim because reverse braking was never measured. That's why it goes to the robot as an
    A/B instead of being dropped or forced on.
  - Never tune sim parameters to make a principle win.
- **Match defaults change only with evidence from at least two sessions.** Shooter logic, tuning and constants are the
  user's: changing a default in `FlywheelConstants` or `ShooterConstants` needs the user's OK. New knobs default to "changes
  nothing".
- **Repo rules still hold** (AGENTS.md):
  - Java 8;
  - no per-loop allocation in hot paths;
  - at most one low-priority I/O thread (`CsvLogger`);
  - no hardware calls off the loop thread;
  - no `java.nio.file` (the Control Hub's Android doesn't have it);
  - TeamCode only, never FtcRobotController.
- **Hypertuners stay safe.**
  - Every knob has hard bounds.
  - A trial only keeps a change that beats the baseline by `OnlineTuner.margin`, averaged over `samplesPerTrial` samples.
  - The baseline is measured again every round.
  - Whatever a tuning OpMode changes in shared static constants, it puts back in `stop()`.

## What a session leaves

| File | What |
|---|---|
| `loop.csv` | Every loop. Flywheel: target, speed, power, boost, feeding, balls seen. Turret: target, position, speed, volts, spin rate it was given, landing. Robot: omega, pose, drive hold. The tuners' last costs, then every knob's value. |
| `events.csv` | Every finished trial: `t`, `tuner` (0 flywheel stability, 1 flywheel shots, 2 turret moves), `knob` (column index into the knob list, in `gains.csv` order), `from`, `to`, `cost`, `baseline_kept` (the baseline, negative when the change was not kept). |
| `gains.csv` | One row each time a change is kept: every knob, plus `rate_scale`. The last row is where the next session starts. |
| `findings.md` | Each tuner's cost before and after, a knob table, and **principle candidates**: knobs pinned at a bound, tuners whose knobs can't move their cost, and the spin-cancel scale pinned. |

## The costs (what "better" means)

- **Flywheel stability.** At speed, not feeding for 0.6 s, inside ±40 ticks/s, scored over 1.5 s windows: RMS error +
  0.5 × RMS loop-to-loop jitter. Knobs: `kp_close`, `kd`.
- **Flywheel shots.** Per burst, from the first feed until it's back inside ±30 for 0.15 s:
  - |mean speed error at contact| + spread of it, which is what changes the shot;
  - plus 0.5 × mean dip below target;
  - plus 300 × recovery seconds.

  Knobs: `feed_boost_volts`, `shot_kick_volts`, `shot_kick_time`, `kp_far`, `max_p`.
- **Turret moves.** Per aim jump of 15° or more with the robot under 30°/s: seconds to within 1° + 0.03 × overshoot° +
  0.1 × volt flips mid-rotation (above 60°/s, more than 20° out) + 0.01 × end shake (°/s RMS). Knobs: `bandwidth`,
  `brake_degrees`, `max_speed_fraction`, `acceleration_fraction`, `resync_degrees`, `landing` (A/B).
- **Spin cancelling.** This is learned, not trialled. While the robot spins faster than 60°/s with a real aim, a turret
  that lags the spin raises `TURRET_RATE_FEEDFORWARD_SCALE`, and one that leads it lowers it (bounds 0.7..1.3). It settles
  where it meets the spin exactly.

## Adding to the hypertuner

- **A knob:** add an `OnlineTuner.Knob(name, start, min, max, step, minStep)` to the right tuner. Write it into the
  controller in that tuner's `apply()`. Restore it in `SHTStabilityTuner.stop()` if it's a shared static. Add a `hint`
  case in `Findings`.
- **A cost term:** add it where the tuner scores (`FlywheelHyperTuner.update`, `TurretHyperTuner.update`). Write down the
  units and weight in "The costs" above.
- **A new hypertuning OpMode** (drive, intake, auto paths): reuse `OnlineTuner` and `Findings`. Write its own folder under
  `/sdcard/FIRST/data/`, and add a pull set in `TeamCode/build.gradle`.

## Principles register

| # | Principle | Status | Evidence |
|---|---|---|---|
| 1 | **Pose-holding drive**: the sticks move a reference, and the robot is held to it in all 3 axes with a feedforward + PID from an identified model; speed only while slowing; traction-limited reference; leash; relocalization jumps carried over. | adopted (sim) | Sim, 60 N shoves: 17.8 in → 1–2 in, slow drive pushed 18 → 40 in/s → held at 20, twist 16.6° → 2.5°, stop as short as before. `DriveCodeTest`. |
| 2 | **Turret plan resync**: more than `TURRET_RESYNC_DEGREES` off its own plan, the plan moves onto the turret instead of the two fighting to 0 V. | adopted | TeleOp 20280507-230429: 31% of loops 15°+ off the plan, 2 s stuck 30° short. Sim: 6–8% → 0%. `turretNeverFightsItsOwnPlan`. |
| 3 | **Brake fade**: cruising, the friction push fades to reverse over `TURRET_BRAKE_DEGREES` instead of flipping at 1° (stopping still flips). | adopted | Sim: speed chopped 100–320°/s → smooth 384°/s cruise. |
| 4 | **Landing brake**: predict the stop with the model (dead time, lag, coasting, friction, reverse braking) and bisect the braking volts that land on the aim; learn the rest offset. | trial (off, `landing` A/B knob) | Sim: ~10% sooner to 1°, but 3–5° overshoot against 1–2°. Reverse braking authority was never measured. |
| 5 | **Shot power**: feed boost while the blocker is open, and a kick after a detected ball hit. | trial (0 V, tuned by SHT) | Log: a ball drops 22–78 ticks/s in one loop. Sim: 0.5 V boost tightened exit-speed spread 0.008 → 0.003 m/s, and 1–2 V overspeeds. |
| 6 | **Spin-cancel scale**: learned share of the robot's spin fed forward to the turret. | trial (learned, starts at 1) | Request: "meet it, don't overdo it". `spinShareConvergesToMeetingTheSpin`. |

### Not built yet (candidates waiting for evidence)

- **Shot load profile (iterative learning control).** Learn each ball's load as volts vs time from the blocker opening, and
  feed it forward. Trigger: `feed_boost_volts` pinned at its top.
- **Reverse-braking stage in the Turret Autotuner.** Measure deceleration under reverse volts, so landing's model is right.
  Trigger: `landing` loses, or `brakeScale` saturates.
- **Notch at the flywheel frequency in the turret error.** Trigger: turret `bandwidth` pinned at its bottom while the
  flywheel runs.
- **Spin lead in the aim.** Trigger: spin-cancel scale pinned at 1.3.
- **Drive hypertuner.** Tune the hold bandwidth and leash in matches, scored on pushes survived and driver stick-to-motion
  error.
