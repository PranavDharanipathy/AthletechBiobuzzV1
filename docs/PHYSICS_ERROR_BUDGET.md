# Shooter physics audit and error budget

Written 2026-10-31 (Claude), from a read-only audit of the repo at `e9ef9b9`. No robot was run.
The numbers come from `python3 tools/shot_error_budget.py` (standard library only), so rerun it
once real measurements replace the assumptions.

## Status: Phase 0 code fixes applied (2026-10-31, not yet run on the robot)

| Finding | Change | Test |
|---|---|---|
| B: flywheel units | `FlywheelConstants`: every power-unit gain and clamp × 12 V, so `toPower(volts)` gets volts. Same behaviour at 12 V as the numbers were tuned for; battery-compensated otherwise. **Pranav: confirm.** | `FlywheelPhysicsTest.productionFeedforwardIsInVolts` |
| C: timestamps | `Flywheel` and `TurretBase` read the encoder, stamp the time, and only then do any battery read. New `update(long sampleNanos)` overloads. `TeleOpBaseOpMode.clearCacheOfLynxModule()` now calls `getBulkData()` on each hub after `HardwareMonitor.update()`, records `getBulkReadNanos()`, and `HoodInterpolationTuner` passes it. | `encoderIsReadBeforeTheBatteryRead`, `suppliedSampleTimeGivesExactVelocity` |
| D1/D2: speed gate | `Flywheel.getWindowVelocity()` (100 ms Δcounts/Δt) and `isReady()` (within `FLYWHEEL_READY_FRACTION` = 1 %). The old `isAtVelocityAndStable` is unchanged. `FlywheelTuner` shows both. | `windowVelocity*`, `reverseRotationIsNeverReady` |
| I.1: servo battery scaling | `TurretBase` divides by `TURRET_NOMINAL_VOLTS` instead of the battery voltage, and no longer reads the battery. `TurretConstants.TURRET_BATTERY_COMPENSATION = true` restores the old behaviour if the diagnostic shows a battery dependence. | `servoCommandDoesNotDependOnBatteryVoltage` |
| I.2: velocity overflow | `Encoder.correctVelocityOverflow` / `getInternalVelocity(estimate)`; `TurretBase.getVelocity()` uses it. The Turret Diagnostic still records the raw hub value on purpose. | `hubVelocityWrapIsCorrected` |
| I.3/I.4: hood | `HoodAngler` per-servo `LEFT/RIGHT_HOOD_TRIM` (`HoodConstants` is now `@Config`), plus an injection constructor. `ShooterConstants`' unused hood pair is marked as last season's. | `HoodAnglerTest` |
| H (Sim) | Profile: `launcher.exit_height_m` 0.292, `offset_forward_m` −0.0518, `offset_left_m` −0.0152, `has_turret` 1 (CAD, `manual`). | Sim tests |

**Still open:**
- Waiting on measurement or a decision:
  - A: target geometry.
  - E: aero.
  - F: turret ticks/°.
  - G: turret feedforward control, which needs K and T from the diagnostic.
  - Per-feed recovery and energy banking: no feed code exists in `main` yet.
- Exit point vs hood angle in the v1 `shooter/` package (reconcile later).
- The Sim's one fixed exit height.

**How to read it:**
- Each item says what the code **models**, what is **real** (physics, CAD, or unknown), and how
  big the gap is in inches at the target.
- Every number carries a confidence tag:

| Tag | Meaning |
|---|---|
| **[physics]** | Follows from first principles. |
| **[CAD]** | From the 10-31 CAD read in DEV_NOTES. |
| **[est]** | Engineering estimate; must be measured. |
| **[unknown]** | Nobody has measured it. |

`TeamCode/...` means `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

---

## 1. The miss window (what "error" is measured against)

Target opening, from `tools/pollen_shot_envelope.py` and `docs/SHOOTER_DESIGN.md`:
- 20 in wide × 14 in tall, centre 59.55 in above the tiles.
- The edges at 53.5 and 65.6 in make the opening plane **≈60° from horizontal**, so its normal
  points ~30° above horizontal, toward the shooter. 12.1 in = 14 in × sin 60°.
- Ball radius 1.4 in.

The usable window is:

| Direction | Window | Notes |
|---|---|---|
| Lateral | ±(10 − 1.4) = **±8.6 in** | Frontal shot. A shot from 45° off the cell's facing direction gets ±5.7 in, because the opening is foreshortened. |
| In-plane height | **±5.0–5.6 in** | 7 in − 1.4 in / cos(incidence). Best when the ball arrives perpendicular to the opening, i.e. descending at ~30°. |

Turret angle allowed for the lateral window: atan(8.6 in / d) = 6.2° at 2 m, 4.2° at 3 m, 3.6° at 3.5 m.

## 2. Sensitivities

Each column below is the in-plane miss, in inches, at the target. Settings:
- 30 g ball, Cd 0.5, Cl 0.
- Hood angle chosen for the largest window.
- Distance is horizontal, measured from the turret axis.

| d (m) | hood | exit v (m/s) | rpm @ η = 0.5 | ToF (s) | 1 % speed | 1° hood | 0.5 in exit height | 1 in distance | Cd +0.15 | Cl +0.2 | mass −30 % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | 54 | 6.40 | 3394 | 0.41 | 0.86 | 1.80 | 0.67 | −0.35 | −0.79 | 1.74 | −1.12 |
| 2.0 | 54 | 6.35 | 3370 | 0.56 | 1.22 | 0.97 | 0.52 | 0.19 | −1.43 | 1.45 | −2.04 |
| 2.5 | 54 | 6.64 | 3524 | 0.68 | 1.55 | 0.49 | 0.46 | 0.41 | −2.24 | 1.39 | −3.19 |
| 3.0 | 53 | 7.03 | 3731 | 0.76 | **1.82** | 0.30 | 0.43 | 0.50 | −3.09 | 1.60 | −4.39 |
| 3.5 | 50 | 7.47 | 3964 | 0.79 | **1.96** | 0.58 | 0.43 | 0.50 | −3.72 | 2.40 | −5.29 |

**Takeaways:**
1. **Exit speed is the dominant vertical error.** At 3 m, 1 % of speed costs 1.8 in, so the whole
   5.6 in window is only ±3 % of speed. For a ~95 % vertical make rate, the total speed error has to
   stay near 1.5 % (1σ).
2. **The hood angle is nearly free at long range.** The best arc sits where ∂(height)/∂θ ≈ 0. Short
   shots (1.5 m) are the exception: they are hood-sensitive.
3. **The aero model must be fitted to real shots before it predicts anything:**
   - Plausible errors in Cd, Cl or mass move the ball 1.5–5 in at 3 m.
   - The **vacuum model** in `PollenPhysics.vacuumSpeed` leaves the ball **4–13 in short** (2–3.5 m,
     30 g), because drag needs 4–6 % more speed.
   - That model is labelled "vacuum reference only", which is correct. It must never set RPM.

---

## 3. Findings: model vs real, ranked by inches at the target

### A. The target model disagrees between files [unknown, decision-critical]

| Where | Opening model |
|---|---|
| `tools/pollen_shot_envelope.py` | Pentagon, plane 60° from horizontal facing the shooter, centre 59.55 in |
| `shooter/core/HiveTarget` + `ShotTable` (v1) | Point at 59.5 in. Needs **descent ≥ 25°** (`ShotTable.java:25`). Cell offset 15.8 in from the pivot (`HiveTarget.java:22`); 18.84 in spacing would give 9.42 |
| `Sim` profile | Rectangle. `target.cell.opening_center_height_m` and `opening_tilt_deg` are **empty**; the catalog defaults are 1.25 m and 20°, both inconsistent with the above |

**Why it matters:**
- The 10-21 conclusion "the hood must reach 40–80°", and with it the team's "accept a longer
  minimum shot distance", comes from the descent ≥ 25° rule.
- If the opening really faces the shooter at 60°, a 54° hood still scores at 1.5 m: the ball enters
  while still rising, at 44° incidence, with a 5.1 in window.
- The descent rule then wrongly rejects the near field.
- Open questions the manual or a real field must answer:
  - Can a rising ball stay in a 12 in deep cell?
  - Does that count as a G417 "bottom" entry?

**Fix:**
- Get the opening centre (x, y, z), plane tilt, shape and facing direction from TU07 Fig. 9-10/11
  and a real field.
- Put one set of numbers in the Sim profile (`manual` source) and HiveTarget.
- Replace "descent ≥ 25°" with the real criterion: the projected aperture,
  14 in · cos(incidence) ≥ ball diameter + margin, inside a pentagon facing a known direction.

### B. Flywheel output is ~12× too weak (units) [physics, high confidence]

**Model:**
- `Flywheel.java:204` computes `power = voltageCompensator.toPower(p+i+d+v+s)`, and `toPower`
  divides by battery volts. So p, i, d, v and s must be in **volts**.
- The gains are in **power** units:
  - `kv = 0.00037` per tick/s, while a 6000 rpm, 28 CPR motor needs ≈ 1/2800 = 0.000357 *power*,
    or 12/2800 = 0.0043 *volts*.
  - Last season's `Flywheel` used `power = p+i+d+v+s` with `kv = 0.0003424`, tuned at 12.17 V.
  - The current gains are that season's scale.

**Real:** the feedforward supplies 1/12 of the voltage it should. The integral has to make up
~92 %, so settling is slow and every disturbance (each shot) recovers on the integral's time scale.

**Fix:** either multiply kp, ki, kd, kv and ks by the voltage they were tuned at (~12), or drop the
division. Then refit kV (and kS, kA) from a characterization run. Ask Pranav first in case he
retuned under `toPower`; the numbers say he did not.

### C. Velocity timestamps are not the sampling times [physics]

- In `Flywheel.update()`, the battery voltage read (`:151`, a full blocking hub transaction; voltage
  is not in bulk data) happens **before** the timestamp (`:157`). The encoder position, though, was
  latched at the bulk read earlier in the loop.
- Velocity = Δcounts / Δt_software, so any loop-to-loop change in (timestamp − bulk-read time) is a
  velocity error of v · δ / dt.
- With the 20 ms voltage period and loops shorter than 20 ms, the read lands in alternate loops.
  A 2 ms read then gives **±13 % (15 ms loop) / ±8 % (25 ms loop)** alternating velocity error.
- Any variable-length I2C read before the update (Pinpoint, color sensors) does the same.
- `TurretBase.update()` has the same structure (`:251`).

**Fix:**
- Stamp the time once, right after the bulk read (or `clearBulkCache` plus the first read), and
  pass it to every subsystem.
- Or use the hub's own velocity, which is timed on the hub.
- Read the battery after the control math, or on its own slot.

### D. Flywheel speed error: quantization, gate width, per-ball dip [physics + est]

1. **Quantization.** The 5202 encoder is 28 counts/rev. At 3700 rpm, a single-difference velocity
   has σ = 0.95 % (25 ms loop) to 1.6 % (15 ms).
   - The "stable" check `|v_k − v_{k−1}| ≤ 20` ticks/s (`FlywheelConstants:33`) has a pure-noise
     σ of 28–47 ticks/s, so it fails most loops at true steady state.
   - **Fix:** average over a fixed window (≥ 100 ms), or use the hub velocity. Gate on the mean.
2. **Gate width.** `FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 50` ticks/s is ±2.9 % at 3700 rpm, which is
   **±5.3 in at 3 m: the entire vertical window**.
   - **Fix:** gate at ≤ 1 % on the window-averaged speed, *and* compensate. Look up RPM/hood from
     the speed **measured at feed time**, not the target.
3. **Per-ball energy dip.** Flywheel J ≈ 2e-4 kg·m² [est, SHOOTER_DESIGN §3.1] holds
   ½Jω² = 15 J at 3700 rpm.
   - A rolling ball carries KE·(1 + I/mr²); hollow balls take 1.67×.
   - Ball mass vs speed drop per ball:

     | Ball mass | Speed drop per ball |
     |---|---|
     | 20 g | 2.7 % |
     | 30 g | 4.1 % |
     | 50 g | 7.0 % |

   - At 30 g, a second ball fed before recovery is **~7 in low at 3 m**.
   - **Bound:** at 12 V the motor has ~0.045 N·m spare at 3700 rpm, enough to recover a 4 % dip in
     **≈ 70 ms if saturated**. The current PIDVS, with finding B, will be several times slower.
   - **Fix:**
     - Fire each ball only once the speed is back inside the gate.
     - "Energy bank": raise the target by half the learned dip just before a feed (simulated in
       SHOOTER_DESIGN D-1).
     - Or add flywheel mass. The dip scales 1/J; doubling J halves it and costs spin-up time.
4. **Exit-speed ratio.** η = v_ball / v_surface = **exactly 0.5** for no slip at either contact,
   independent of the channel curvature [physics: the stationary hood makes the ball centre move at
   the average of the two surface speeds].
   - Real η < 0.5 because of slip and compression. The Sim default 0.5 is an upper bound.
   - **The irreducible floor is the ball-to-ball scatter in η** (diameter 2.80–2.90 in into a
     2.72 in gap means 2–4.6 mm squeeze). Sim assumes 2 %, which alone is 3.6 in 1σ at 3 m.
     **Measure it first** (Pollen Shot Experiment).

### E. Aero parameters are inconsistent across files [unknown]

| Parameter | v1 `Ballistics` | Sim default | envelope tool | SHOOTER_DESIGN |
|---|---|---|---|---|
| mass | 50 g | 20 g | 25 g (0.055 lb) | 50 g |
| Cd | 0.50 | 0.5 | 0.45 (bounds 0.3–0.6) | 0.5 |
| Cl | 0.12 | 0 | 0 (bounds 0–0.2) | 0.1–0.25 |

**Real:**
- The drag deceleration scales with ρCdA/(2m). A 2.5× mass disagreement is a 2.5× drag disagreement.
- The spin is set by geometry. With no slip, the ball rolls on the stationary hood with **backspin
  at spin ratio S = ωr/v = 1** [physics].
  - Smooth-sphere data at S ≈ 1 give Cl ≈ 0.3–0.4.
  - The 0–0.2 bounds likely **under**-predict lift; slip lowers it.
- The Reynolds number is ≈ 3e4 (sub-critical), so constant Cd is reasonable. If POLLEN is a
  perforated ball, both coefficients change; check the real ball.

**Fix:**
- Weigh 10 balls and caliper them.
- Fit k_d = ρCdA/(2m) and Cl from measured shots at ≥ 3 distances × 2 hood angles.
- Fix η from slow-motion video (≥ 240 fps) at the same time.
- Fit one parameter set and use it in all three places.

### F. Turret angle scale [CAD vs code]

- `ShooterConstants.TURRET_TICKS_PER_DEGREE = 65.738` (last season). The CAD gives 8192·3/360 =
  68.267, a **3.7 %** scale error.
- At a 90° turret angle that is 3.3°, so **6.9 in lateral at 3 m** (80 % of the window).
- `TurretBase`'s hold-decay also uses 65.738 (`:302`).

**Fix:** measure it with the Turret Diagnostic passive mode (step 1) before changing it.

### G. Turret tracking while the robot moves [physics + est]

**Plant:**
- An Axon CR servo is close to a velocity source: speed ≈ K·u, with a deadband, so θ/u ≈ K/s
  (an integrator).
- The effective delay T is about 60 ms [est]: ½ loop of sample-and-hold, plus up to one loop from
  sense to write, plus the servo PWM frame, plus the servo internals, for a 30 ms loop.

**Best feedback-only design:**
- For 45° phase margin, crossover ω_c ≤ π/(4T) ≈ 13 rad/s.
- The tracking lag is then ω_robot / ω_c:

| Robot yaw rate | Turret lag | Miss at 3 m |
|---|---|---|
| 90 °/s | **6.9°** | 14 in |
| 180 °/s | 13.8° | 28 in |

**Uncompensated latency:** a further ω·T, which is **5.4° at 90 °/s**.

**Why it shakes and overshoots:**
1. `TurretBase` adds an **integral** to an integrating plant. That makes the loop type-2, which must
   overshoot to unwind, and the delay erodes the phase margin.
2. `kISmash` rescales the integral on every sign change, a nonlinear reset.
3. The `kHold` term is a smoothed **relay** (`f = kHold·sign(e)·(1 − e^{−|e|/w})`, `:303`).
   - Describing-function prediction: a relay in an integrator + delay loop limit-cycles at
     ω ≈ π/(2T) ≈ 4 Hz.
   - Its amplitude is ≈ 4·kHold·K/(π·ω). Example: kHold = 0.05, K ≈ 430 °/s → **~±1°**, about
     ±2 in at 3 m, unless the smoothing width w exceeds that amplitude.
   - This is a testable prediction: run the diagnostic's "hold reference" plan and take an FFT of
     the position.

**Fix:** feedforward the motion instead of integrating it:
- u = (θ̇_target − ω_robot)/K + kp·e (+ kd).
- Use the target angle **predicted at t + T** (v1 `AimSolver` already does this with
  `latency = 0.06`).
- Keep the integral tiny or off.
- Size kHold from the measured breakaway command, with w ≥ the measured limit-cycle amplitude.
- With a 10 % feedforward gain error, the lag at 90 °/s drops from 6.9° to ~0.7°.

### H. Launch kinematics and geometry [CAD + physics]

| Item | Model | Real | Error |
|---|---|---|---|
| Exit height | v1 `AimSolver` 12.4 in constant (`:27`); Sim one constant | 9.30 + 2.82·cos θ = 11.72 (31°) … 10.96 (54°) in [CAD] | 0.7–1.4 in height, so ≈ 0.6–1.2 in at the target |
| Exit fore-aft | distance from the exit point | exit is 2.57 − 2.82·sin θ in ahead of the turret axis (1.1 → 0.3 in) | ≤ 0.5 in |
| Turret axis vs drive centre | must aim from the turret axis | 2.82 in behind centre | up to 2.8 in lateral if bearing is computed from the robot centre |
| Exit lateral offset | 0 | ≈ 0.6 in right of the axis [CAD] | 0.6 in lateral, constant with distance |
| Lever arm ω × r | Sim `Launcher.launchVelocity` (`:179`) has none (the Sim fires only from rest today) | r ≈ 2 in. At 3 rad/s that is 0.15 m/s sideways | **≈ 4.7 in** at 3 m (ToF 0.76 s) |
| Robot velocity | v1 leads with ToF; `main` has no aim code yet | inherited fully | 1 m/s × 0.76 s = **30 in** if not led |
| Heading error | – | 1° | 2.1 in at 3 m |

### I. Smaller items

1. **Battery compensation on CR servos.** `TurretBase:306` divides by battery volts, but servo ports
   run off a regulated rail [est: hub servo rail]. This makes loop gain vary ±8–9 % across
   11–13 V with no physical basis. Remove it, or show the effect is real with the diagnostic: run
   open-loop `late_speed` at two battery levels.
2. **Turret hub velocity overflow.** The hub reports velocity as a 16-bit signed count rate, so it
   wraps above 32767 counts/s, which is **480 °/s of turret** at 68.27 ticks/°. The Axon Max at 1:1
   can approach that. `Encoder.getInternalVelocity()` has no overflow correction (Road Runner's
   `Encoder` does). Position is 32-bit and fine.
3. **Mirrored hood servos.** They are rigidly coupled. Any neutral mismatch between them makes them
   fight: current draw, heat, and a hood angle that depends on which servo is stronger. A 25T spline
   tooth is 14.4°.
   - `HoodAngler` sends both the identical position. Add a per-servo trim, or set centres in the
     Axon programmer, and zero the fight by watching the current.
   - Good news [physics]: the hood arc is concentric with its pivot, so the ball's squeeze force is
     radial and puts **no moment** on the hood. Only friction does, so the hood should not be
     back-driven during a shot.
4. **Two hood limit constants.** `HoodConstants` (1 … 0) and `ShooterConstants` (0.78 … 0) both
   define the range. The four-bar toggles at ~60° launch, so keep one limit, set from the measured
   31–54° servo endpoints.
5. **Stale tables.** `HOOD_DISTANCES` / `FLYWHEEL_VELOCITIES` are last season's robot and target;
   `HOOD_POSITIONS` are all 0.
6. **Constants that are fine.** Air density (1.20 vs 1.204), g, and RK4 steps (≤ 2e-4 m) are all
   negligible at ≤ 0.3 in. Altitude, though, changes ρ by up to −17 % (Denver).

---

## 4. Budget at 3 m: now vs after the plan (1σ, inches)

The projections assume the σ values shown; the measurements in §5 replace them.

| Source | Now | After | Basis |
|---|---|---|---|
| **Vertical** (window ±5.6) | | | |
| Speed at feed (gate ±2.9 % → ≤ 1 % averaged + measured-speed compensation) | 3.0 | 1.1 | finding D |
| Per-ball dip in a burst (first ball 0) | up to 7 | ≤ 1 | finding D3 |
| Ball-to-ball η scatter (assumed 2 %; **unmeasured**) | 3.6 | 3.6 → 1.8 if 1 % | finding D4 |
| Aero / model (uncalibrated → fitted residual) | ~4 | ~1 | finding E |
| Exit height, hood, distance | 1.0 | 0.6 | H, §2 |
| **Total vertical (RSS)** | **≈ 6–9** | **≈ 4.1 (2.6 if η scatter is 1 %)** | |
| **Lateral** (window ±8.6) | | | |
| Turret scale (at 90°) | 6.9 | 0.3 | F |
| Tracking at 90 °/s (lag + latency) | 14 + 11 | ~1.5 | G |
| Relay limit cycle | ~2 | ~0.5 | G |
| Lever arm, parallax, exit offset | ~5 | ~0.3 | H |
| Heading (1°) | 2.1 | 2.1 | localization |
| **Total lateral (RSS), robot turning 90 °/s** | **≈ 20** | **≈ 2.7** | |

**Conclusion:**
- Lateral error is a **control and geometry** problem, and it is fixable in software.
- Vertical error is a **flywheel and ball-consistency** problem. After the software fixes, the
  ball-to-ball exit-speed scatter is the floor. Its size decides whether long shots are worth taking
  and whether the hood/squeeze needs mechanical work.

## 5. Plan, in order

**Phase 0: code, no robot needed (each one a separate, reviewable change):**
1. Flywheel units (B). Confirm with Pranav, rescale the gains, and add a unit test: steady-state
   power at 12 V ≈ kV·ω/12.
2. One loop timestamp at the bulk read, passed to Flywheel and TurretBase. Move the battery read
   after the control math (C).
3. Exit point as a function of hood angle in v1 `AimSolver`, the Sim launcher and the experiment
   (H). Aim from the turret axis, with the exit-offset parallax.
4. Hood: one limit set, plus per-servo trim (I.3, I.4).
5. Turret hub-velocity overflow correction (I.2).
6. Window-averaged flywheel speed, with the gate at 1 %. Log the speed **at each feed**.

**Phase 1: measure the real side (each fills an [unknown]):**

| Measure | How | Fills |
|---|---|---|
| Ball mass and diameter, 10 balls | scale, calipers | m, squeeze, η-scatter prior |
| Flywheel J, kS/kV/kA | spin-up/coast-down fit (`FlywheelCharacterization`) | dip per ball, recovery |
| η and its scatter, spin | Pollen Shot Experiment + 240 fps video | the vertical floor |
| k_d, Cl | landing points at 3 distances × 2 hood angles | aero model |
| Hood servo → launch angle | inclinometer, both servos' currents | hood map, fight |
| Turret ticks/°, deadband, dead time, backlash, K | Turret Diagnostic steps 1–3 | F, G |
| Loop dt distribution, T | HardwareMonitor + diagnostic `loop_dt_ms` | G crossover |
| Opening centre, tilt, shape, facing | real field / TU07 | A |

**Phase 2: control:**
- **Flywheel:** volts feedforward kS + kV·ω + kA·ω̇, plus P (small I). For a 4-ball burst, use
  per-feed recovery gating or energy banking. Set RPM/hood from the measured speed at feed.
- **Turret:** motion feedforward from yaw rate and bearing rate, target predicted at t + T, P(D)
  feedback, integral ≈ 0, kHold from the measured breakaway. Fire gate: |e| < atan(window/d).

**Phase 3: close the loop on reality:**
- Fit the physics model's parameters (η, k_d, Cl) to the Phase 1 shots.
- Keep only a small residual table.
- Validate on **held-out** distances; a fit is only trusted if it predicts shots it was not fitted on.
- Log predicted vs actual for every shot, and refit when the residual mean drifts by more than 1σ.

---

*Assumptions behind every number: 30 g ball unless stated, Cd 0.5, Cl 0, J = 2e-4 kg·m²,
η = 0.5, T = 60 ms, and the opening model from §1. Each is listed in §5 Phase 1 to be replaced by
a measurement.*
