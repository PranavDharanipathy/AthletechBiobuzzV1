# Shooter / Turret / Hood / Aim: Engineering Design

Team 31666 Athletech, BIOBUZZ 2026-27. Written 2026-10-21.
Process and constraints: see `CLAUDE.md`. Specs and the CAD extraction: see `DEV_NOTES.md` §0.
`❓` = assumption that must be measured or confirmed. Every ❓ has a characterization OpMode or field measurement that resolves it.

---

## 1. Problem statement (from the game manual, Competition Manual TU04/TU05)

- **Target:** the alliance's **upward-facing CELL** on the HIVE in the **center of the field**.
  - Opening: 20 in wide × 14 in tall. Bottom edge 53.5 in, top edge 65.6 in above the tiles, so the center is **≈59.5 in**.
  - The HIVE is tilted 30° in its stable state. Pivot at 43.95 in. The two CELLs are 18.84 in apart, and the red and blue HIVEs are 25.5 in apart (center to center).
  - **The target moves:** after every TIP the other CELL becomes the upward one, so the aim point jumps. The code tracks which CELL is up.
- **Game piece:** POLLEN, a ≈2.8 in (71 mm) polyethylene ball (Gopher ResisDent). NECTAR is 3.6 in. Our 72 mm wheels are sized for POLLEN, so the shooter is designed for POLLEN only ❓.
- At most **4** game pieces controlled at once (G407). **No launch-zone restriction**, so we can shoot from anywhere. Robot max height when expanded: 29 in.
- Shooting into the side or bottom of a CELL is a foul if deliberate (G417). Accuracy is a rules issue, not just points.

## 2. Requirements (measurable; proposed values, user may revise ❓)

| ID | Requirement | Target | Basis |
|---|---|---|---|
| F1 | Flywheel spin-up 0 → 4000 rpm | ≤ 1.1 s at 12.5 V | physics bound: τ ≈ J·R/(Kt·Ke) ≈ 0.9 s at full voltage |
| F2 | Steady-state speed error (RMS) | ≤ 1 % | exit speed error → range error ≈ 2 % |
| F3 | Recovery after a shot to within 2 % | ≤ 250 ms | 4-ball bursts |
| F4 | Effect of battery 12.8 V → 11.0 V | < 1 % speed change | voltage independence |
| T1 | Turret 90° step settle within ±1° | ≤ 0.8 s | servo speed bound ≈ 170°/s at the turret ❓ |
| T2 | Tracking error while robot spins at 90°/s | ≤ 1.5° | shoot on the move |
| T3 | No hunting or limit cycles at rest | 0 | backlash + servo deadband |
| A1 | Lateral aim tolerance | computed: atan((10 in − 1.4 in)/d) | 20 in opening, 2.8 in ball |
| S1 | Core math per loop | ≤ 0.3 ms, **zero allocations** | loop budget |
| S2 | Any sensor fault | fail safe within 1 loop, reported | robustness |

## 3. Physics models

### 3.1 Flywheel (goBILDA 5202 6000 rpm, 1:1 GT2 belt, 72 mm Rhino + 2× SWYFT)
Electrical: `V = i·R + Ke·ω`. Mechanical: `J·ω̇ = Kt·i − b·ω − τc·sgn(ω) − τ_ball`.
Eliminating i gives the standard identified form, which characterization measures directly:

```
V = kS·sgn(ω) + kV·ω + kA·ω̇ + (disturbances)
kV = Ke + b·R/Kt   [V/(rad/s)],  kA = J·R/Kt  [V/(rad/s²)],  kS = τc·R/Kt  [V]
```
Datasheet priors (goBILDA 5202 1:1): 12 V, 6000 rpm free, 9.2 A stall, 0.144 N·m stall, so R ≈ 1.30 Ω, Kt ≈ 0.0157 N·m/A, Ke ≈ 0.0186 V·s/rad.
J (CAD): Rhino 6.34e-5 kg·m², plus 2× SWYFT (no CAD mass; ≈1.2e-4 estimated), plus shaft, hub and pulley. **J ≈ 2e-4 kg·m² ❓**.
Prior estimates: kV ≈ 0.019 V/(rad/s), kA ≈ 0.017 V/(rad/s²), mechanical time constant ≈ 0.9 s.

Ball shot energy: KE_ball ≈ ½·m·v² ≈ ½·0.05·8² ≈ 1.6 J (mass ❓), plus slip and spin losses, so roughly 3 J taken from the flywheel (½·J·ω² ≈ 17 J at 420 rad/s). That is a **5–10 % speed dip per ball**. Recovery is power-limited (≈ 30–40 W available), which takes ≈ 100–200 ms. This is why §4 includes shot anticipation and speed-adaptive hood.

Sensing: the encoder is **28 counts/rev**. At 400 rad/s and a 10 ms loop, that is ≈ 18 counts per loop, so differencing has ±1 count = **±5.6 % velocity noise**. Naive velocity is unusable, which drives the observer design (D-2).

### 3.2 Turret (Axon Max ×2 → 24T pinions → 72T ring, 3:1)
Servo in continuous mode: the command u∈[−1,1] sets motor voltage through the servo's driver from a regulated servo rail (5–6 V), so it is **largely independent of battery voltage**. Turret dynamics in identified form:
`u = kS·sgn(θ̇) + kV·θ̇ + kA·θ̈ (+ backlash, deadband)`.
Axon Max ≈ 0.115 s/60° at 6 V ❓ gives ≈ 520°/s at the servo, so **≈ 170°/s at the turret**. This is slower than a hard robot spin (> 300°/s), so the aim code must handle turret speed saturation gracefully.
Backlash: two pinions on one ring gear, so there is gear lash at each mesh.

### 3.3 Hood
Two mirrored Axon Max servos (positional mode) drive a four-bar linkage (CAD: servo, arm "Part 6", link "Part 7", hood), so **servo position → hood angle is nonlinear**. It is modeled with a measured monotone table.

### 3.4 Ball flight
`m·a = m·g + F_drag + F_lift`, with `F_drag = −½ρ·Cd·A·|v|·v` and Magnus lift `F_lift = ½ρ·CL·A·|v|²` perpendicular to velocity (hood shooters give backspin, which produces lift).
71 mm ball: A = 3.96e-3 m², Cd ≈ 0.5 ❓, m ≈ 0.05 kg ❓, CL ≈ 0.1–0.25 ❓. At 8 m/s, drag deceleration ≈ 1.9 m/s², about **20 % of gravity**, so drag is not negligible.
Exit speed: `v_exit = η·ω·r_wheel`. For a single wheel against a fixed hood with no slip, the ball center moves at ½ the surface speed, so **η ≈ 0.45–0.5 ❓** (calibrated from shots).

---

## 4. Decisions (alternatives debated)

### D-1 Flywheel control law
Options:
1. **Feedforward only** (kS + kV·ω + kA·ω̇, voltage-scaled). Best for spin-up, but has no correction: it drifts with temperature and friction and is blind to shots. *Rejected alone.*
2. **Feedforward + model-assisted ADRC** (extended state observer estimates speed **and** total disturbance; control = feedforward + bandwidth-limited correction − estimated disturbance). Disturbance cancellation replaces an integrator, with no windup when the input fed to the observer is the *saturated* one. **CHOSEN.**
3. **MPC.** Constraint handling is nice, but for a first-order plant with an input bound, the optimal action is "saturate, then track", which option 2 already achieves through reference shaping. More CPU, harder to tune, no gain. *Rejected.*
4. **Sliding mode** (boundary layer). Robust, but with 28-count sensing the switching term amplifies quantization chatter, which means audible and electrical noise. *Rejected.*
5. **Bang-bang.** Forbidden by the team (and noisy, overshoots, current spikes).
6. *Unconventional (adopted as complements):*
   - **Shot anticipation:** we command the feeder, so we know when energy will be taken. Inject a learned voltage pulse at contact time instead of reacting after the dip.
   - **Speed-adaptive hood (D-5):** instead of waiting for the flywheel to recover, re-solve the hood angle for the *actual* speed. The shot is right even when the speed is off.
   - **Model-based current limit:** `i = (V − kV·ω)/R`, so the command can be bounded to keep current ≤ I_max. This prevents brownouts on spin-up and braking.
- **Reference shaping:** the internal reference ramps no faster than the motor can physically follow at the present battery voltage (`ω̇_max = (V_avail − kS − kV·r)/kA`). This prevents overshoot after saturation.
- **Energy banking** (added after simulation): anticipation alone did *not* help (132 → 142 ms), because recovery is power-limited and the extra voltage just saturates. The physics-correct move is to **store energy before contact**: once a feed is notified, the reference rises by half the learned per-ball loss, and the dip lands around the target. The result is 28 ms, with the dip inside ±2 %. A second finding: when the reference comes down while the wheel is already below it, the reference **snaps to the measured speed** instead of ramping. Ramping had commanded a −6 V "deceleration" that the ball had already done.
- **Observer bandwidth study:** ωo = 45 gave ±1.1 V ripple, ωo = 30 gives ±0.43 V. 30 was chosen; a shot is still 17σ above the noise.
- **Fault handling:** NaN or glitch rejection, plus **stall detection** (high voltage with no speed → latched 0 V fault, cleared by commanding 0). This catches an unplugged encoder, which reads a *finite constant*.
- Revisit if: the ESO disturbance estimate is noisy enough to cause > 1 % RMS speed noise. Then lower ωo or add hub velocity fusion.

### D-2 Velocity estimation
Options: (a) position differencing (±5.6 % noise, see §3.1); (b) hub `getVelocity()` (firmware-computed, unknown window and latency, still quantized); (c) Kalman on differenced velocity (last season); (d) **3rd-order ESO driven by encoder position**, which acts as an optimal-ish velocity + disturbance estimator using the motor model. (d) is **CHOSEN**: one observer serves estimation and control. It uses the discrete *current-estimator* form (Miklosovic & Gao 2006) with gains computed from the measured dt **every loop**, so loop-time jitter cannot destabilize it. Innovation clamping rejects encoder glitches.

### D-3 Voltage compensation
Power = V_cmd / V_battery_filtered. The battery sensor read costs an I/O transaction (≈1–2 ms), so it is **sampled at 20 Hz**, filtered with a 50 ms time constant and sanity-bounded (6–16 V). The ADRC disturbance estimate absorbs residual sag between samples (sag acts as an input-gain error, which the ESO sees as a disturbance). Alternative considered: estimating battery internal resistance to *predict* sag. Rejected for now because the current drawn by the drivetrain is unknown to the shooter. Revisit if speed dips correlate with drive acceleration in the logs.

### D-4 Turret actuation mode
| Option | Range at 3:1 | Control | Verdict |
|---|---|---|---|
| Axon positional mode (internal loop) | ~355°/3 ≈ **118°** | black box, two servos may fight | too little range; not our control |
| **Axon continuous mode + external angle sensor + our control** | **±180°+** (cable-limited ❓) | full model-based control, preload possible | **CHOSEN** |
| Positional + "unwind" tricks | 118° | complex | rejected |

Shooting from anywhere on the field requires about ±180° of coverage, otherwise the drivetrain has to turn to aim. Default soft limits are **−200°…+200°** (overlap gives hysteresis so the turret never unwinds mid-shot) ❓ cables.

### D-5 Turret control law (no PID)
Options:
1. **Online time-optimal reference generator + inverse-model feedforward + ADRC** (ESO estimates angle, rate and disturbance). The reference generator plans a jerk-free, accel-limited path to a *moving* target, including the robot yaw rate. **CHOSEN.**
2. MPC: too heavy for the gain. 3. Sliding mode: chatters through backlash. 4. Pure feedforward: no correction.
- **Robot-rotation feedforward:** turret target rate = target bearing rate − robot yaw rate. This is the dominant term when shooting on the move.
- **Hunting prevention:** once settled on a still target, the turret **parks** (output released). It un-parks at 2× the hold band, so it rests instead of dithering through deadband and lash. Verified: 0 sign changes at rest.
- **Stuck detection** on the *raw* measurement (high power with no change in reading) catches a frozen or unplugged encoder in 0.5 s. A first version used the observer's velocity, which kept "predicting" motion and took 1.4 s.
- **Anti-backlash preload (unconventional):** the two pinions drive one ring gear. Commanding them with a small *opposing* bias (`uL = u + p`, `uR = u − p`) loads both tooth flanks, eliminating lash at hold. The bias fades as |u| grows, so full torque is still available. Default p = 0 until it has been tested for servo heat ❓.
- Routing: choose the equivalent angle within limits that is nearest the current angle, with hysteresis, so it never takes the long way around or flips at ±180°.

### D-6 Turret angle sensing (encoder unknown, so design around it)
Interface `TurretAngleSensor` with implementations:
- **Quadrature encoder on a motor port** (REV Through Bore at 3:1 gives 8192·3 = 24576 counts per turret rev ❓). Relative only: homed at init against a known pose (stored last pose or a startup position).
- **Analog absolute** (an Axon analog feedback wire or an analog absolute encoder). If it is on the pinion (3:1), each reading is ambiguous across 3 sectors, so the sector must come from the startup pose.
- Health checks: NaN, implausible jumps (> max speed·dt), stuck readings while commanded to move → fault, turret output 0.

### D-7 Ballistics: physics model + calibrated correction
Options: (a) pure lookup table from field testing (last season; needs many points, can't extrapolate, and breaks when anything changes); (b) pure physics (unknown Cd/CL/η); (c) **physics model with calibrated parameters + a residual correction table** **CHOSEN**; (d) ML regression (overkill, needs data). (c) needs only a few calibration shots to fit η (and CL), and extrapolates sensibly.
- Precomputed at init, so there's no runtime integration:
  - **1D nominal table** over distance: the optimal hood angle, flywheel speed and time of flight.
  - **2D table θ(d, v_exit)** for the speed-adaptive hood.
- **Trajectory choice per distance:** among feasible angles (hood range, speed limit, descent angle ≥ minimum entry angle ❓), minimize the predicted vertical miss variance `(σv·∂h/∂v)² + (σθ·∂h/∂θ)²`, with a small penalty on time of flight and on being near hood limits. This chooses the arc that is *least sensitive* to our real errors.
- **Speed reserve** (found in simulation): the optimizer above converges to the *minimum-energy* arc, because ∂h/∂θ = 0 there. But at that point **any** speed dip is uncompensable at every angle (only 3/24 test points were solvable). Fix: require the exit speed to be ≥ v_min/(1 − 6 %). The hood can then re-aim for dips up to 6 %. σv in the cost is the *measurement* error (≈1 %), since the hood compensates using the measured speed.
- **Table cache:** a build takes ≈0.45 s on a PC (several seconds on a Control Hub). The table is saved to `/sdcard/FIRST/shooter_shot_table.bin` with a signature over every input, so it rebuilds automatically (on a background thread at init) only when a parameter changes.

### D-8 Shoot on the move
"Virtual target" fixed-point iteration: aim at `target − v_launcher·tof(d)` (3 iterations), using the pose predicted ahead by system latency (sensor → compute → actuation, ❓ measured). Launcher velocity includes the turret-offset lever arm (ω × r). The turret target rate feeds D-5's feedforward.

### D-9 Fire gate (readiness)
A shot is allowed only if **all** of the following hold, and the code reports the *first* failing reason for driver feedback and logs:
1. All sensors are healthy.
2. The target is reachable within turret limits.
3. The turret is within the computed angular tolerance.
4. Turret rate error is small.
5. The flywheel is stable, and its speed is inside the 2D table's valid range.
6. The hood has arrived at the angle required for the *current* speed.
7. Not inside the post-shot recovery window (unless burst mode).

### D-10 Software architecture
- **Pure-Java core** (`shooter/core`, no Android imports) holds the models, observers, controllers, ballistics and aim. It is unit-tested on a PC against plant simulators (`src/test`) with quantization, voltage sag, delays, backlash, deadband and shot disturbances.
- **Hardware layer** (`shooter/hardware`) holds device mapping, write caching, voltage sampling and sensor implementations.
- **OpModes** (`shooter/opmode`) are characterization and calibration (to measure every ❓) plus a test TeleOp.
- No allocations in the loop. `System.nanoTime()` timing. All math is dt-aware. Every input is NaN-guarded.

---

## 5. Calibration procedure (do in this order)
1. `Shooter: Flywheel Characterization`: gives kS, kV, kA (OLS fit, R² printed) and the CSV log.
2. `Shooter: Turret Characterization`: gives deadband/kS, kV, kA, backlash and max rate.
3. `Shooter: Hood Calibration`: servo position → hood angle table (measure the angle with a phone inclinometer).
4. `Shooter: Shot Calibration`: shoot at measured distances. Record whether each shot was high or low and adjust η (then CL). Then fill the residual table.
5. Measure on a real field: CELL opening center (x, y, z) for both CELL states.
6. Measure latency: log command time vs. observed turret motion.

---

## 6. Verification results (simulation; `tools/run_tests.sh`, 34 checks, all passing 2026-10-21)

The simulators are deliberately *richer than and different from* the controllers' models:
- **Flywheel:** +20 % inertia mismatch, battery internal resistance plus a 35 A drivetrain load, 28-count quantization, 8–15 ms loop jitter, 2 ms I/O delay, 2.5 J ball contacts.
- **Turret:** two servos with deadband, 1° gear lash per mesh, stiff contact dynamics, about 25 % error in the model constants, and robot yaw-acceleration torque.

| Req | Result | Notes |
|---|---|---|
| F1 spin-up | 1.17 s vs **1.14 s physical minimum** (12.8 V); 1.48 vs 1.45 s at 11.3 V | Within 3 % of what the motor can physically do |
| F2 steady error | **0.12 % RMS** (0.30 % max) | Requirement 1 % |
| F4 voltage | 0.15 % RMS at 11.3 V; 0.50 % max during a 35 A / ~2 V sag step | Voltage-independent |
| F3 shot recovery | **28 ms** with anticipation + energy banking vs 132 ms reactive | Dip stays inside ±2 % (−1.6 % vs −5.3 %) |
| F3 timing error | 108 ms with a 20 ms contact-timing error | Graceful |
| Shot detection | 4/4, no false positives | Gated at speed, hysteresis |
| T1 90° step | 0.77 s (0.64 s with preload), 1.0° overshoot | Requirement 0.8 s |
| T2 robot spin 90°/s | **0.44° RMS**, 1.27° max | Requirement 1.5° |
| T2 robot spin 150°/s | 2.1° RMS, 4.0° max | 87 % of servo speed: saturation-limited (see finding 4) |
| T3 hunting | 0 output sign changes at rest, 0.02° rest error | Park logic |
| A1 stationary aim | 0.16 in worst miss (3D flight, 5 positions) | |
| D-8 moving aim | **1.4 in** miss at 36 in/s + 60°/s (30 in without lead) | Opening allows ≈ 8.6 in |
| S1 full system | **4/4 hits** stationary and while driving 30 in/s + rotating 40°/s; first shot ≈ 0.95 s from cold | |
| Speed-adaptive hood | ≤ 12.8 mm off-grid interpolation error at ±5 % speed, 28/28 points | |
| SysID | kV within 0.1 %, kA within 1.2 %, kS within 7 % of truth | Centered differences |
| S2 faults | NaN sensor → 0 output in 1 loop; dead flywheel encoder → fault in 0.70 s; frozen turret encoder → fault in 0.52 s; glitch → 0.3 % | |
| S1 loop | **0 bytes allocated/loop**, 0.2 µs on PC | |
| Build | `assembleDebug` + Android lint clean (API 25 safe) | |

## 7. Design findings (from the studies; they change requirements for other subteams)

1. **Hood range must reach about 40°–80° launch angle.** Below about 75° max, the robot can't score from closer than about 1.1 m. → Mechanical ❓
2. **The arc is steep (63°–79°), flight time 0.8–1.2 s.** Requiring a ≥ 25° entry angle into the upward-facing CELL rules out flat, fast shots. Lead compensation makes the long flight time acceptable while moving.
3. **Speed reserve (6 %) is needed.** The angle-robust choice is the minimum-energy arc, but there *any* speed dip is uncompensable. We run 6 % above minimum energy, so the hood can absorb dips by re-aiming.
4. **Turret speed ≈ 170°/s (Axon Max at 3:1) is the binding limit for shooting while spinning.** Tracking is precise to about 100°/s of robot rotation. Above that the fire gate holds shots. Drivers (or drive-assist code) should keep yaw rate ≤ 100°/s while firing.
5. **Flywheel needs only ≈ 2900–4500 rpm**, so the 6000 rpm motor has plenty of headroom for recovery.
6. **Observer bandwidth is set by the 28-count encoder.** ωo = 30 rad/s balances command ripple (±0.43 V) against reaction time. A higher-resolution flywheel encoder (external) would allow faster rejection, which is optional.

## 8. Integration (how a TeleOp/Auto uses it)

```java
Shooter shooter = new Shooter(hardwareMap, HiveTarget.Alliance.RED, /*manageBulkCache=*/true);
RobotState rs = new RobotState();
// loop:
rs.set(xIn, yIn, headingRad, vxInPerS, vyInPerS, omegaRadPerS, shooter.time()); // from Pinpoint/odometry
shooter.setTracking(gamepad2.right_bumper);       // spin up + aim
shooter.update(rs);
if (fireButton && shooter.canFire()) { feeder.release(); shooter.notifyFired(); }
if (hiveTippedButton) shooter.notifyHiveTipped();
shooter.addTelemetry(telemetry);
```
- Pose must be in the **field frame of `HiveTarget`** (origin at field centre/HIVE pivot, inches). Pass the timestamp *when the pose was measured*, so latency is compensated correctly.
- If the OpMode already manages bulk caching, construct with `manageBulkCache=false` and clear the cache yourself, once per loop, *before* `update`.
- `canFire()` is false with a reason (`core().readiness()`). Show it to drivers; it's the fastest way to diagnose.

## 9. Open questions ❓ (each has a resolving test)

| # | Question | Resolve by |
|---|---|---|
| 1 | Hood launch-angle range achievable (need ~40–80°) | CAD linkage / Hood Calibration |
| 2 | Turret encoder: model, mounting (3:1?), wiring (quadrature vs analog) | Mechanical/electrical decision → `ShooterConfig.TURRET_SENSOR` |
| 3 | Turret cable-limited travel (default ±200°) | Mechanical |
| 4 | POLLEN mass, Cd, CL; exit efficiency η | Weigh a ball; Shot Calibration |
| 5 | Exact CELL opening centre (x, y, z) per state and HIVE x positions; minimum entry angle | Official field CAD / measure on field |
| 6 | Exit point height/offsets | CAD (shooter assembly) / measure |
| 7 | System latency (pose age + loop + actuation) | Log analysis (ShooterTuning CSV) |
| 8 | Do both hood servos move mirrored (mirrorOffset = 1.0)? | Hood Calibration |
| 9 | Does the shooter ever launch NECTAR (3.6 in)? | Strategy; 72 mm wheels suggest POLLEN only |
| 10 | Anti-backlash preload: servo temperature at 0.12 | Bench test (thermometer) |
