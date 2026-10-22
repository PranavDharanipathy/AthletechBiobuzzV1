# Dev Notes: Shooter & Turret

Team 31666 (Athletech). **FTC BIOBUZZ 2026–2027.**
Working notes for shooter/turret programming. Keep this updated as things change.
`❓` = unknown / needs confirming on the real robot.

## Repo status
- **This repo (`AthletechBiobuzzV1`)** is the 26-27 robot code. On 2026-10-21 it was a bare FTC SDK v12.0 (empty TeamCode, no extra libraries).
- **Scope: shooter + turret only for now.** Other subsystems come later.
- Reference: `../AthletechPremiers2526` is last season's code. It has a mature flywheel PIDVS, a turret cascade (position → velocity), hood interpolation and auto-aim. Port ideas from it, but re-derive every constant for the new hardware.
- Last season's libraries (not yet added here): Pedro Pathing 2.1.x, FTC Dashboard 0.5.1, Panels (bylazar fullpanels), NextFTC 1.1, Apache commons-math3.

---

## 0. NEW ROBOT: "V1 Biobuzz Athletech 26-27" (from Onshape CAD)

> ⚠️ Sections 1–5 below describe **last season's code** (`AthletechPremiers2526`) and are kept for reference only.
> File paths there refer to that repo. This section is the source of truth for the new robot.

Source: Onshape doc `231d0d2ccd57c7da2e3d018d`, read-only via API on 2026-10-21. The CAD is V1 and may change.

### Coordinate conversion (CAD → robot frame)
- CAD −Y = robot **forward** (the intake is at −Y). CAD +X = robot **left**. CAD +Z = up.
- Floor is at CAD z ≈ −0.37 in (wheel center z = 1.68 in, 104 mm wheels).
- **Drivetrain center** (midpoint of the 4 mecanum wheels) = CAD (−4.82, 1.14).
- Wheelbase is 8.52 in (front to back). Track width is 13.39 in (left to right). Both are wheel center to wheel center.

### Turret
| Item | Value | Source |
|---|---|---|
| Rotation axis | CAD (−4.82, 3.96). **2.82 in behind drive center, centered left/right** | Revolute 13 (Top Turret Plate ↔ Turret Base) |
| Turret plate height | ~7.9 in above floor | |
| Drive | **24T pinion → 72T turret gear = 3:1** (servo turns 3× per turret turn) | Gear relation "Gear 1", ratio 0.3333 |
| Pinion shafts | 2, at CAD (−7.71, 2.35) and (−1.94, 2.35). Each is 3.31 in from the turret axis | Revolutes 19 and 21 ("Gear 2" = 1:1 reversed between them) |
| Actuators | Axon **Max** servos (25T spline) via Servo Hubs. ❓ Confirm 2 for the turret | BOM |
| Encoder | **REV absolute encoder**, at CAD (−4.25, 0.24). ❓ Is it geared (24T → 3:1?) or direct? | BOM / position |
| Bearing | 8 "Bearing Stacks" (thrust bearings + 8x14x5 bearings) | Turret Assy |
| Mechanical limits | None modeled in CAD ❓ (check wiring/hard stops) | |

If the encoder is a REV Through Bore (8192 CPR quadrature) **on a 3:1 gear**: `TURRET_TICKS_PER_DEGREE = 8192·3/360 ≈ 68.27`. If it's direct on the turret: 22.76. ❓ Confirm.

### Shooter (rides on turret)
- **1× goBILDA 6000 RPM bare motor** (5202, 28 CPR). It drives the flywheel shaft with a **20T → 20T GT2 belt = 1:1**.
- Flywheel stack: **3601 Rhino Wheel 72 mm** + **2× 72 mm SWYFT flywheels**, on a Sonic Hub (8 mm REX).
- Max flywheel free speed ≈ 6000 RPM = 2800 ticks/s. Surface speed ≈ 22.6 m/s (no-load).
- Flywheel axle is ~9.6 in above the floor.
- **Hood: 2× Axon Max servos**, one on each outer shooter plate. Each goes Servo Hub → linkage arm (Part 6/Part 7) → Hood. ❓ Do both servos move the hood together (mirrored)?
- Aligners and a ball Blocker are in the feed path.

### Other relevant
- **Limelight 3A**: centered left/right, **3.5 in forward of drive center**, ~8.6 in above the floor, tilted (CAD axis (0, −0.64, 0.77), about 40° from vertical). ❓ Confirm the camera pitch.
- **Pinpoint** + 2× goBILDA 4-bar odo pods. Pod positions: CAD (−3.40, 5.75) and (−4.76, 4.90).
- 5× 6000 RPM motors total: 4 drive + 1 flywheel. Drive is 20T pinion → 48T wheel gear.

---

## 1. Robot Specs (shooter-related, OLD 25-26 robot)

### Hardware map (`Constants/MapSetterConstants.java`)
| Device | Config name | Type | Notes |
|---|---|---|---|
| Left flywheel | `left_flywheel` | DcMotorEx | Encoder source for flywheel velocity |
| Right flywheel | `right_flywheel` | DcMotorEx | Gets the same power as left |
| Turret base L | `left_turret` | CRServoImplEx | 2 CR servos drive the turret |
| Turret base R | `right_turret` | CRServoImplEx | |
| Turret encoder | `intake` | external encoder | Uses the **intake motor's** encoder port (direction REVERSE) |
| Hood | `hood_angler` | Servo | Direction REVERSE |
| Blocker | `blocker` | Servo | Gate before the flywheel |
| Transfer | `transfer` | DcMotor | Feeds rings/artifacts into the shooter |
| Limelight 3A | `limelight` | USB | MT1/MT2 relocalization |
| Pinpoint | `pinpoint` | I2C | Odometry |

### Flywheel
- 2 motors, run with `RUN_WITHOUT_ENCODER` and a custom PIDVS loop (`Systems/Flywheel.java`).
- Motor free speed set to 6000 RPM, 12 V core. ❓ Motor model? (Probably a goBILDA 5202 at 1:1, 28 ticks/rev.)
- Flywheel assembly mass: 829 g. Shaft diameter: 8 mm.
- Velocity is filtered with a Kalman filter: `FLYWHEEL_KALMAN_FILTER_PARAMETERS = {56, 40, 4.25, 5}`.
- "At speed" tolerance: ±10 ticks/s velocity, ±10 ticks/s stability between loops.
- ❓ Wheel diameter, compression, and gear ratio.

### Turret
- Driven by 2 CR servos with an external encoder.
- `TURRET_TICKS_PER_DEGREE = 65.738`
- Soft limits: **−90° to +90°** from home (`MIN/MAX_TURRET_POSITION_IN_DEGREES`).
  - PD/FF tables span ±5800 ticks, which is about ±88°.
- Turret is offset from the robot center by `TURRET_POSITIONAL_OFFSET = -1.775591` in.
- Control is a cascade: **position loop** (`TurretBase`) → target velocity → **velocity loop** (`TurretActuator`). The velocity loop includes static, Coulomb, and Stribeck friction feedforward.
- Position Kalman filter: `{1050, 200, 1800, 5}`.
- ❓ Servo model, gear ratio, and encoder type/CPR.

### Hood
- One positional servo. Usable range is 0.0 to 0.78. The constants are named backwards: `HOOD_ANGLER_MAX_POSITION = 0` and `MIN = 0.78`.
- ❓ Which direction gives a flatter or steeper shot? `HOOD_POSITION_FOR_MAX_SHOT_DISTANCE = 0`.

### Field / goal (`Constants/FieldConstants.java`)
- Pedro coordinates are in inches. x is forward/back and y is left/right.
- Red goal at about (70, 70) and blue at about (−70, 70). There are separate aim points for close-alliance, close-opponent, and far.
- `FAR_ZONE_CLOSE_ZONE_BARRIER`: if y < −35, the robot is in the far zone.

---

## 2. Software Architecture

```
TeleOpBaseOpMode
 └─ Shooter (EffectivelySubsystem)          Systems/Shooter.java
     ├─ PoseEstimator  ← Pedro Follower + Camera (Limelight MT1/MT2)
     ├─ PoseSpeedTracker (robot vel/accel, used by THC)
     ├─ TurretBase      (position PID + hold FF) Systems/TurretBase.java
     │    └─ TurretActuator (velocity PIDVS on 2 CR servos)
     ├─ Flywheel        (velocity PIDVS, Kalman)  Systems/Flywheel.java
     └─ Hood            (distance→servo interp)   Systems/Hood.java
          └─ HoodAngler (servo wrapper, clamped)
```

### Shooter.update() loop order
1. Update the pose speed tracker and the camera. MT1 relocalizes automatically when the robot is slow and MT2 isn't eligible. Controller rumbles on MT1.
2. Update the pose estimator to get the robot pose.
3. **THC (turret hysteresis control)**: predicts the robot's future pose so the turret leads its motion. **Currently hard-disabled** (`if (false /*shouldUseTHC*/)`).
4. Compute the turret pose from the robot pose and the turret encoder.
5. Pick the goal aim point from the zone (y vs. the barrier).
6. Compute the angle to the goal → `routeTurret()` (wraps or clamps to ±90°) → ticks → `turret.setPosition`.
7. Set the hood aim zone and compute the distance to the goal.
8. Flywheel: **left bumper (gamepad1) toggles it**. Target velocity is interpolated from distance.
9. `turret.update()`, `flywheel.update()`, `hood.update(distance)`. The hood must update after the flywheel.

### Tuning OpModes (`Tuners/`)
- `FlywheelTuner`, `FlywheelVelocityKalmanFilterTuner`
- `HoodInterpolationTuner`: collects distance → hood/flywheel data points.
- `GeneralVeloMotorTuner`, `TransferTuner`
- Testing: `TurretBasePowerTesting`, `TurretActuatorTesting`, `HoodAnglerTesting`, `SpinFlywheelPowerBased`

Most constants are `@Config`, so they can be tuned live with FTC Dashboard or Panels.

---

## 3. Current Tuning State

| Item | Status |
|---|---|
| Flywheel PIDVS | Tuned (`FLYWHEEL_PIDVS_COEFFICIENTS`) |
| Turret position PID | Tuned, but `.withTuning(true)` is still on |
| Turret velocity loop | Tuned (`TURRET_VELOCITY_COEFFICIENTS`) |
| Turret PD/FF interpolation tables (`TURRET_KPS/KDS/KFS`) | All 0, not used yet |
| Close flywheel velocity table | 5 points, 48.5–98.2 in → 1300–1750 tps |
| Close hood positions | **All 0.0, not tuned** |
| Far hood table | 2 points (110, 150 in), **all 0.0, not tuned** |
| Far flywheel velocity | **No far table**; see issues below |
| Hood flywheel-velocity correction | Influence = 0 (off) |
| THC | Off. `THC_ACCELERATION_INFLUENCE = 0` |

---

## 4. Known Issues / Things Spotted in Code

- [ ] **The flywheel always uses the CLOSE table.** `getFlywheelTargetVelocityFromInterpolation` ignores the aim zone. In the far zone it clamps to 1750 tps. `FAR_SIDE_FLYWHEEL_SHOOT_VELOCITY = 1800` exists but is never used.
- [ ] Duplicate interpolation code in `Shooter` (flywheel) and `Hood`. Could be one shared lookup-table helper.
- [ ] **dt units differ.** `TurretBase` uses milliseconds (`timer.milliseconds()`) and `Flywheel` uses seconds. Gains aren't comparable between them. Be careful when porting.
- [ ] `TurretBase.update()`: `rawDerivative = (error - prevError) / dt` runs before the `dt > 0` check. On the first loop that can produce Inf/NaN, which then sticks in the low-pass filter.
- [ ] I-clamp divides by `ki` (`minI / ki`). When `ki = 0` (outside `iSwitch[1]`), the bounds become ±Infinity. That works by accident, but `0/0` would give NaN.
- [ ] Turret encoder shares the `intake` port name. Make sure nothing resets that encoder (e.g. `STOP_AND_RESET_ENCODER` on the intake motor).
- [ ] Commented-out driver-2 controls for nudging turret home and goal position (`Shooter.java` ~L272).
- [ ] No "ready to shoot" gate that combines flywheel at speed, turret on target, and hood settled before the blocker opens.

---

## 5. Plans / TODO

### Short-term
- [ ] Tune the close and far hood tables with `HoodInterpolationTuner`.
- [ ] Add a far-zone flywheel velocity table and select it by aim zone.
- [ ] Add a `isReadyToShoot()` check (flywheel `isAtVelocityAndStable` + turret error < X° + hood settled) and use it to gate the blocker/transfer.
- [ ] Turn off turret `.withTuning(true)` for competition builds.

### Medium-term
- [ ] Tune THC (lookahead model in `Constants/Models.java`), then enable `shouldUseTHC`.
- [ ] Shoot on the move: compensate aim and distance for robot velocity at release.
- [ ] Tune hood flywheel-velocity drop compensation (`FLYWHEEL_VELOCITY_HOOD_CORRECTION_INFLUENCE`) for rapid-fire.
- [ ] Re-enable driver-2 manual offsets for turret home and goal.

### Autonomous
- [ ] Make sure the shooter works in auto with pose transfer (`PoseTransfer`, `PostAutonomousRobotReset`).

---

## 5b. Decision Log (26-27)

Format: **D-# Title** — Context · Options considered (≥3, incl. one unconventional) · Decision · Why others lost · Revisit if…

**D-0 Ground rules** (2026-10-21). From the team: no PID, no bang-bang flywheel, explicit voltage compensation, no third-party libraries for now, CAD read-only. See `CLAUDE.md`. The team confirmed that observer-based corrective feedback (ADRC/ESO) counts as "different feedback" and is allowed.

Full debates, numbers and verification are in **`docs/SHOOTER_DESIGN.md`**. Summary:

| ID | Decision | Rejected alternatives |
|---|---|---|
| D-1 | Flywheel: identified feedforward + model-assisted ADRC; reference shaping; shot anticipation + **energy banking**; model-based current limit | FF-only, MPC, sliding mode, bang-bang |
| D-2 | Speed sensing: 3rd-order discrete ESO on encoder position (ωo = 30) | Differencing (±5.6 % noise), hub getVelocity, Kalman on differences |
| D-3 | Voltage: 20 Hz sampled, filtered, sanity-bounded; ESO absorbs the remainder | Per-loop reads (I/O cost), internal-resistance prediction |
| D-4 | Turret: Axon in **continuous** mode + external sensor (range ±200°) | Axon positional mode (only 118° at 3:1) |
| D-5 | Turret: time-optimal moving-target reference + FF + ADRC; yaw-rate FF; park (no hunting); optional anti-backlash preload | MPC, sliding mode, pure FF |
| D-6 | Turret sensor behind an interface (quadrature or analog absolute), with health checks | — (hardware undecided) |
| D-7 | Ballistics: physics (drag + Magnus) + calibrated η + residual table; robust-arc choice with **6 % speed reserve**; cached table | Pure LUT, pure physics, ML |
| D-8 | Shoot on the move: latency-compensated virtual-target iteration | No lead (30 in miss in test) |
| D-9 | Fire gate with first-failing-reason reporting | Speed-only gating |
| D-10 | Pure-Java core + simulators + 34-check suite; hardware layer separate; zero allocations | — |

---

## 6. Changelog

| Date | Change |
|---|---|
| 2026-10-21 | Created dev notes from existing code review. |
| 2026-10-21 | Added 26-27 robot specs from Onshape CAD (section 0). |
| 2026-10-21 | Moved notes into new repo `AthletechBiobuzzV1`. Scope: shooter + turret. |
| 2026-10-21 | Shooter/turret/hood/aim v1: `TeamCode/.../shooter/{core,hardware,opmode}`, simulators + tests (`tools/run_tests.sh`, 34/34 pass), design doc `docs/SHOOTER_DESIGN.md`. Builds and lints clean against SDK 12. Not yet run on the robot. |

---

## 7. Open Questions for the Team
- Which libraries do we want this season (Pedro, Dashboard/Panels, NextFTC)?
- **Hood must reach about 40°–80° launch angle** to score from 0.7–3.5 m (ballistics study). Can the linkage do that?
- Full ❓ list with how to resolve each: `docs/SHOOTER_DESIGN.md` §9.
- Turret encoder: exact model and gearing. Number of turret servos.
- Hood: are the two servos mirrored? What is the servo-position → hood-angle range?
- Turret travel range on the new robot (no limits in CAD).
- Motor, servo, and gear specs for the flywheel and turret (marked ❓ above).
- Max useful turret range. Is ±90° a mechanical limit or a wiring limit?
- Shot cadence target (balls per second) and how many balls are held at once.
- Is driver 2 used in TeleOp?
