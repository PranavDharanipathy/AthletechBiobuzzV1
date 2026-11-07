# AutoPath Red

Driver Station: **AutoPath Red**, group **A_Match**.

## Sequence

1. Follow path 1, hold its endpoint, and shoot.
2. Follow paths 2 and 3, hold path 3's endpoint, and shoot.
3. Follow paths 4 and 5, hold path 5's endpoint, and shoot.
4. Follow paths 6 and 7, then stop.

The intake runs at existing `IntakeConstants.INTAKE_VELOCITY` throughout active travel AND shooting. The flywheel spins and the turret auto-aims throughout. The blocker stays closed outside the three shooting windows. INIT does not spin up, intake or aim; the turret must be straight at INIT because the existing turret constructor establishes its home there.

The route keeps the supplied seven-step red-half pickup/shoot logic and headings, with revised endpoints/control points for clearance and the flywheel-table range. The third shooting station is now on the south side. It is not alliance-mirrored or rescaled. Geometry lives in `Autonomous/Paths/AutoPathRoutes.java`; routine in `Autonomous/Autos/AutoPath.java`.

## Shooting behavior

After each designated path, hold the endpoint. Require robot position/heading and motion tolerances, a valid aim solution that is not waiting at a turret travel end/swinging around, turret on-target, and flywheel speed readiness. These autonomous readiness checks operate even though the match TeleOp gate-enable flags are currently false; global flags are unchanged.

The flywheel gate uses an initial ±50 ticks/s entry band, ±100 ticks/s hold band, a dwell, and a 50 ms minimum-open allowance for speed drops. A normal shot dip can pause feeding and trigger another readiness dwell. Aim/motion faults can close immediately. The blocker only opens while the shot window is FIRING. Timings are loop-based, not sensor-confirmed ball release.

Default: **1.5 cumulative seconds commanded open**, within a **4-second wall-time window**, at each of paths 1/3/5. This is a configurable timed burst, not proof four balls fired. If readiness never arrives or recovery consumes the timeout, close and continue the route; count the shot timeout in telemetry. A timing fault aborts the routine. No ball-count sensor is added, so continuous intake cannot itself enforce the four-piece control limit.

## Settings

`Constants/AutoPathConstants` is Dashboard-configurable. Settings are snapshotted during INIT; change values and re-INIT for the next run.

| Setting | Initial value |
| --- | --- |
| AUTO_LIMIT_SECONDS | 29.5 |
| PATH_TIMEOUT_SECONDS | 8 |
| SHOT_TIMEOUT_SECONDS | 4 |
| SHOT_FEED_SECONDS | 1.5 cumulative open seconds |
| SHOT_READY_DWELL_SECONDS | 0.1 |
| FLYWHEEL_ERROR_TICKS_PER_SECOND | 50 |
| POSITION_ERROR_INCHES | 1 |
| HEADING_ERROR_DEGREES | 3 |
| MAX_SHOOTING_SPEED_INCHES_PER_SECOND | 3 |
| MAX_SHOOTING_YAW_DEGREES_PER_SECOND | 10 |
| MAX_LOOP_SECONDS | 0.15 |

These are initial experiment values, not field-tuned guarantees. All outputs stop on completion, stop request, invalid localization, path timeout, loop fault, exception, or autonomous deadline. Cleanup attempts every output even if one device fails. The follower is updated once per loop; the cache and HardwareMonitor are serviced on the same thread. No blocking shooting sleeps or hardware background threads.

## Revised coordinates

All coordinates are inches; headings are degrees. Start is unchanged because its wall-contact reference must match how the robot is physically staged.

| Point | Original x, y | Revised x, y | Heading | Purpose |
| --- | --- | --- | --- | --- |
| Start | 31.4072, 8.2495 | unchanged | 90 | Original wall-contact start |
| 1 | 43.2487, 12.0376 | 20, 17 | 90 | First shot, south side |
| 2 | 8.9248, 10.2542 | 10, 16.75 | 270 | Garden approach, intake facing wall |
| 3 | 54.2865, 123.8186 | 20, 128 | 300 | Second shot, north side |
| 4 | 59.0665, 32.0101 | 34, 32 | 270 | South collection leg, west of hive |
| 5 | 58.3463, 121.8874 | 17, 18 | 270 | Third shot, south side |
| 6 | 8.3808, 58.6368 | 11, 58.6368 | 270 | Left-wall flower approach |
| 7 | 8.8433, 94.5704 | 11, 94.5704 | 270 | Original final region, farther off side wall |

Controls changed: path 2 → (32,44), path 4 → (32,100), path 6 → (24,54). Path 3 keeps its original (31.6945,122.2396) control. The west-side corridor avoids sweeping through the central hive envelope. Path 5 no longer returns all the way north after the south pickup; it shoots from that side instead. Intake remains running throughout and the shooting stages are still after 1/3/5.

### Geometry/error checks

Run `python3 tools/check_auto_path.py` after route/geometry changes. It compiles the actual production routes with the cached Pedro 3.0.2 jar and uses the production FieldConstants/ShooterConstants distance table. An optional `--core-jar` supports another cache location.

The planning footprint uses confirmed intake-down reach 359 mm = 14.134 in and half-width 7.64 in. Rear reach **8.2495 in is inferred from the unchanged heading-90, rear-wall-contact start**, not independently measured. Verify that inference and the pose origin on the real robot. The original wall-contact start is deliberately not inflated by an uncertainty margin until it has moved 3 in off that wall.

At 4001 samples per leg, nominal rectangle wall clearance after the first leg is at least **2.24 in**; the garden endpoint's intake nose is **2.62 in** from the wall. An illustrative perturbation envelope of ±1 in on each position axis and ±3° heading leaves at least **0.63 in** wall clearance after the initial contact departure. These assumed errors match the tightened endpoint tolerances; the follower does not guarantee that tracking errors stay inside those bounds. Larger real errors may invalidate clearance.

A conservative central review envelope starts at x=45 with y=45..99. The modeled body stays west of it with at least **1.03 in** clearance under those assumed errors. This envelope is a planning approximation, not a measured collision mesh or claim that every point inside is physically blocked. Flower retrieval intentionally approaches a structure; this 2-D check does not certify intake/flower height or pickup success.

The turret-to-selected-goal distances at shots 1/3/5 are **57.52 / 58.94 / 55.13 in**. With the same perturbations, ranges remain **56.01..59.03 / 57.49..60.39 / 53.61..56.65 in**, inside the existing 51.715..100.443 in table domain. This removes distance clamping; it does not prove ballistic accuracy, rim clearance, backspin behavior, or retention.

### Remaining physical/state requirements

- **Cell state:** shots now alternate south → north → south, consistent with the plan that each prior burst tips the hive. That is an expectation, not a sensor observation. A missed/incomplete tip, an extra tip, or a partner's tip can change which cell is actually up. Coordinate-only planning cannot guarantee this; verify the expected state/tip reliability before using this open-loop routine. Do not blindly increase burst duration and assume one burst means exactly one tip.
- **Start configuration:** the unchanged starting pose requires an actual wall-contact measurement. Confirm the legal 18-in starting configuration and intake deployment arrangement. The confirmed 14.134-in deployed front reach is a travel footprint; a running intake motor is not an intake-deployment command.
- **Pickup:** moving the garden approach preserves a near-wall intake approach, but does not guarantee all four staged balls enter. Confirm intake-mouth width/reach, compliance, retrieval height and ball positions. Point 4 remains a south-side floor pickup, whose supply is uncertain if it depends on spilled pollen. Point 6 remains a flower approach. No collection sensing is added.
- **Feed/capacity:** calibrate burst duration/recovery and confirm the blocker retains balls in transit. Continuous intake does not enforce the four-piece control limit by itself.
- **Field coordinates:** verify the robot's 144-in corner frame and visualizer 141.5-in convention when physically laying out this revised route; no silent scaling is applied.
- Validate actual tracking errors, swept mechanism clearance, starting contact, pickup success, hive state and total runtime on the field. The route is about **361.3 in** long in the geometric check; runtime is not distance divided by top speed.

## Integration changes

`Shooter.updateAutonomous(boolean)` reuses the match aiming/flywheel calculations without reading gamepads. TeleOp `update()` keeps its button behavior. `isAimSolutionUsable()` exposes the travel-route check for autonomous feed readiness. `GeneralVeloMotor` has a device-injection constructor so auto uses monitored hardware with the existing intake controller. The single hardware-free `AthletechRobot` satisfies NextFTC's scanner without another hardware owner. This SDK/Ivy routine controls its own lifecycle; NextFTC hooks do not advance it.

## Verification

Android debug assembly passes. Seven new Sim tests run the actual AutoShotWindow class and cover uninterrupted readiness, recovery pauses, cumulative open time, timeouts, timing faults, new-shot reset and invalid settings. The existing Sim suite is also run. The reproducible `tools/check_auto_path.py` constructs all seven paths with the actual Pedro 3 jar, verifies endpoint continuity, and checks 28007 footprint samples plus assumed position/heading errors and shooting range bounds. No on-robot tests or deployment have been performed.
