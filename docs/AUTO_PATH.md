# AutoPath Red

Driver Station: **AutoPath Red**, group **A_Match**.

## Sequence

1. Follow path 1, hold its endpoint, and shoot.
2. Follow paths 2 and 3, hold path 3's endpoint, and shoot.
3. Follow paths 4 and 5, hold path 5's endpoint, and shoot.
4. Follow paths 6 and 7, then stop.

The intake runs at existing `IntakeConstants.INTAKE_VELOCITY` throughout active travel AND shooting. The flywheel spins and the turret auto-aims throughout. The blocker stays closed outside the three shooting windows. INIT does not spin up, intake or aim; the turret must be straight at INIT because the existing turret constructor establishes its home there.

The route uses the latest user-supplied coordinates exactly, with the original headings. No mirroring, axis swapping or scaling is applied. Geometry lives in `Autonomous/Paths/AutoPathRoutes.java`; routine in `Autonomous/Autos/AutoPath.java`.

## Shooting behavior

After each designated path, hold the endpoint. Require robot position/heading and motion tolerances, a valid aim solution that is not waiting at a turret travel end/swinging around, turret on-target, and flywheel speed readiness. These autonomous readiness checks operate even though the match TeleOp gate-enable flags are currently false; global flags are unchanged.

The flywheel gate uses an initial ±50 ticks/s entry band, ±100 ticks/s hold band, a dwell, and a 50 ms minimum-open allowance for speed drops. A normal shot dip can pause feeding and trigger another readiness dwell. Aim/motion faults can close immediately. The blocker only opens while the shot window is FIRING. Timings are loop-based, not sensor-confirmed ball release.

Default: **1.5 cumulative seconds commanded open**, within a **4-second wall-time window**, at each of paths 1/3/5. This is a configurable timed burst, not proof four balls fired. If readiness never arrives or recovery consumes the timeout, close and continue the route; count the shot timeout in telemetry. A timing fault aborts the routine. No ball-count sensor is added, so continuous intake cannot itself enforce the four-piece control limit.

## Settings

`Constants/AutoPathConstants` is Dashboard-configurable. Settings are snapshotted during INIT; change values and re-INIT for the next run.

| Setting | Initial value |
| --- | --- |
| REVERSE_STRAFE | true (auto only; re-INIT to apply) |
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

## Current coordinates — restored user route

Coordinates are inches, headings degrees. Curve controls shape the curve; they are not stops.

| Path | Endpoint x, y | Heading | Curve control x, y |
| --- | --- | --- | --- |
| Start | 31.407, 8.249 | 90 | — |
| 1 | 43.249, 12.038 | 90 | — |
| 2 | 8.925, 10.254 | 270 | 23.484, 45.586 |
| 3 | 54.286, 123.819 | 300 | 31.695, 122.240 |
| 4 | 59.067, 32.010 | 270 | 65.091, 118.077 |
| 5 | 58.346, 121.887 | 270 | — |
| 6 | 9.888, 58.637 | 270 | 33.501, 103.337 |
| 7 | 11.428, 94.993 | 270 | — |

Paths 2/3/4 interpolate the original endpoint headings; paths 1/5/6/7 hold constant headings. Shots remain after 1/3/5, with continuous intake. Point 6 is a partner-push approach, not a pickup; the pending change to pass through it continuously has not been implemented. The routine still has separate drive commands for paths 6 and 7.

The previous west-side clearance/range revision is superseded. Its footprint/range PASS does not apply to this route. `tools/check_auto_path.py` retains those conservative constraints and currently fails for the restored route; do not interpret a successful build as field clearance or shot calibration validation. In particular, point 2 at heading 270 places the 14.134-inch deployed front reach beyond y=0, and the shooting positions need shorter-range calibration. Shots 3 and 5 are both on the north side; actual hive state remains unobserved.

## Reported physical horizontal reversal

No alliance-dependent path transform is present. Path 1 requests delta X=+11.842, delta Y=+3.789. With the chassis staged at heading 90 (front toward field +Y), its lateral component is to the robot's right.

Before another autonomous run, use pose telemetry with drive outputs disabled to check manual movement: moving toward field +X must increase reported X; moving toward +Y must increase Y. Then verify heading 90 matches physical staging and positive heading is counterclockwise. If lateral odometry is reversed, inspect Pinpoint pod assignment/directions and mounting. If pose signs are correct but commanded lateral motion is reversed, inspect mecanum motor mapping/directions and drive conventions. The physical cause is not yet established. Code review found matching TeleOp/auto motor names and directions, direct Pinpoint pose passthrough and no path mirroring. Both pod directions are configured FORWARD, but their physical signs cannot be confirmed from code.

At user request, `AutoMecanum` now reverses the body-frame strafe component before wheel normalization. `AutoPathConstants.REVERSE_STRAFE` defaults true and is captured at INIT; set false and re-INIT to undo. Forward and yaw terms are preserved, and Pedro uses the same corrected wheel mixer for saturation calculations. There are no extra per-loop allocations beyond Pedro's existing wheel array. The 12 coordinates and headings stay unchanged. TeleOp, global motor directions and Pinpoint directions stay unchanged. This corrects an output-side lateral reversal only: it does not repair incorrect odometry or a mis-staged chassis heading. It reverses chassis-sideways motion, not field X at every heading. INIT telemetry displays the selected setting.

## Integration changes

`Shooter.updateAutonomous(boolean)` reuses the match aiming/flywheel calculations without reading gamepads. TeleOp `update()` keeps its button behavior. `isAimSolutionUsable()` exposes the travel-route check for autonomous feed readiness. `GeneralVeloMotor` has a device-injection constructor so auto uses monitored hardware with the existing intake controller. The single hardware-free `AthletechRobot` satisfies NextFTC's scanner without another hardware owner. This SDK/Ivy routine controls its own lifecycle; NextFTC hooks do not advance it.

## Verification

The previous 70-test Sim result covers mechanism logic, not physical drive-axis correctness. Current coordinate restoration is checked against actual Pedro path geometry for the exact supplied endpoints/controls, continuity and finite samples, plus Android debug assembly. The conservative footprint/range checker fails for this restored route as described above. No robot deployment or physical validation performed.

Manual strafe correction: Android debug assembly passes; 10,000 mixed-command algebra checks confirm forward/yaw preservation and strafe inversion before saturation. Actual motor response and localization signs remain unverified.
