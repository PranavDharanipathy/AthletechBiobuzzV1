# Autonomous code review

Reviewed main commit `d8649e7`. This is an evidence review, not physical validation. No robot-control code or tuning was changed in this review.

## Sources checked

- AGENTS.md, AGENTOPS.md, CLAUDE.md, DEV_NOTES.md, autonomous README and docs/AUTO_PATH.md.
- All files under Autonomous: AutoPath, AutoShotWindow, AutoMecanum, AutoPathRoutes and package documentation.
- AthletechRobot, all four Systems/Mechanisms adapters, Shooter, Goal, FieldConstants, DrivetrainConstants, DeviceConstants, AutoPathConstants, ShotGate, Blocker, flywheel/turret integration and TeleOp drive mixing.
- Read-only robot_profile.db parameters AND their source records, rather than filling unknown entries.
- Installed NextFTC v2 robot 0.3.0 source jar: NextOpMode, OpModeHook, Mechanism and registration scanner. Installed Pedro revhub 3.0.2 source: PinpointLocalizer and mecanum mixing. Actual Pedro core 3.0.2 bytecode and executable stop probe.
- SHOOTER_DESIGN.md describes proposed architecture as well as implementation. Its claim that code tracks the upward cell is not implemented by the current Goal enum; current executable code takes precedence.

## Findings, in priority order

### 1. Exception cleanup is not guaranteed (high)

Locations: `Autonomous/Autos/AutoPath.java:179`, `Systems/AthletechRobot.java:74`, `Systems/Mechanisms/ShooterMechanism.java:65`.

AutoPath only calls its stop method from periodic/end. In the installed NextFTC source, BoundNextOpMode calls end only on its normal execution path. Its finally invokes hook.afterEnd, but RobotHook does not invoke robot.stopAll there. An exception in a mechanism, command, periodic or start therefore bypasses custom output cleanup. If the constructor throws, the wrapper's opMode reference may still be null, so even hooks cannot clean up that partially constructed robot.

Also, stopAll is sequential without independent cleanup attempts: failure closing the blocker can skip intake, shooter and drive shutdown; failure resetting the flywheel can skip turret shutdown. AutoPath sets stopped before these operations, so later stop attempts return immediately.

Recommended implementation: add an application-owned afterEnd cleanup hook, make constructor/init failure clean up already-created devices, and attempt each output independently. The normal SDK shutdown may still disable hardware; this finding is about the missing application-level guarantee, not a claim that hardware runs indefinitely.

### 2. DriveMechanism.stop does not immediately send zero (high)

Location: `Systems/Mechanisms/DriveMechanism.java:69`.

Pedro 3.0.2 Follower.stop clears its state and selects IDLE; it does not call drivetrain.stop. Only a subsequent follower.update sends that stop. Verified with the actual installed core jar and proxy localizer/drivetrain objects: after manual drive + update, Follower.stop caused zero stop writes; the next update caused one.

A normal AutoPath deadline stop is followed by the robot hook, whose drive periodic provides that update. But end/exception cleanup cannot rely on another periodic occurring.

Recommended implementation: stop the follower state and explicitly stop its public drivetrain during shutdown. Do not run an extra localization/controller update just to obtain a zero write.

### 3. All three shot stations clamp to the same table endpoint (high for scoring)

Locations: `Systems/Shooter.java:190`, `Constants/ShooterConstants.java:40`, `Autonomous/Autos/AutoPath.java:125`.

Using the exact route, current goal coordinates and current turret offset, with zero robot velocity:

| After path | Selected goal | Turret-to-goal distance | Table result |
| --- | --- | --- | --- |
| 1 | South/FAR | 49.660 in | 1300 ticks/s |
| 3 | North/CLOSE | 39.015 in | 1300 ticks/s |
| 5 | North/CLOSE | 36.948 in | 1300 ticks/s |

The table begins at 51.715033 in. Shooter.lookUp deliberately holds the first value below that distance; it does not extrapolate. isAimSolutionUsable checks finite target and turret routing, but not table-domain validity, so readiness can permit feeding at these clamped targets. No repository evidence establishes the correct shorter-range speed.

Recommended implementation: retain the requested coordinates and obtain validated short-range table points, or explicitly reject out-of-domain shots until those points exist. Do not invent RPM or replace empirical data with an unverified physics estimate. Table units are encoder ticks/s, not wheel RPM.

### 4. The requested partner push-through is still absent (high for route intent)

Location: `Autonomous/Autos/AutoPath.java:77`.

Paths 6 and 7 are separate commands. Path 6 must satisfy its endpoint condition at SIX, and holdEnd is true. There is no continuous compound path, partner-contact speed constraint, partner staging input or park confirmation.

Moreover, with the supplied quadratic path-6 control and path-7 line, the tangent direction changes approximately 154.58 degrees at SIX. Merely joining the command list would not make that geometry smooth. Preserve the coordinates, and explicitly design the velocity transition before claiming this is a continuous contact maneuver. At heading 270, northward travel leads with the robot's rear under the documented chassis convention. Partner dimensions, waiting position and contact alignment are not established in the reviewed files.

### 5. Fresh localization faults can reach controllers before AutoPath checks them (medium)

Locations: `Autonomous/Autos/AutoPath.java:159`, `Systems/Mechanisms/DriveMechanism.java:37`.

Verified framework order: AutoPath.periodic -> AthletechRobot.periodic -> drive/intake/blocker/shooter periodic -> Ivy scheduler. AutoPath checks the preceding pose/velocity. The new localizer state is then read and used inside follower.update, before a new application-level finite-state check. The shooter uses that new pose as well. Scheduler shot checks use the current pose, so this is specifically a controller/fault-order gap, not a claim all readiness checks are stale.

Recommended implementation: validate newly acquired localization at the controller boundary before output calculation, using the installed Localizer/Drivetrain interfaces. Avoid adding a second Pinpoint/follower update or duplicate scheduler loop.

### 6. The manual reversal remains an unverified output override (physical cause unresolved)

Location: `Autonomous/AutoSubsystems/AutoMecanum.java:19`.

The override algebraically changes body strafe s to -s before normalization, preserving forward and yaw. maxScaling uses the same virtual mixer, so saturation calculations also see that change. It does not mirror field X at every heading and does not correct Pinpoint signs, swapped pods, wheel placement, motor wiring or physical start-heading mismatch.

Motor names/directions match TeleOp, but that alone does not establish equivalent movement: TeleOp's positive joystick strafe/turn and Pedro's body-left/CCW convention have opposite signs by design. Opposite-looking equations are not evidence of a bug. The repository contains no reviewed measurement proving that the manual reversal is the required correction. It remains enabled because the user explicitly requested it; this review does not silently undo it or certify it.

Use the existing Pinpoint direction/offset tuning procedures and recorded motor configuration to establish signs. Do not change encoder and drive signs together merely to make estimated motion look plausible.

### 7. Prior footprint validation used an unsupported rear reference (medium)

Location: `tools/AutoPathGeometryCheck.java:14`.

The checker sets rear reach to START.y = 8.249 in, inferring wall contact from a route coordinate. This was not a measured robot dimension. The database source for robot.length_m instead says: CAD back plate +171 mm to bash bar -247 mm, total 418 mm, intake-down reach -359 mm. This provides a CAD rear extent of 6.732 in, conditional on the CAD origin matching the Pedro pose reference. DEV_NOTES explicitly marks that origin match as an assumption. Neither the route coordinate nor the CAD note proves physical staging.

FieldConstants records side reach 194 mm and bash-bar reach 247 mm; deployed intake reach is 359 mm. At the restored point 2 and heading 270, that deployed front extent reaches y=-3.880 in under the documented coordinate reference. This is conditional geometry, not a measurement of the currently deployed mechanism or proof of physical contact.

The checker also hard-requires south/north/south shots, whereas the restored route is south/north/north, and uses a conservative central review envelope rather than exact field solids. Its old PASS cannot certify this restored route or game-rule compliance. Keep these diagnostics explicitly separated from actual field validation.

## Confirmed behavior and limits

- Registration is consistent with the installed scanner: public AutoPath(AthletechRobot), @NextAutonomous and one hardware-free NextRobot constructor. package-info.java files are documentation only.
- NextFTC owns the scheduler loop. Drive localization precedes shooter update and scheduler readiness checks. There is no reason to add another Scheduler.execute or follower.update in AutoPath.
- Mechanisms reuse the current flywheel and turret constants/classes. Auto readiness timing/bands are separate experiment settings, not automatically tuned by the turret/flywheel tuners.
- Intake target is 2300 ticks/s during active travel and shots, from IntakeConstants. Blocker is a positional servo (0.17 CLEAR / 0.365 BLOCK, reversed direction); turret uses two CR servos. This is what the code controls, not confirmation the devices work physically.
- AutoShotWindow accounts for elapsed intervals previously commanded open and faults on invalid/long timing. It has no ball sensor or servo-position feedback. The 1.5-second feed budget is not a ball count.
- Auto deliberately requires low robot speed before feeding (3 in/s, 10 deg/s). All time-of-flight table values and turret lead time are currently zero. This auto is not configured for genuine shoot-on-the-move compensation.
- Goal selection uses robot Y relative to 72, not a hive-tip sensor. Paths 3 and 5 both select the north goal. No code here establishes which cell is physically up after a burst.
- Robot profile drive acceleration, deceleration, force/current limit and several other entries are NULL. Their absence is not permission to substitute the motor free-speed or a plausible-looking contact speed.

## Verification performed

- Re-ran all seven AutoShotWindow tests: pass.
- Ran the actual Pedro core stop probe described in finding 2: behavior reproduced.
- Checked shot ranges and path-6/7 tangent directions against the exact stored points and current constants.
- Latest main merge Android assembly passed. Full suite result at that merge was 70 pass, 1 skipped, 1 turret-hold failure; that failure was reproduced identically on pre-merge main 7c2fc70. The helper tests do not execute the Android/NextFTC lifecycle or validate motor response.
- No deployment, on-robot run, motor-direction change, control change, commit or push performed as part of this review.
