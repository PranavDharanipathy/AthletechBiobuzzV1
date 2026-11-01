# Robot code audit

Started 2026-10-31. This is an ongoing audit, not a declaration of robot readiness.

## Evidence and robot understanding

The workspace initially supplied (`Documents/athletech`) is empty. The robot repository
is `Documents/GitHub/AthletechBiobuzzV1`. Its worktree was clean at audit start.
Read AGENTS.md, AGENTOPS.md, CLAUDE.md, DEV_NOTES.md and SHOOTER_DESIGN.md.
The October 31 hardware notes supersede the October 21 shooter design assumptions.

Current documented hardware: mecanum drivetrain, intake and blocker, one 6000 rpm
flywheel motor with 1:1 belt and 72 mm wheels, two continuous-mode Axon turret servos
with net 1:1 transmission, motor-port quadrature turret feedback geared 3:1, and two
mirrored positional hood servos. Shooter gap makes this a pollen-only shooter.
Hood launch elevation is estimated at 31–54 degrees, not calibrated. Limelight is
chassis mounted, pitched 50 degrees; its visibility constraints remain unresolved.
Cable travel limits and turret homing remain unverified.

Official rules checked against FIRST's current TU07 manual (October 29):
https://ftc-resources.firstinspires.org/ftc/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU07.htm

G407 limits simultaneous control to four pieces; G408 prohibits control of opponent
nectar. G417 permits manipulating hive motion only by launching into the upward-facing
cell. Section 10.5.1 warns that firing while the hive tips can impede the tip. These
require consideration in inventory control, targeting and feed permission.

## Findings from source inspection

| Area | Evidence | Consequence / remaining work |
| --- | --- | --- |
| Hood | `Systems/Hood/HoodAngler.java` maps only `hood`; `DeviceConstants` has one hood name | Baseline does not implement documented mirrored pair. Confirm second device mapping before integration. |
| Turret calibration | `ShooterConstants.TURRET_TICKS_PER_DEGREE` is 65.738; current notes specify 8192*3/360 | Reconcile calibration and dependent gains against actual hardware. |
| Turret travel | `TurretBase.setPosition` accepts arbitrary targets | Declared angle limits elsewhere do not enforce cable safety here; establish homing and validated wiring range. |
| Turret arithmetic | `TurretBase.update` divided integral bounds by zero and could evaluate 0/0 for zero hold decay at rest | Reproduced with direct baseline-controller tests; explicit zero-gain reset and zero-width feedforward handling now keep commands finite. Negative integral gains now order bounds correctly. |
| Flywheel sensing | Baseline flywheel differences encoder positions per loop | At 28 counts/revolution, quantization can dominate short-loop velocity; compare hub velocity / estimator with measured logs. |
| Controller reset | `DynamicTrapezoidalSum.setSum` returned early when sum already equaled requested value | Reproduced stale sample adding 6 units immediately after reset. Fixed reset to always clear history; raw anti-windup setter retains history intentionally. Four Java 8 regression checks cover both behaviors. |
| Drive mixing | `RobotCentricDrive` sends unnormalized sums | Individual clipping changes requested drive ratios at combined inputs; add coherent normalization and verify signs separately. |
| Integration | Baseline uses `Systems/*`; second implementation lives under `shooter/*` | Passing legacy shooter tests does not validate baseline controllers. Reconcile intentionally rather than swapping controllers blindly. |
| Legacy model | `ShooterConfig` hardware names differ from `DeviceConstants`; design uses 3:1 turret and steep launch arcs | Legacy verification uses assumptions inconsistent with current hardware. Preserve as algorithm evidence only. |
| Runtime completeness | `Systems/Drivetrain/Drivetrain.java` is empty; base TeleOp is abstract | Trace actual annotated OpModes before claiming competition integration or autonomous readiness. |
| Voltage fault containment | Shared `VoltageCompensator` allowed NaN power, poisoned its sampling clock on NaN time, and used rejected reads to shorten the next filter interval | Fixed nonfinite command stop, clock rejection, configuration validation and filter elapsed time. Original implementation fails the new fault regression. |

## Follow-up inspection

- `TeleOp` contains only the abstract base. There is no `Autonomous` source directory
  in this checkout. `Testing` provides individual hardware and drivetrain OpModes.
- Read the full intake sweep geometry chain (`FIArm`, `SectorPrism`, `GJK`, vectors,
  `Obstacle`, `Footprint`, `Prism`, `Extension`, `FieldCollisionMap`). It is geometric
  logic, not a mechanism actuator controller. Constructors commonly accept NaN
  dimensions; actual field configuration and invocation still need tracing. GJK's
  iteration limit is deliberately conservative (blocked), but broad geometry tests
  are still needed before trusting collision clearance.
- Read `GeneralVeloMotor`, `Encoder`, `InterpolationData`, turret gain interpolation
  and the baseline controller constants. Baseline turret coefficients start at zero;
  lookup tables and gains remain tuning inputs rather than measured evidence.
- No GitHub Actions workflows exist in `.github` in this checkout.

## Baseline controller regression work

Added JUnit 4 to TeamCode tests and exercised actual `Systems/Flywheel/Flywheel`
and `Systems/Turret/TurretBase` classes with SDK-interface proxies. No Android
return-default-value setting or physical device is used. The turret has an injected
device constructor; existing hardware-map constructors retain their device names,
directions and encoder setup.

Five failing baseline flywheel tests reproduced and drove fixes:

- Zero integral gain with zero integral bounds produced NaN, suppressing otherwise
  valid proportional/feedforward motor output. Disabled integral now clears its
  history and contributes exactly zero. Negative-gain bounds are ordered correctly.
- Gain selection used the preceding velocity sample. It now follows the current read.
- `reset()` left the motor powered until a subsequent update. It now stops immediately
  and clears reported output/velocity/derivative state.
- Absolute velocity allowed wrong-direction spin to be reported ready. Readiness now
  compares signed velocity and rejects invalid tolerances and manual mode.
- Idle/unobserved wheels could report ready. A nonzero target and a sample are required.

Two failing turret tests reproduced the same zero-gain/zero-bound issue and the
zero-decay-at-target NaN. Both are fixed. A further failing flywheel test reproduced
nonfinite direct manual motor output; manual commands now stop on nonfinite input,
clamp finite power to [-1,1], and update the reported power.

The legacy desktop shooter tests use JDK management APIs unavailable in android.jar;
TeamCode's Android unit-test source set explicitly excludes that package. It remains
covered by `bash tools/run_tests.sh`. Utility main-based regression suites are also
invoked by JUnit so the normal Gradle test task covers their checks.

The baseline integral accumulator used a boxed `Double` for its previous sample,
creating a potential per-update allocation. Replaced it with a primitive and an
initialization flag; existing reset and history regressions verify behavior.

Read monitoring infrastructure `CacheSlot`, `DeviceTracker`, `DeviceStats`,
`RunningStats`, `Histogram`, and `EventBuffer`. Remaining checks include cache
ownership/concurrency at lifecycle transitions, histogram configuration/nonfinite
inputs, and event-buffer ownership while stopping the writer. These files have
been inspected, but their system-level behavior is not yet verified by this audit.

## Storage and monitor lifecycle review

Read all of `HardwareMonitor`, `MonitorStore`, `MonitorWebServer`,
`MonitorOpModeHook`, and `MotorElectricalModel`, including sampling, summary
generation and session startup/shutdown paths.

Reproduced two storage defects with deterministic writer scheduling and temporary
files, then fixed them:

- If opening `system.csv` failed after `devices.csv` opened, the partially assigned
  fields made later flushes skip initialization. Open now publishes the writer set
  only after all files succeed and closes partial resources on failure. A test
  blocks `system.csv` with a directory, removes it, and successfully retries.
- Device, motor and system row queues were unbounded. Each is now capped at 4,096
  pending rows and uses non-waiting offers. Excess rows increment
  `monitor.queuedRowsDropped` in the summary and appear in dashboard findings.
  Admission is synchronized with `finish()` so no producer can enqueue after the
  final flush has been scheduled. Tests overload all three queues and explicitly
  delay finalization to check this race window.

The row cap bounds pending row count, not all session storage. Event logging has
its existing separate byte cap; system/motor logs can still grow throughout long
practices. File-system stalls can still delay the writer indefinitely; the OpMode
does not wait for queue capacity. Deletion protection while an old session is
finishing and stale hardware-sample handling remain review items.

Verification after these changes: `:TeamCode:testDebugUnitTest` has 15 passing
JUnit tests (including 3 storage tests), zero failures/errors; `:TeamCode:assembleDebug`
succeeded. Dashboard inline scripts pass `node --check`. The changed finding has
not yet been exercised in a browser with a real dropped-row session.

## Verification captured so far

- `bash tools/run_tests.sh`: 34 checks, zero failures. Tests exercise the older
  pure-Java shooter and its plant models, not the baseline `Systems` controllers.
  Updated its compiler gate from release 11 to release 8 after independently
  confirming every core and test source compiles with Java 8.
- `./gradlew :Sim:test :TeamCode:assembleDebug`: Sim XML reports show 47 tests,
  zero failures/errors across five suites. Android build succeeded (58 tasks).
- `bash tools/run_utility_tests.sh`: focused Java 8 integration-reset regressions.
- Voltage fault/recovery regressions run in the same utility script, with no SDK
  stubs or external dependencies required for the fixed production class.
- After baseline controller fixes and primitive accumulator change,
  `./gradlew :TeamCode:testDebugUnitTest :TeamCode:assembleDebug` succeeded.
  Current XML reports: 8 flywheel tests, 2 turret tests, 2 utility-suite tests;
  zero failures/errors. These verify arithmetic and state transitions with fake
  device feedback, not physical settling time, encoder polarity or shot accuracy.
- After voltage changes, `:TeamCode:assembleDebug :TeamCode:lintDebug` succeeded.
  Lint reports 18 warnings: 12 native 16KB alignment, 3 locale, 2 unused resources,
  1 static-field leak. Its embedded Kotlin analyzer also emits metadata-version
  diagnostics (expected 2.2 vs stdlib 2.4); this is not a clean lint result despite
  the successful task exit. Investigate the toolchain separately.
- No robot deployment or physical mechanism operation performed.

## Hardware wrappers and launcher simulation capabilities

Read every `Monitored*` wrapper and its base class. Added five tests against real
wrapper/cache code and fake SDK devices: same-loop reuse, cycle refresh even with
monitoring disabled, invalidation after encoder reset and motor power writes,
current-unit conversion without another transaction, max-age/runtime cache changes,
and same-loop IMU yaw-reset invalidation. All pass. Shared mutable cached SDK
objects and multi-threaded reads still need an explicit ownership policy.

The simulator previously required fits for both ball types and always produced
both sets of shot predictions. Added required binary capability inputs
`launcher.pollen.enabled` / `launcher.nectar.enabled`. Launcher construction,
launcher runs, loop timing's selected shot type, and profile completeness reports
now honor those capabilities. Disabled types are not simulated; prior simulated
derived results are emptied subject to normal source-priority protection. Measured
curves remain stored; downstream consumers must check the capability flags.

Updated Catalog, synchronized the tracked database through `profile init`, and
regenerated PROFILE.md. Set the shared profile to pollen=1, nectar=0 with source
`manual`, based on the October 31 hardware notes. Also entered documented 72 mm
flywheel, one motor, 1:1 gearing, 6000 rpm motor free speed and estimated 31–54° hood
launch range as manual inputs. Hood bounds are geometric estimates, not measured
servo limits; no robot servo range or direction was edited. Missing dynamics,
ball masses, uncertainty distributions and calibration remain unfilled.

Three new Sim regressions cover pollen-only construction without nectar inputs,
missing/nonbinary capability rejection, and a saved launcher scenario that clears
stale simulated nectar output while retaining measured curves. Current Sim suite:
50 tests, zero failures/errors. TeamCode suite: 20 JUnit tests, zero failures/errors.

Ran a persistent scratch scenario with the estimated hood range and pollen-only
capability: `sim launcher --save --trials 10 --db out/audit_pollen_only.db
--out out/audit_pollen_only`. It completed and wrote a pollen table with no nectar
table. All outputs are explicitly placeholder: inertia, losses, battery, ball mass
and other physics still use example inputs. Those simulated ranges/recovery times
are not evidence of this robot's performance. Report content and placeholder warning
were inspected at `Sim/out/audit_pollen_only/launcher/report.html`.

## Projectile integrator checkpoint

Read `MotorSpec`, `DcMotor`, `Battery`, `Projectile`, `Aperture`,
`ObservationImport`, and the analytic physics tests. Battery convergence under
extreme loads and import validation remain review items, not verified fixes.

Reproduced per-timestep projectile allocation growth (35,448 vs 347,808 bytes per
flight at 10 ms vs 1 ms steps), timeout overshoot (25 ms request ran 30 ms), and
invalid flight inputs being accepted. Reused per-flight RK4 buffers, delayed
temporary vector creation until needed, validated ball/timing inputs and clamped
the last timestep to the requested horizon. The new allocation test measures 712
bytes per flight at both timesteps; trace/no-trace solutions agree.

Final checkpoint command `:Sim:build :TeamCode:testDebugUnitTest :TeamCode:assembleDebug`
succeeded. XML reports: 54 Sim tests and 20 TeamCode tests, no failures/errors/skips.
The affected pollen-only scratch launcher run completed and remains explicitly
placeholder. Andrew requested a checkpoint before discussing a larger issue;
`docs/CHECKPOINT.md` is the concise cross-agent handoff.

## Remaining scope

Review all team-owned source, simulation physics/prediction/profile handling,
monitoring/storage/web dashboard, collision geometry, utility helpers, tuning and
test OpModes, build configuration and CI. Reproduce defects before fixes, add
meaningful regression coverage, and record remaining measured hardware inputs.
Compilation and simulated success cannot establish physical reliability or optimality.
