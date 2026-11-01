# Checkpoint for Andrew, Codex and Claude

2026-10-31. Andrew requested a checkpoint before a larger issue is discussed.
The broad robot audit is unfinished. This is a handoff, not competition readiness.

Remote sync after checkpoint: merged `origin/main` at `a183f8a` into
`shooter-turret-v1` (merge `8c35060`), retaining the staged audit work. Resolved the
`TeamCode/build.gradle` conflict by keeping JUnit/test configuration and the remote's
Slothboard Dashboard dependency setup. Clean Android build plus 54 Sim and 20
TeamCode tests pass. A backup stash of the pre-pull checkpoint remains retained.

## Repository and working state

- Actual repo: `/Users/andrewxnchen/Documents/GitHub/AthletechBiobuzzV1`.
  The supplied `Documents/athletech` workspace is empty.
- Changes are staged locally, uncommitted. No deployment or physical robot operation.
- Read `AGENTS.md`, `AGENTOPS.md`, `DEV_NOTES.md`, then `docs/CODE_AUDIT.md`.
  October 31 hardware notes supersede the October 21 shooter design assumptions.

## Documented robot understanding

- Mecanum drive, intake and blocker; four game pieces held.
- Pollen-only shooter: 72 mm wheels against a 2.72 inch gap; nectar does not fit.
  One goBILDA 6000 rpm bare motor, 1:1 flywheel belt.
- Two Axon Max CR turret servos; net servo-to-turret drive is **1:1**, despite the
  older design's 3:1 assumption. Separate quadrature encoder is geared **3:1**:
  documented scale is 8192*3/360 = 68.2667 ticks/degree.
- No turret hard stop. Wiring-safe travel and reliable homing are unknown.
- Hood pivots about the flywheel axle and uses **two mirrored servos**. Estimated
  launch elevation is 31–54 degrees; these are not measured servo endpoints.
- Limelight is chassis-mounted, pitched 50 degrees, so it does not rotate with the
  turret. Useful visibility/localization performance remains unverified.
- These facts come from saved CAD/team notes, not new physical measurements.

## Most important unresolved gaps

1. There are two shooter implementations: baseline `Systems/*` (team-selected)
   and older `shooter/*` (ADRC/observer/ballistics). They are not reconciled.
   Legacy green tests assume different hood/turret behavior and cannot establish
   current hardware performance.
2. The newly pulled baseline `HoodAngler` now maps `lhood` and `rhood` with opposite
   directions. This resolves the previous one-servo software gap; actual hardware
   configuration and linkage endpoints still need physical verification.
3. Baseline turret scale remains 65.738 ticks/degree; travel limiting and homing
   need hardware evidence. Baseline turret gains are zero placeholders.
4. This checkout has an abstract TeleOp base and diagnostic/tuning OpModes, no
   integrated competition TeleOp or autonomous source directory.
5. Inventory sensing, feed gating during hive tips, measured shot tables, dynamics
   and fault recovery on the real robot remain unverified.
6. Full source review is incomplete. In particular, database internals, prediction
   algorithms, import edge cases, remaining simulator dynamics and OpModes still
   need review. Do not treat the test count as coverage of all source.

## Changes made during the audit

- Fixed integral reset history; removed boxed previous-sample allocation.
- Hardened voltage compensation against nonfinite commands, invalid clocks and
  rejected sensor samples.
- Fixed baseline flywheel zero-integral arithmetic, stale gain selection, immediate
  reset stop, signed readiness and manual-command handling. Fixed turret zero-gain
  bounds and zero-decay-at-target arithmetic. Added direct baseline tests.
- Fixed retry after partial logger file-open failure. Bounded each pending telemetry
  queue at 4096 rows and report dropped rows. Finalization stops row admission.
  Deleting sessions while the old writer finishes is still a review item.
- Added wrapper/cache tests for cycle refresh, write invalidation, unit conversion,
  age/runtime changes and IMU yaw reset.
- Sim now requires explicit `launcher.pollen.enabled` and `launcher.nectar.enabled`.
  Unsupported types need no launcher fit, are not simulated, and have stale simulated
  outputs cleared subject to source priorities. Historical measured curves remain;
  downstream consumers must honor the flags.
- Shared profile has manual pollen=1/nectar=0, 72 mm flywheel, one motor, 1:1 gear,
  6000 rpm free speed, estimated 31–54 degree launch range. Other missing physics
  were not filled with examples. Catalog and PROFILE.md were synchronized.
- Projectile RK4 now reuses per-flight buffers and avoids per-step temporary vectors
  unless recording a trace. Rejects invalid timestep/ball inputs and stops exactly
  at the requested time horizon.

## Verified checkpoint

`./gradlew :Sim:build :TeamCode:testDebugUnitTest :TeamCode:assembleDebug` succeeds.
Current XML reports: **54 Sim tests and 20 TeamCode tests, zero failures/errors/skips**.
The legacy desktop suite last passed 34 checks separately via `bash tools/run_tests.sh`.

Projectile allocation test on this JDK: before, 35,448 bytes/flight at 10 ms steps
and 347,808 at 1 ms; after, 712 bytes/flight for both. This is allocation evidence,
not a robot timing guarantee. Existing analytic flight tests and trace consistency pass.

The affected launcher scenario was rerun on `Sim/out/audit_pollen_only.db`; report
is `Sim/out/audit_pollen_only/launcher/report.html`. All its outputs are placeholder
because it uses example inertia, ball mass, battery and losses. Its range and recovery
numbers must not be used as real robot performance.

Known build tooling issues remain: JDK/Gradle deprecation warnings and previously
observed Kotlin lint-analyzer metadata diagnostics plus 18 lint warnings. No claim
of clean lint or physical validation is made.
