# Pollen shot experiment — servo-free, fixed geometry

This version of **Pollen Shot Experiment** (group **testing**) does not initialize, require, or command hood, blocker, or turret servos. The hood angle and turret aim are fixed manually. Only the flywheel and intake/feed motors operate; drive motors are kept at zero. The existing turret encoder is monitored read-only for motion. Match TeleOp and other servo subsystems are unchanged.

## Physical setup

Secure the hood and turret mechanically so they cannot move during shots. Measure the actual fixed exit angle. Position the blocker mechanically clear of the ball path if needed. Verify that the stopped intake actually retains the ball: **without a working blocker, software cannot stop gravity-fed release**. If balls roll into the flywheel with the intake stopped, this experiment cannot control their release; correct that arrangement before loading shots. Begin with one pollen and observe whether each motor pulse releases exactly one ball.

Keep the angle fixed while tuning RPM and shooting distance. To compare two angles, stop the OpMode, manually reposition/secure the hood, remeasure the angle, and start another batch. There is no angle adjustment through Dashboard. No servo calibration, direction, or endpoint changes are needed.

## Controls and configuration

1. Start stationary facing the upward cell, initially about 6 ft from **ball release to mouth center**. Aim manually; no automatic turret control or active holding is provided.
2. Select **Pollen Shot Experiment**, START, and set Dashboard fields:
   - `MEASURED_LAUNCH_DEG`: actual fixed angle, 31–54° for the current CAD reference model.
   - `RELEASE_DISTANCE_FT`: measured horizontal release-to-mouth distance, 4–12 ft.
   - `FLYWHEEL_TICKS_PER_REV`: verified encoder counts per flywheel revolution. Default 28 is an assumption for the 1:1 bare motor; check actual hardware/sign.
   - `TARGET_RPM`: chosen experimental speed, 500–5000; default 0 blocks operation.
   - `FEED_POWER`: verified positive intake motor power; default 0 blocks operation.
   - `HARDWARE_CONFIRMED=true`: confirms fixed geometry, aim, motor/encoder direction and scale, and that intake alone controls release.
3. Release **RB**, then press **X** to apply a frozen settings snapshot. X performs no servo movement. Dashboard edits take effect only on another X press with RB released and the preceding attempt labeled.
4. Hold **RB** to spin up. Readiness requires RPM within ±75 RPM and acceleration ≤400 RPM/s continuously for 0.35 s, valid voltage, and a stationary robot/turret. Starting motion limits are drive ≤20 ticks/s and turret ≤100 ticks/s; verify against actual encoder noise/gearing. Encoders do not detect every chassis slip, and manual aim is not verified by software.
5. When **READY**, press **A** freshly. Intake starts immediately for nominal `FEED_SECONDS=0.12`; there is no blocker-open stage or hood-settling delay. Early/held presses are never queued. An attempt is counted when the intake pulse starts, not when a sensor detects a ball.
6. Releasing RB or pressing **B** stops motor outputs. Motion, invalid readings, low voltage, or a loop gap >0.1 s inhibits/cancels feed. A normal RPM dip during the pulse is expected; afterward speed must recover with at least 0.8 s shot spacing. Pulse duration is loop-timed: it can overshoot by a loop interval, or longer during an actual stall. Stopping intake is not a physical ball gate and cannot recall a released ball.
7. Label each completed/canceled attempt: **D-pad UP** clean entry/stayed, **DOWN** clean entry/exited, **LEFT** miss, **RIGHT** rim contact. **Y** marks INVALID/no ball/multiple balls. A new attempt waits for the previous label. Use RIM even if the ball eventually stays, keeping retention statistics limited to clean entries.
8. Release RB before adjusting RPM. Compare 10 shots per setting at the same fixed angle and distance. Tune from a known working RPM in small increments; preserve high-frame-rate side video. Repeat with several marked balls. If the mechanical angle is changed, start a new measured batch.
9. STOP normally to save `/sdcard/FIRST/data/ShotExperiments/pollen-*.csv`. Pull via `adb pull /sdcard/FIRST/data/ShotExperiments`, then run `python3 tools/summarize_pollen_shots.py path/to/session.csv`. The summary separates clean-entry rate from retention after clean entry and excludes invalid/unlabeled attempts. Old servo-version CSV files remain readable.

The bounded CSV holds up to 6000 rows in RAM and saves after motor outputs stop, avoiding disk work in the control loop. It logs approximately 10 Hz normally and up to 50 Hz during feed/recovery, plus state/label changes. At capacity arming stops; restart for another session. A crash/power loss loses this CSV. Logs include fixed angle/range, applied RPM/feed parameters, measured RPM/voltage/motion, batch, attempts and labels; no hood servo position or servo timing is recorded.

Overspeed >5500 RPM, negative RPM below -150, invalid voltage, or failure to obtain stable speed within 5 s latches a motor fault; release RB to clear and inspect. Flywheel control uses nominal 6000-RPM-at-12-V feedforward plus bounded PI, a 90% duty cap, and a power slew limit. It is an experiment controller, not a characterized motor model. An unreachable target remains blocked; do not bypass readiness by arbitrarily widening thresholds.

## Physics checks for this change

1. **Fixed geometry:** removing servo control does not change the ballistic equations. Angle is a measured fixed input, not an actuator request. RPM/distance are the available software experiment variables. At fixed angle and distance, arbitrarily lowering RPM can miss low; the prior higher-angle/lower-speed comparison only applies if the angle is physically changed and secured. Release-height CAD inference is valid only for the documented hood geometry, not an arbitrary rebuilt undershooter.
2. **Release physics:** no blocker means no guaranteed retention or one-ball indexing. The sequencer now goes READY directly to FEEDING. Expected RPM dip continues through a pulse; interlocks still cancel it. Neither stopping the motor nor a software readiness check can prevent uncommanded gravity release. Attempt counting and CSV now track commanded intake pulses.
3. **Real errors:** retained RPM dwell, acceleration, recovery, voltage, motion and loop checks. `python3 tools/pollen_shot_envelope.py` tests 128 hypothetical corners per 6/8-ft, 50/54° candidate: ±2° direction, ±5% ball speed, ±5 cm range, ±0.5 in release height, ±2.5 cm lateral aim, Cd 0.3–0.6, Cl 0–0.2, plus radius/rim allowances. None clears every corner. These are assumed bounds, not measured failure rates; the model excludes interior collisions/retention. The script's angles are hypothetical comparisons, never servo commands.

Vacuum reference at 6 ft: 39.4° needs 9.88 m/s, 50° 6.47 m/s, 54° 6.08 m/s. Single wheel against stationary hood ideally gives ball speed = half wheel surface speed; actual slip needs measurement. Illustrative aero estimates near 3900/3700 RPM at 6 ft for 50/54° assume a 0.45 ball/wheel speed ratio and unmeasured drag/lift. They are search references, not universal commands. One wheel cannot independently set spin and translation. Software can compensate repeatable flight errors; clean-entry rebound still needs video diagnosis.

## Validation and references

17 focused tests cover direct motor-only feed timing, no queued shots, interlocks, expected RPM dip/recovery, unit conversions and motor limits. Android assembly and TeamCode unit tests pass. No physical shot validation or deployment has been performed.

- FIRST TU07 dimensions/geometry: https://ftc-resources.firstinspires.org/ftc/game/cm-html
- AndyMark nominal ball properties: https://andymark.com/products/biobuzz-scoring-elements?variant=47194281148588
- NASA lift/model limitations: https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ideal-lift-of-a-spinning-ball/
- Hardware geometry: DEV_NOTES.md, 2026-10-31 CAD/team notes.
