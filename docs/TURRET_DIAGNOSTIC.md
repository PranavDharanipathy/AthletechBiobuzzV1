# Turret diagnostic: procedure and how to read the data

OpMode **Turret Diagnostic** (group `diagnostics`), source
`TeamCode/.../Testing/TurretDiagnostics/`. It **collects data and diagnoses**: it does not replace or
tune the controller, and it changes no robot constants.

**Problem it targets:** the turret shakes and overshoots while the robot drives quickly. Candidate
causes are friction, mechanical play, inertia, long or variable loops, and filtering delay. Each
run below isolates one of them.

> Not yet run on the robot. Unit tests and a build pass, but neither proves the hardware behaves.
> Software travel bounds are only as good as the bounds you enter, the assumed deceleration and
> the encoder. **They do not guarantee wiring safety.** One person holds the deadman; a second
> watches the turret with a hand on the Driver Station stop.

## Before the first active run (robot disabled)

Set these in Dashboard (`TurretDiagnosticConfig`). Each starts as NaN, and arming is refused until
it is set and confirmed:

| Field | How to choose it |
|---|---|
| `TICKS_PER_DEGREE` | Measure it in passive mode (step 1). Documented theory is 68.2667; baseline `ShooterConstants` still has 65.738. The tool never writes it back. |
| `MIN_DEG`, `MAX_DEG` | Rotate the turret by hand from the reference until the cable just becomes taut, each way. Then enter something **smaller** than that. There is no hard stop. |
| `STOP_MARGIN_DEG` | Room kept between the predicted stopping point and the bound. Start generous (e.g. 15–20°). |
| `MAX_COMMAND` | Clamp on every servo command. Start at 0.1–0.15. |
| `ASSUMED_DECEL_DEG_PER_S2` | Low = conservative. After step 3, replace it with the *smallest* measured `coast_decel_deg_s2`. |
| `MAX_SPEED_DEG_PER_S` | Overspeed trip. Set it just above the speeds the planned levels should produce. |

Gamepad 1:
- `LB + A` confirms the settings.
- `LB + X` confirms the starting reference, with the turret physically at your reference mark.
- `LB + Y` arms. Arming only works after START, with the deadman released.
- Hold `RB` (the deadman) to run. Releasing it stops and disarms.
- `B` or `BACK` cancels.
- dpad left/right selects the mode; dpad up/down selects the plan.

Any Dashboard edit after confirmation cancels the confirmation and stops an active run.

## Procedure (in this order)

1. **Passive: encoder scale.** Press `X` to set a marker, rotate the turret by hand exactly 90° or
   180° using physical marks, and read the implied ticks/degree. Repeat both ways. If it doesn't
   match 68.27 or 65.74, stop and find out why before trusting any degree value. Then confirm
   `TICKS_PER_DEGREE`.

2. **Open loop, polarity and servo fight.**
   - Run the schedule with `ACTUATORS = LEFT_ONLY`, then `RIGHT_ONLY`, then `BOTH`, at low levels.
     Use a single level such as `{0.06}` and `INCLUDE_HOLDS = true`.
   - In `*_steps.csv`, `polarity_agrees` must be 1. If it is 0, the encoder and servo directions
     disagree, and the baseline controller would push the turret away from its target. Do not run
     CONTROLLER until that is resolved; the tool does not change directions.
   - Compare `late_speed_deg_s` across the three runs. `BOTH` should be at least as fast as either
     servo alone. If `BOTH` is slower, or one servo alone moves the turret the opposite way, the two
     servos are fighting (direction or trim mismatch). That alone can cause shake.

3. **Open loop, friction, play and inertia.** Run the full schedule with `ACTUATORS = BOTH`.
   Levels ascend, and each level is pulsed `+, +, -, -` with neutral in between, then held `+, -`.

4. **Controller, baseline behavior and loop sensitivity.**
   - CONTROLLER mode runs the unmodified `TurretBase` through the guard and records `ctrl_p/i/d/f`,
     the gains it selected, its own `ctrl_dt_ms`, and its output.
   - Pick the gains with `GAIN_SOURCE`: `CONSTANTS` is what the TeleOp base uses (currently all
     zero placeholders, so it commands 0); `TUNER` uses the `TurretBaseTuner` Dashboard values.
   - Repeat the target schedule with `EXTRA_LOOP_DELAY_MS` = 0, 20, 40 (optionally
     `EXTRA_LOOP_JITTER_MS`).
   - The `hold reference` plan holds 0° indefinitely, so the hold can be watched while the turret
     is disturbed by hand.

## Which evidence points to which cause

| Cause | Evidence (file / column) |
|---|---|
| Static friction | Lowest level with `moved = 1`, per direction; `dead_time_s` on a same-direction restart. |
| Play / backlash | `dead_time_s` on a reversal (`reversal = 1`) minus a same-direction restart at the same level. The servo-side gear train is not visible to the encoder (which meshes the 72T directly), so play shows **only** as this extra delay. |
| Viscous / speed-dependent loss | `late_speed_deg_s` vs command on holds: the slope and the zero-speed intercept. |
| Inertia vs. drag | `coast_disp_deg` and `coast_decel_deg_s2` after neutral. Large coast with low decel means inertia dominates; quick stops mean friction dominates. |
| Loop timing | `loop_dt_ms`, `read_us`, `write_us`, `hwmon_loop_ms` in every file. In the controller runs, check whether overshoot and oscillation grow with `extra_delay_ms`. |
| Filter / measurement delay | `vel_hub_ticks_s` (the hub's windowed velocity) against `vel_deg_s` (raw difference, no filter). The lag between them is the hub velocity delay. Baseline `TurretBase` uses raw position, plus hub velocity in telemetry. |
| Controller structure | `ctrl_p/i/d/f` per loop: which term dominates during overshoot; integral wind-up through the target; `saturated = 1` time; derivative spikes at long `ctrl_dt_ms`. |

These are single-run measurements. Repeat each test (at least 3 runs, both directions, two battery
levels) before drawing conclusions.

## Files

Data is written to `/sdcard/FIRST/data/TurretDiagnostics/<yyyyMMdd_HHmmss>/`. Get it with
`adb pull /sdcard/FIRST/data/TurretDiagnostics`.

- `NNN_passive.csv`, `NNN_controller.csv`, `NNN_open_loop.csv`: one file per segment.
  - The activity is in the file name, in the `# activity=` header, and in the `activity` column of
    every row (0 passive, 1 controller, 2 open loop).
  - The headers also carry the confirmed settings, reference, scales, configured directions and
    controller gains.
  - While disarmed, rows are always `passive`, whatever mode is selected.
- `NNN_open_loop_steps.csv`: one row per pulse or hold, with the measures above.
- `events.csv`: every confirmation, arm, run start, stop and fault, with its reason.

## Known limits

- **No chassis driving in this OpMode.** `DrivetrainConstants` and `TeleOpBaseOpMode` disagree on
  drive motor directions, and that can't be resolved from the code. The shake-while-driving case
  therefore isn't reproduced here. The recorder, supervisor and guard are separate classes, so they
  can be added to a TeleOp once the drive directions are verified.
- Bounds, margin, deceleration and command limits are operator inputs. The stopping prediction
  uses 2× the worst recent loop interval plus braking at the assumed deceleration. A slower real
  deceleration, encoder slip or a mis-entered bound defeats it.
- Detection thresholds (`MAX_LOOP_GAP_MS`, `IMPLAUSIBLE_SPEED_DEG_PER_S`, frozen-encoder and
  command-opposes-motion windows) are heuristics, not physical limits.
- Baseline `TurretBase` notes:
  - On its first update, `prevError` is 0 and `dt` is measured from construction, so any
    nonzero initial error produces a derivative kick. The diagnostic holds the start position
    first to avoid this.
  - Its hold-decay uses `ShooterConstants.TURRET_TICKS_PER_DEGREE` (65.738) internally,
    whatever scale is confirmed here.
  - `TurretBase.reverse()` mutates the static gain tables; the diagnostic never calls it.
