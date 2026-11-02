# Pollen shot experiment — automatic flywheel sweep

**Pollen Shot Experiment** (group **testing**) is a hands-off routine. Press INIT, press START,
and it runs by itself until telemetry shows **Done!**. Only the flywheel motor moves (plus the
intake, if shots are turned on). Turret, hood, blocker and drive are not touched or even looked up.

## What it does

1. **Open-loop voltage sweep** (`VOLTAGE_STEPS`, default 1–6 V in 0.5 V steps, then 7–12 V in
   1 V steps). Each step holds a constant voltage (power = volts / battery) until the speed is
   flat: at least `VOLTAGE_MIN_STEP_SECONDS` (0.75 s), then as soon as the fitted RPM slope over the
   last `VOLTAGE_AVERAGE_SECONDS` (0.5 s) is under `VOLTAGE_STEADY_RPM_PER_SECOND` (40 RPM/s).
   It gives up at `VOLTAGE_MAX_STEP_SECONDS` (3.5 s) and marks that step not steady. RPM and
   applied volts are averaged over that last window. Steps can't go
   above `BATTERY_MIN_VOLTS` (12 V): a motor can't be driven above the battery's own voltage, so
   every step stays reachable on any allowed battery. The sweep stops early once the flywheel
   passes the top RPM setpoint (2100), so it never runs faster than the closed-loop range needs. A straight-line fit of applied volts against ticks/s gives
   **kv (V per tick/s)** and **ks (V)**. Those are in the units `Flywheel` uses, so they can go
   straight into `FlywheelConstants`. Each step also reports a 63 % time constant.
2. **Spin down** to below `SPIN_DOWN_RPM`.
3. **Closed-loop RPM sweep** (`RPM_SETPOINTS`, default 500–2100 in 200 RPM steps). For each
   setpoint, it starts from rest using `ExperimentFlywheel`: feedforward from the kv/ks just measured,
   plus PI gains scaled from kv. If the fit fails, the run stops there. It records rise time
   (first time within `RPM_TOLERANCE`), settle time (within tolerance for `STABLE_SECONDS`),
   overshoot, then holds for `HOLD_SECONDS` to record mean error, RPM standard deviation, min/max
   and mean power. A setpoint not settled within `SETTLE_TIMEOUT_SECONDS` is marked not reached.
4. **Optional shots** (`SHOTS_PER_SETPOINT`, default 0). After the hold, it waits for stable
   speed, runs the intake at `FEED_POWER` for `FEED_SECONDS`, and records the RPM dip and the
   time to recover. Load balls first; with the default of 0 no balls are needed and the intake is
   never initialized.

The default run takes about two to three minutes. INIT telemetry shows an estimate for the current settings.

## Safety

- START only runs if the battery reads between `BATTERY_MIN_VOLTS` and `BATTERY_MAX_VOLTS`
  (12–14.5 V); otherwise INIT shows the error and nothing moves.
- Any time: RPM above `MAX_RPM` (2500), the battery sagging below `RUN_MIN_VOLTS` (10.5 V) or
  rising above 14.5 V, or an invalid reading stops all outputs and ends the run early.
- If the flywheel reads below −`SPIN_DOWN_RPM` while driven forward, the run stops and tells you
  to flip `FLYWHEEL_MOTOR_DIRECTION`.
- STOP aborts at any point. Data collected so far is still saved.
- `FLYWHEEL_TICKS_PER_REV` (default 28, bare motor) must match the real encoder or every RPM is
  wrong.

## Output

After the motors stop, it writes two files to `/sdcard/FIRST/data/ShotExperiments/`:

- `flywheel-sweep-<time>-samples.csv`: about 50 Hz raw log (time, dt, phase, setpoint, shot, RPM,
  battery volts, flywheel and intake power), up to 30 000 rows.
- `flywheel-sweep-<time>-summary.csv`: status, kv/ks fit with R², the voltage-step table and the
  setpoint table.

Pull them with `adb pull /sdcard/FIRST/data/ShotExperiments`. The same summary is shown on
telemetry under **Done!** until you press STOP.

All settings are `@Config` fields; change them in FTC Dashboard during INIT.
