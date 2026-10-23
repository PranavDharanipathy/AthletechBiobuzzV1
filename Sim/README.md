# Sim: robot physics sims + the robot profile database

A plain Java desktop module (no phone, no robot). It does two jobs:

1. **Simulates the robot**: motors, battery sag, mecanum drive dynamics, the flywheel launcher, ball
   flight with drag and spin, the turret, and odometry drift. It runs prediction algorithms on top: the
   shooting table (hood + rpm per distance), Monte Carlo make probabilities, and stop-to-stop move times.
2. **Keeps one database a game simulator reads**, `data/robot_profile.db` (SQLite). It holds every number
   that describes how this robot plays: speed and acceleration, how they change with battery and current
   draw, shot accuracy, localization drift, and action times. [PROFILE.md](PROFILE.md) is the data
   dictionary. The file starts **empty** apart from values copied from the BIOBUZZ game manual.

Everything runs through Gradle from the repo root (PowerShell needs the `./`):

```bash
./gradlew :Sim:run --args="help"
./gradlew :Sim:test
```

## Filling the profile

| step | command | fills |
|---|---|---|
| 1. Robot build | `profile set robot.mass_kg 13.4` (and the rest from `profile report`) | inputs: mass, wheels, motors, launcher, ... |
| 2. Real-robot logs | `profile import-monitor C:\path\to\HardwareMonitor` | battery resistance + sag curve, loop time, current draw, top speed |
| 3. Shot / pose logs | `profile import-shots shots.csv`, `profile import-poses poses.csv` | measured make %, rpm and aim error, drift (`*.measured_*` curves, kept next to the sim's own) |
| 4. Sims | `sim all --save` | every derived curve: move times, battery effect, shooting table, make % |
| 5. Check | `profile report --html` | completeness + every curve charted (`out/profile_report.html`) |

- Get the HardwareMonitor folder with `adb pull /sdcard/FIRST/data/HardwareMonitor` (one sub-folder per
  session; `import-monitor` also accepts a single session folder).
- Templates for the shot and pose logs are in `data/templates/`.
- Error sizes go in with `profile set-dist`, e.g. `profile set-dist launcher.rpm_error normal 0 35`.
- Action times go in with `profile set-action`, e.g. `profile set-action intake.nectar 0.9 0.3 0.85`.
- `profile clear <name>` empties any entry again (parameter, curve, surface, distribution or action).
- Every value keeps its source. Priority is `measured` = `game_manual` > `manual` > `sim` > `placeholder`.
  A lower-priority write never replaces a higher one unless you pass `--force`.

## Trying it before you have data

```bash
./gradlew :Sim:run --args="profile init --example --db out/example_profile.db"
./gradlew :Sim:run --args="sim all --save --db out/example_profile.db"
```

`--example` fills empty inputs with generic FTC numbers marked `placeholder`. Anything the sims compute
from them is also `placeholder`, so it can never pass for real data. Reports go to `out/` (git-ignored).

## Quick predictions

```bash
./gradlew :Sim:run --args="predict shot --distance 2.2 --ball nectar --battery 12.2"
./gradlew :Sim:run --args="predict drive --distance 1.5 --battery 12.2"
./gradlew :Sim:run --args="predict turn --angle 90"
```

## Code map (`org.athletech.sim`)

| package | what |
|---|---|
| `config` | `Catalog` (every entry, the single source of truth), `ProfileDb` (SQLite), `ProfileSchema`, `RobotParams` (profile to models), export/report |
| `physics` | `MotorSpec`/`DcMotor` (linear DC motor), `Battery` (Voc - I*R sag), `Projectile` (RK4 flight, drag, Magnus) |
| `robot` | `MecanumDrive`, `Flywheel`, `Launcher`, `Turret` |
| `field` | `Aperture` (scoring openings at any orientation) |
| `undeterminables` | `Distribution` (error models), `OdometryErrorModel` (drift) |
| `predict` | `AimSolver` (shooting table), `ShotPredictor` (Monte Carlo), `DriveTimePredictor` (bang-bang move times) |
| `scenarios` | `DriveScenario`, `LauncherScenario`, `LocalizationScenario`: run the sims, save curves, write reports |
| `input` | `MonitorImport` (HardwareMonitor sessions), `ObservationImport` (shot/pose CSVs) |
| `visualization`, `telemetry`, `util` | SVG/HTML reports, time-series recorder, CSV/JSON/stats helpers |

To add a new piece of information, add it to `Catalog.java` (follow the `category.name_unit` naming),
then run `profile init` (adds the empty row, keeps every value) and `profile docs` (refreshes PROFILE.md).

## Model limits

These are simplifications. Check them against real data before relying on the numbers.

- **Wheel slip.** Wheels never slip in the model. When traction caps a wheel's force, the motor still
  turns at chassis speed, so peak current while traction-limited is overstated.
- **Braking.** Stop-to-stop times brake by reversing full power, which draws about twice stall current
  per motor. Set `drive.current_limit_a` if your code limits it. Real path followers are slower than this,
  so scale with `drive.path_overhead_factor`.
- **Shot contact.** The ball leaves at `exit_speed_ratio` x wheel surface speed and takes
  `energy_loss_factor` x its kinetic energy out of the flywheel. Both are fitted numbers, so measure exit
  speed with slow-motion video.
- **Goal geometry.** The HIVE CELL is a rectangular opening. Its height and tilt are not in the manual
  text, so measure them. A ball scores when its center goes through the opening at least one ball radius
  from every edge. Bounce-outs are not modelled; log them as observations.
