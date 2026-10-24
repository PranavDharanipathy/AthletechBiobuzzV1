# Sim: robot physics sims + the robot profile database

A plain Java desktop module (no phone, no robot). It does two jobs:

1. **Simulates the robot**: motors, battery sag, mecanum drive dynamics, the flywheel launcher, ball
   flight with drag and spin, the turret, and odometry drift. It runs prediction algorithms on top: the
   shooting table (hood + rpm per distance), Monte Carlo make probabilities, and stop-to-stop move times.
   Two "undeterminables" get their own sims:
   - **Loop timing** (`sim loop`): what each loop spends on your code, every hardware read and write, and
     random jitter/spikes, and what slow or uneven loops cost - turn and move settle time, overshoot,
     flywheel speed at the feed, odometry integration error, how old the pose is when it is used, how late
     time-based actions end, and which short sensor pulses (beam breaks, limit switches) get missed.
   - **Static electricity and disconnects** (`sim esd`): charge on the robot frame, its electronics, the
     field perimeter, metal structures and the tiles, driven by rolling, sliding, wheel slip (pushing and
     full-power launches, so current draw feeds static), intaking and hits, leaking faster the more humid it
     is, with every coulomb accounted for; every discharge (perimeter metal, other robots, inside the robot); plus
     brownouts, current surges and drops, impacts and motor noise from a full physics match - all turned
     into Control Hub reboots, Expansion Hub drops, I2C lockups, camera drops and Wi-Fi drops with
     outage times. Swept over humidity with and without a grounding strap, and over a competition day.
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
| 2. Real-robot logs | `profile import-monitor C:\path\to\HardwareMonitor` | battery resistance + sag curve, loop time and what it is made of (logic, each kind of read/write, jitter, spikes), current draw, top speed |
| 3. Shot / pose logs | `profile import-shots shots.csv`, `profile import-poses poses.csv` | measured make %, rpm and aim error, drift (`*.measured_*` curves, kept next to the sim's own) |
| 3b. Disconnect log | `profile import-disconnects disconnects.csv` | real disconnect rate, outage times, rate vs humidity |
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
./gradlew :Sim:run --args="predict loop --sensor-reads 3 --motor-writes 6"
./gradlew :Sim:run --args="predict disconnect --humidity 20 --strap 0"
```

## Code map (`org.athletech.sim`)

| package | what |
|---|---|
| `config` | `Catalog` (every entry, the single source of truth), `ProfileDb` (SQLite), `ProfileSchema`, `RobotParams` (profile to models), export/report |
| `physics` | `MotorSpec`/`DcMotor` (linear DC motor), `Battery` (Voc - I*R sag), `Projectile` (RK4 flight, drag, Magnus) |
| `robot` | `MecanumDrive`, `Flywheel`, `Launcher`, `Turret` |
| `field` | `Aperture` (scoring openings at any orientation) |
| `undeterminables` | `Distribution` (error models), `OdometryErrorModel` (drift), `LoopTimingModel` (loop = reads + code + writes + jitter), `RobotCharge` / `TileChargeGrid` / `ChargeSim` / `Electrostatics` (static charge), `Disconnects` (stress to failures) |
| `predict` | `AimSolver` (shooting table), `ShotPredictor` (Monte Carlo), `DriveTimePredictor` (bang-bang move times), `LoopControlSim` (controllers run once per loop) |
| `robot` (also) | `MatchDriver`: a full physics match (driving, hits, pushing, flywheel) for the electrical and static sims |
| `scenarios` | `DriveScenario`, `LauncherScenario`, `LocalizationScenario`, `LoopTimingScenario`, `EsdScenario`: run the sims, save curves, write reports |
| `input` | `MonitorImport` (HardwareMonitor sessions), `ObservationImport` (shot/pose CSVs) |
| `input` (also) | `ObservationImport.importDisconnects` (disconnect log) |
| `visualization`, `telemetry`, `utility` | SVG/HTML reports, time-series recorder, CSV/JSON/stats helpers |

To add a new piece of information, add it to `Catalog.java` (follow the `category.name_unit` naming),
then run `profile init` (adds the empty row, keeps every value) and `profile docs` (refreshes PROFILE.md).

## Model limits

These are simplifications. Check them against real data before relying on the numbers.

- **Wheel slip.** Quasi-static: when a motor pushes harder than traction holds, its wheel instantly spins
  (or skids) to the speed where the motor's torque equals the traction limit, and draws that speed's current.
  Wheel and gearbox inertia and the difference between static and sliding friction are ignored.
- **Braking.** Stop-to-stop times brake by reversing full power, which draws about twice stall current
  per motor. Set `drive.current_limit_a` if your code limits it. Real path followers are slower than this,
  so scale with `drive.path_overhead_factor`.
- **Shot contact.** The ball leaves at `exit_speed_ratio` x wheel surface speed and takes
  `energy_loss_factor` x its kinetic energy out of the flywheel. Both are fitted numbers, so measure exit
  speed with slow-motion video.
- **Loop timing.** Turns and moves use plain PD controllers on IMU/odometry readings; your code may differ
  (motion profiles, feedforward). Without your gains (`control.heading_kp`/`kd`, `control.drive_kp`/`kd`)
  the sim tunes some on a fast loop, so its numbers show the trend, not your exact robot. The hub's own
  velocity-loop rate (`launcher.hub_velocity_hz`) is not documented; set what you measure.
- **Static charge.** A lumped model: each robot is two conductors (frame, electronics), the field is a
  perimeter conductor, a metal-structure conductor and a grid of tile charge. Every coefficient (charging
  per meter, leakage at 40% RH, humidity sensitivity, how sensitive each device is) is a placeholder until
  measured: a field meter (frame voltage after driving, decay time) and a disconnect log are the way in.
  Robots start every match at 0 V, the other three robots charge like this one (with spread), and touching
  metal always brings two conductors to the same voltage. Contact charging from game elements and plastic
  walls adds charge to the robot; its opposite charge leaves with the element or stays on the wall (not
  tracked further). Within the model the books balance to rounding: each match checks that the robot's and
  the whole field's charge equal what came in minus what drained (`sim esd` prints the error).
- **Electrical stress.** Brownouts come from the battery's resistance (no wiring inductance: at
  millisecond scale its voltage spikes are tiny). How current surges, drops and impacts cause disconnects
  is an empirical "50% point" per device, to be fitted from your disconnect log.
- **Goal geometry.** The HIVE CELL is a rectangular opening. Its height and tilt are not in the manual
  text, so measure them. A ball scores when its center goes through the opening at least one ball radius
  from every edge. Bounce-outs are not modelled; log them as observations.
