# BIOBUZZ autos

Six NextFTC autos, group **A_Match**:

| Auto | OpMode | What it does |
| --- | --- | --- |
| Red Park / Blue Park | `RedPark`, `BluePark` | Leaves the wall and parks in the LOADING ZONE. |
| Red Preload + Park / Blue Preload + Park (1 tip) | `RedPreloadPark`, `BluePreloadPark` | Starts exactly where 2 Tip Garden does (in line with the upward CELL), shoots the 4 preload POLLEN into it (the same tip 1, `rampUp()` sets the flywheel for the distance), then curves round the HIVE to park in the LOADING ZONE. |
| Red 2 Tip Garden + Park / Blue 2 Tip Garden + Park | `RedTwoTipGarden`, `BlueTwoTipGarden` | Starts against the audience wall (red) / far wall (blue) straight in line with the upward CELL and shoots the preload into it (tip 1). Takes the GARDEN's POLLEN with the intake only (FIArm stays up, no FLOWERS), then shoots them from the LOADING ZONE into the CELL that came up (tip 2), parked. |

The autos are command based (Ivy on NextFTC). Each auto builds its paths from its own `Constants/Autos/<Auto>Constants`,
then lists one sequence of robot commands. For example, `RedPreloadPark`:

```java
routine = robot.autonomous(
        robot.shooter.rampUp(),
        robot.shoot(start),
        robot.drive.follow(Drive.curve(start, PARK_CONTROLS, park))
);
```

What each command does:

- **`drive.follow(path)`:** Ivy's `PedroCommands.follow`, finished once the robot is really at the end (tolerance). A
  timeout faults the auto.
- **`robot.shoot(pose)`:** holds the pose and waits for the flywheel to be up to speed (within `FLYWHEEL_ERROR_TICKS_PER_SECOND`
  of its target, or feeds anyway after `SHOT_TIMEOUT_SECONDS`), then opens the blocker and runs the intake for
  `SHOT_FEED_SECONDS` in one go. No per-ball gating.
- **End of the routine:** everything stops, and the turret goes back home and holds there (TeleOp takes its INIT position as home).
- **`robot.collect()` / `stow()`:** arm down and intake on / arm up and intake off (no auto uses them now; 2 Tip Garden uses `intake.run()`/`stop()` with the arm up).
- **`robot.autonomous(...)`:** runs the steps in order, stopped by the time limit or a fault (loop timing, localization, a
  path timeout), then makes everything safe.

Commands live on the mechanisms (`Autonomous/AutoSubsystems`) and on `AthletechRobot`, and nothing is shared between autos.

## Rules they're built on (Competition Manual TU08)

- **Points:**
  - LEAVE is 3 (AUTO only): no longer touching the perimeter wall.
  - PARK is 5: at least partly in your LOADING ZONE.
  - Both are judged at the end of AUTO.
  - A HIVE TIP is 20, and tips finished before TELEOP count as AUTO.
- **Start (G304):**
  - on your own side (red x < 72, blue x > 72);
  - touching the wall;
  - not in the LOADING ZONE or touching a FLOWER;
  - touching exactly 4 preload POLLEN;
  - motionless after INIT.
- **HIVE:**
  - At setup each alliance's upward CELL holds 3 NECTAR: red's is the FAR one, blue's the CLOSE one.
  - It tips once "enough" is launched in. The manual gives no number, so whether 4 POLLEN is enough is unknown until it's tried on a real HIVE.
- **CELL choice:** `Shooter` picks the CELL by the robot's half (y > 72 = CLOSE). The Preload autos shoot from the start, which must be on the upward CELL's half. A pose on the wrong half throws at INIT.
- **Taking POLLEN:**
  - GARDENS can be emptied by either alliance.
  - FLOWERS only from their bottom opening (G418).

## Coordinates

Everything uses the turret's frame (`FieldConstants`): Pedro inches, (0, 0) in the audience-left corner (red GARDEN), x toward blue, y away from the audience. The Pedro visualizer's **BIOBUZZ field** uses the same frame, so its points go in unchanged. No mirroring.

| | Red | Blue |
| --- | --- | --- |
| Park start | (9.72, 64, 0°), back on the red wall | (134.28, 78, 180°), back on the blue wall |
| Preload (1 tip) start | (59.25, 9.72, 90°), the 2 Tip Garden start | (84.75, 134.28, −90°), the 2 Tip Garden start |
| Preload (1 tip): to LZ controls | (23, 34), (23, 50); clears the HIVE by 3.9 in | (130, 104), (114, 92); 3.9 in either way it turns |
| Park (LOADING ZONE) | (15, 108, 0°), red LZ x 0–11, y 96–120 | (129, 36, 180°), blue LZ x 133–144, y 24–48 |
| 2 Tip Garden start | (59.25, 9.72, 90°), back on the audience wall under the FAR CELL | (84.75, 134.28, −90°), back on the far wall over the CLOSE CELL |
| 2 Tip Garden: to GARDEN | (8.64, 22, −90°), controls (34, 34), (34, 39) | (135.36, 122, 90°), controls (110, 110), (110, 105) |
| 2 Tip Garden: sweep end | (8.64, 10.72, −90°), front 1 in off the wall | (135.36, 133.28, 90°) |
| 2 Tip Garden: to LZ controls | (26, 30), (26, 90) | (115, 114), (115, 54) |

The robot outline (front 9.72 in, side 7.64 in) was checked along every path against the walls, the FLOWERS and the HIVE frames from the visualizer (2 Tip Garden: against the whole HIVE with its CELLS, x 44–72 / 72–100, y 46–98; closest 3.75 in, the wall just after a start turn). Blue isn't an exact rotation of red, because the blue wall FLOWER sits 2 in lower, so each side is tuned on its own.

## Tuning

- **Per-auto constants:** `Constants/Autos/<Auto>Constants`, for example `RedPreloadParkConstants` (Dashboard, read at INIT).
  Each one holds that auto's poses and its own timing:
  - `AUTO_LIMIT_SECONDS`, path timeout, loop limit;
  - position and heading tolerance;
  - for the shooting autos: shot feed seconds, timeout, ready dwell, flywheel band, still-robot limits;

  Tuning one auto never moves another.
- **Visualizer projects:** `docs/autos/*.pp`. Open them in the Pedro visualizer with File → Open from computer, using the BIOBUZZ field. Keep them in step with the constants.

## Check on the field before matches

- Does the intake take the GARDEN POLLEN with the FIArm up? 2 Tip Garden drives front-first to 1 in off the wall over x 1–16 of the strip (the POLLEN sit about x 0–11).
- Does one 4-POLLEN volley tip the CELL?
- Is `SHOT_FEED_SECONDS` (1.5 s) long enough to empty 4?
- Turret straight at INIT, as for every shooting OpMode.
- Not yet run on the robot.
