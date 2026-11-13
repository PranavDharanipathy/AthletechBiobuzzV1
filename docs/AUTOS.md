# BIOBUZZ autos

Six NextFTC autos, group **A_Match**:

| Auto | OpMode | What it does |
| --- | --- | --- |
| Red Park / Blue Park | `RedPark`, `BluePark` | Leaves the wall and parks in the LOADING ZONE. |
| Red Preload + Park / Blue Preload + Park (1 tip) | `RedPreloadPark`, `BluePreloadPark` | Starts exactly where 2 Tip Flowers does (in line with the upward CELL), shoots the 4 preload POLLEN into it (the same tip 1, `rampUp()` sets the flywheel for the distance), then curves round the HIVE to park in the LOADING ZONE. |
| Red 2 Tip Flowers + Park / Blue 2 Tip Flowers + Park | `RedTwoTipGarden`, `BlueTwoTipGarden` | Starts against the audience wall (red) / far wall (blue) straight in line with the upward CELL and shoots the preload into it (tip 1). No GARDEN. Takes POLLEN from both FLOWERS on its own half with the dropdown (FIArm), its alliance FLOWER first (red: Red Alliance then Scoring-Side, blue: Blue Alliance then Audience-Side). At each it stops far enough out that the dropdown can swing down without hitting the FLOWER, drops it, creeps in at 6 in/s aiming 3.5 in past the FLOWER's face, stops the moment it stalls and holds there (never pushes), intakes, backs out, raises it. Tip 2 is shot at the side FLOWER: dropdown down at the stage, in to the shot pose 2.5 in closer to the FLOWER to shoot what the alliance FLOWER gave, then sitting in it with the blocker open and the intake on for `FLOWER_SHOOT_SECONDS` (4 s), so every ball that comes in, bounces off the walls included, goes straight out. Then it parks in the LOADING ZONE without shooting. |

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
  `SHOT_FEED_SECONDS` (2 s) in one go, stops the intake, keeps the blocker open `SHOT_SETTLE_SECONDS` (0.15 s) so the last ball
  clears, then closes it before driving off. No per-ball gating.
- **End of the routine:** everything stops, and the turret goes back home and holds there (TeleOp takes its INIT position as home).
- **`drive.approach(path, still, stall, max)`:** for touching things: ends at the path end, when the robot has stopped (it touched),
  or at `max` seconds, never a fault, then holds where it really is so it doesn't keep pushing. 2 Tip uses it for the FLOWER.
- **`robot.collect()` / `stow()`:** arm down and intake on / arm up and intake off (no auto uses them now; 2 Tip Flowers runs the intake from after the first shot and moves the arm itself at each FLOWER).
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
| Preload (1 tip) start | (59.25, 9.72, 90°), the 2 Tip Flowers start | (84.75, 134.28, −90°), the 2 Tip Flowers start |
| Preload (1 tip): to LZ controls | (23, 34), (23, 50); clears the HIVE by 3.9 in | (130, 104), (114, 92); 3.9 in either way it turns |
| Park (LOADING ZONE) | (15, 108, 0°), red LZ x 0–11, y 96–120 | (129, 36, 180°), blue LZ x 133–144, y 24–48 |
| 2 Tip Flowers start | (59.25, 9.72, 90°), back on the audience wall under the FAR CELL | (84.75, 134.28, −90°), back on the far wall over the CLOSE CELL |
| 2 Tip Flowers: alliance FLOWER stage (dropdown goes down here) | Red Alliance FLOWER: (19.47, 47, 180°), controls (48, 24), (24, 40); face x 3.8 + swing 13.77 + 1.9 | Blue Alliance FLOWER: (122.03, 94.75, 0°), controls (100, 120), (108, 112); face x 137.7 − 13.77 − 1.9 |
| 2 Tip Flowers: alliance FLOWER touch | (12.49, 47, 180°): face + roller reach 12.19 − 3.5 | (129.01, 94.75, 0°) |
| 2 Tip Flowers: side FLOWER stage | Scoring-Side FLOWER: (46.75, 122.03, 90°), controls (24, 116), (40, 108) | Audience-Side FLOWER: (94.475, 19.47, −90°), controls (116, 16), (108, 48) |
| 2 Tip Flowers: side FLOWER shot (dropdown already down) | (46.75, 124.51, 90°): roller 1 in short of the face | (94.475, 16.99, −90°) |
| 2 Tip Flowers: side FLOWER touch (sits here shooting) | (46.75, 131.01, 90°): 5.5 in past the face | (94.475, 10.49, −90°) |
| 2 Tip Flowers: back out to each stage, then to LZ controls | (16, 108), (32, 116) | (112, 26), (104, 26) |
| Dropdown (Onshape Main, 2026-11-12) | deployed roller front 12.19 in ahead of the drive center, 2.46 in past the bash bar (9.73), 13.7 in wide, 0.4–1.2 in off the floor; pivot 6.07 in ahead, arm 7.70 in, so it sweeps out to 13.77 in while dropping | |

The robot outline (front 9.72 in, side 7.64 in) was checked along every path against the walls, the FLOWERS and the HIVE frames from the visualizer (2 Tip Flowers: against the whole HIVE with its CELLS, x 44–100, y 46–98; closest 3.04 in red, 3.19 in blue). The robot's turning corners reach 12.36 in, so headings turn along the curves, never on the spot at a wall. Red and blue use different FLOWERS, so each side is tuned on its own.

## Tuning

- **Per-auto constants:** `Constants/Autos/<Auto>Constants`, for example `RedPreloadParkConstants` (Dashboard, read at INIT).
  Each one holds that auto's poses and its own timing:
  - `AUTO_LIMIT_SECONDS`, path timeout, loop limit;
  - position and heading tolerance;
  - for the shooting autos: shot feed seconds, timeout, ready dwell, flywheel band, still-robot limits;

  Tuning one auto never moves another.
- **Visualizer projects:** `docs/autos/*.pp`. Open them in the Pedro visualizer with File → Open from computer, using the BIOBUZZ field. Keep them in step with the constants.

## Check on the field before matches

- Does the dropdown roller take the FLOWER's POLLEN at the touch pose, and does it stop gently (no bending)? If it stops short, move `FLOWER_TOUCH` 0.5 in at a time toward the FLOWER (red: y up, blue: y down); if it pushes, raise `FLOWER_STILL_INCHES_PER_SECOND` so it stalls out sooner.
- Does the dropdown clear each FLOWER as it swings down at the stage (1.9 in to spare)?
- Does one 4-POLLEN volley tip the CELL?
- Is `SHOT_FEED_SECONDS` (2 s) long enough to empty 4, and does the last ball clear before the blocker shuts (`SHOT_SETTLE_SECONDS`)?
- Turret straight at INIT, as for every shooting OpMode.
- Not yet run on the robot.
