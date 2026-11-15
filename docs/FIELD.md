# BIOBUZZ field: every measurement we have

From the Competition Manual (TU09) section 9 text and figures, read 2026-11-14. Section 9.1 says the field CAD is the
official version, and measurements taken from it carry ±1 in. The figures are nominal, also ±1 in. Imperial units
govern.
- Field CAD (Onshape): https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad, which redirects to
  document `a355e772e3d24813de7852ee`, workspace `f106353168f1f92100b81259`, element `95d1e1e442b4138cccaf2d73`. It needs
  an Onshape login, so nothing here was read from it yet.
- Re-extract the manual's figures: `python -I tools/manual_figures.py MANUAL.pdf` (they land in the git-ignored
  `tuning-data/manual-figures/`).
- Code that uses these numbers: `tools/field_geometry.py` (the CELL model), `Constants/FieldConstants.java` (goal aim
  points), `Sim` (`Aperture`, profile `target.*`).

## Frame

Pedro inches:
- (0, 0) is the audience-left corner.
- x points toward blue, y away from the audience, z up from the tiles.
- Heading 0 = +x, counterclockwise.

The red alliance area is behind x = 0 and blue's behind x = 144.

## Field (9.2)

- 144 × 144 in, 36 tiles of about 24 × 24 × 0.59 in.
- Contents: one HIVE Structure (a frame holding a red and a blue HIVE) and 4 FLOWERS.
- **Loading zone** (section 9 field figures): a taped box against the alliance wall, 11 in deep and about 23 in wide (width set by tile
  seams). In code: red at (15, 108, 0), blue at (129, 36, 180).
- **GARDEN:** two strips of 1 in tape about 23 in long along a wall, set by tile seams, in opposite corners.
  Red is along the audience wall from the audience-left corner; blue is the opposite corner.
- **Alliance area:** about 97 in wide by 54 in deep, outside the field wall.

## HIVE Structure (9.6)

- **Frame** (Fig 9-8):
  - 49.46 in wide at its widest, 38.95 in deep at the base.
  - The pivots' axis is **43.95 in** above the tiles, at the top of the frame.
- **HIVEs** (Fig 9-9, 9-10):
  - Red and blue HIVE centers are **25.5 in** apart, red toward x = 0. In code: x = 59.25 red, 84.75 blue.
  - Each HIVE is two CELLs on an arm through the pivot. The arm runs along y; the inner ends are **18.84 in** apart,
    each CELL is **12.04 in** deep, and it's **42.91 in** end to end.
  - It's bistable and rests tipped **30°**, one CELL up and one down. Launching into the up CELL is the only legal way to
    tip it (G417).
  - Bottom of the HIVE is 30.6 in above the tiles. The up CELL's opening runs from **53.5 in** at its bottom to
    **65.6 in** at its top.
- **The CELL** (Fig 9-11, 9-16):
  - **The opening is the CELL's outer end:** the end away from the pivot, facing the half of the field that CELL sits
    over. The inner end is the back wall.
  - **The opening is a pentagon, house shaped:** a flat bottom **20 in** wide, straight sides up to **7.61 in**, then
    a roof meeting at a peak **14 in** above the bottom, in the middle. These are inside dimensions. The corners are
    rounded about 1.5 in.
  - **Tipped up,** that face leans back 30° (its normal points 30° above horizontal, outward), with the peak toward the
    pivot.
    - So the opening spans 14 × cos 30° = 12.12 in of height (53.5 to 65.6 ✓) and 14 × sin 30° = 7 in of y.
    - **The aim point** is the top middle of the pentagon's rectangle, **7.61 in up the face** (just under where the roof
      starts), z ≈ 60.1 in. In code (`FieldConstants` goals) it's at y = 87.47 (CLOSE CELL, opens toward +y) or 56.53 (FAR,
      opens toward −y). Until 2026-11-14 it was the middle of the 14 in (7 in up, y 87.77 / 56.23); the CELL itself didn't move.
  - **AprilTags:** a cluster of 4 on the CELL's bottom (outside) face, facing the tiles. 3.25 in tags, 36h11. IDs:
    - red: 30–33 opposite the audience, 34–37 on the audience side;
    - blue: 38–41 on the audience side, 42–45 opposite.
    - Tag centers 2.75 in from the cluster centerline; the 0.5 in reference holes are 9.938 in from the CELL's front
      (Fig 9-15).
- **Rules (G417):**
  - Launching into the outside bottom, sides or top of a CELL is a foul if done on purpose.
  - A missed launch that hits them is not.
  - Launching at a tipping HIVE's down CELL can stop the tip (10.5.1).

## FLOWER (its section 9 figure)

- 21.5 in tall to the top ring.
- Top ring opening 4.0 in across; bottom ring inside diameter 2.79 in, 0.43 in thick.
- Retrieval opening 3.55 in tall.
- Base 3.57 in.
- From the center: 2.40 in to one side and 1.89 in to the other.
- Faces in code: Audience-Side y 3.8 (x 94.475); Scoring-Side y 137.7 (x 46.75); Red Alliance x 3.8 (y 47); Blue Alliance
  x 137.7 (y 94.75).

## Game pieces (9.8)

POLLEN about 2.8 in. NECTAR about 3.6 in.

## What the pentagon does to a shot (`tools/field_geometry.py`)

- **The model:** a drag-free arc at `SHOT_LAUNCH_ANGLE_DEGREES` (72) from `SHOT_RISE_INCHES` (48.58) below the aim point
  (exit 11.5 in up). It aims at the aim point from the turret. `Systems/Flywheel/CellGeometry` is the same model in Java (the
  Shooter runs it every loop for the feed gate's window; `FlywheelBurstTest` checks the two agree).
- **What it reports:** the range of speeds, as a ratio of the speed that hits the aim point, for which the ball crosses the
  tilted face with its whole 2.8 in inside the pentagon (center 1.4 in up to 12.3 in up straight on). Examples at the red
  CLOSE CELL:

| Turret at | Away | Off straight-on | Still goes in |
|---|---|---|---|
| (59.25, 130) | 42 in | 0° | 4.9% slow to 3.6% fast |
| (40, 125) | 42 in | 27° | 5.1% slow to 3.2% fast |
| (30, 110) | 37 in | 52° | 6.0% slow to 3.1% fast |
| (20, 100) | 41 in | 72° | 7.0% slow to 3.1% fast |
| (59.25, 110) | 22 in | 0° | 3.5% slow to 2.9% fast |

- **Why not the middle:** from an angle a fast ball crosses off-center under the sloped roof (below), so the room above the
  aim shrinks while the room below grows. Aiming at the top of the rectangle keeps the roof out of a centered shot; the
  old 2028-05-15 video showed the misses were mostly balls clipping the top and bouncing out.

- **Why the fast side shrinks from an angle:**
  - The aim is at the center. A faster ball crosses the tilted face higher up, and so further in (the face leans back),
    and so further along its own sideways path.
  - It ends up off-center, where the roof is lower than the 14 in peak, so a speed that goes in under the peak straight
    on hits the roof or goes over from the side.
  - A slower ball crosses lower and off-center the other way, where the sides are still a full 7.61 in, so the slow side
    grows.
  - So from an angle the best speed is a bit *below* the straight-on speed for the same distance: about −1% at 53°
    (≈15 ticks/s) and −2% at 73° (≈25 ticks/s).
- **Close in** (under about 20 in in front), a 72° arc can't get in at all.

### How the flywheel fit uses it

- Each ball gets its own window, worked out from its turret position, the CELL it aimed at, and its approach angle
  (`shots.csv`: `approach`, `slow_room`, `fast_room`).
- The fit's table is the **aim-point** speed (the top middle of the rectangle). A ball counts as:
  - short below center × (1 − slow room);
  - long above center × (1 + fast room).
- **Synthetic test** (true center speed = table + 20):
  - the shape-aware fit found +28, 80% range +19..+37;
  - the old fixed ±12 ticks/s window found +2, confidently wrong.

### Not known yet (check before trusting the numbers)

- **Launch angle and exit height.** 72° and 11.5 in are in `ShooterConstants`, but not measured. The window scales with
  them: steeper arcs come down more vertically, which changes how much a speed error moves the crossing.
- **Exit gain** (the window in wheel speed over the window in exit speed; 1 = exit speed proportional to wheel speed).
  - 72° with gain ~0.45, or ~68° with gain 1, both fit the table's slope.
  - The 2028-05-15 video fits 0.35-1 about equally: its balls sat near the table's speed, so they only pin the fast edge.
  - So `fit` keeps the whole range and picks the table that scores across it.
  - The SHT probes (`SHTStabilityTuner.PROBE`) measure it.
- **Not modeled:** drag, spin, and the other HIVE, the frame or the arm being in the way.
