# AutoPath

Driver Station: **AutoPath Red**, group **A_Match**. Movement only: no shooting, intake or aiming. The shooter and intake are
built but never started, and the blocker stays shut.

## Route

`Autonomous/Paths/AutoPathRoutes.java` holds the user's visualizer export (2026-11-07), points exactly as given, through
`PoseFactory.degrees().mirrorX(70.75)`: x → 141.5 − x, heading → 180° − heading. The robot drives the mirror image of the
drawing, so path 1 moves it about 12 in to its **left**. Remove `.mirrorX(70.75)` to drive the drawing as drawn.

| Path | Type | Drawn end (x, y, heading) | Drawn control | Actual end after mirror |
| --- | --- | --- | --- | --- |
| Start | — | 31.4072, 8.2495, 90 | — | 110.093, 8.250, 90 |
| 1 | line, constant | 43.2487, 12.0376, 90 | — | 98.251, 12.038, 90 |
| 2 | curve, linear heading | 8.9248, 10.2542, 270 | 23.4838, 45.5865 | 132.575, 10.254, 270 |
| 3 | curve, linear heading | 54.2865, 123.8186, 300 | 31.6945, 122.2396 | 87.214, 123.819, 240 |
| 4 | curve, linear heading | 59.0665, 32.0101, 270 | 65.0907, 118.0772 | 82.434, 32.010, 270 |
| 5 | line, constant | 58.3463, 121.8874, 270 | — | 83.154, 121.887, 270 |
| 6 | curve, constant | 8.3808, 58.6368, 270 | 33.5008, 103.3373 | 133.119, 58.637, 270 |
| 7 | line, constant | 8.8433, 94.5704, 270 | — | 132.657, 94.570, 270 |

Each path ends when Pedro is at the parametric end and within `POSITION_ERROR_INCHES` / `HEADING_ERROR_DEGREES`. A path
over `PATH_TIMEOUT_SECONDS`, a loop over `MAX_LOOP_SECONDS`, non-finite localization or `AUTO_LIMIT_SECONDS` stops
everything. Settings are in `Constants/AutoPathConstants` (Dashboard), read at INIT.

## History

- `16d5d0d` ran a west-shifted route an earlier agent had made; its path 1 went to (20, 17), which is the "X flipped"
  run. `33368b6` restored the drawn points and added an auto-only strafe flip. The flip was a misdiagnosis: with the drawn
  points it makes path 1 run away sideways. The flip (`AutoMecanum`, `REVERSE_STRAFE`) is now removed, and the drive is plain
  Pedro `Mecanum` again, as on main before `d8649e7`.
- `docs/AUTO_REVIEW.md` reviewed the shooting version at `d8649e7`; its shooting findings don't apply to this
  movement-only routine.

## Verification

Android debug assembly passes. Running the real Pedro core 3.0.2 Follower/Foresight, with this file and the repo's tuned
constants, against an idealized simulated robot: every path finishes, about 10 s in total, and path 1 moves −11.9 in X.
The real robot will be slower. Not yet run on the robot. `tools/AutoPathGeometryCheck.java` still names the old route
fields and no longer compiles.
