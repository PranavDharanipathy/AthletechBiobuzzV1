# Match video -> flywheel table review

You send a phone video of a match (any position, just hold the phone up) and the robot's logs from it. An agent watches
every ball in the video, lines each one up with the log, and makes small, evidence-backed changes to
`ShooterConstants.FLYWHEEL_VELOCITIES` and, when the evidence points there, to the flywheel's tuning. Tool:
`tools/shot_video.py` (needs ffmpeg/ffprobe on PATH, numpy, Pillow).

## What to send

- **The video file.** Put it in `tuning-data/videos/` (git-ignored) or give its path. Keep the shooter in view, and the goal
  if you can. Sound helps: the thump of each ball hitting the flywheel is how it syncs on its own.
- **The logs.** Every OpMode records `/sdcard/FIRST/data/RobotLog/<time>-<OpMode>/`, the Temporal Log TeleOp too. Pull them
  with `python tools/robot_log.py pull`, or any Gradle build with the robot on adb. A match is two runs: the auto and the
  TeleOp.
- **The match time and alliance,** if there were several matches.

## How the video and log get lined up

The phone and the robot started at different times, and the robot's clock is wrong (it reads 2028), so wall clocks are
useless. Each run gets one offset: `video seconds = log seconds + offset`.

1. **The log's balls.** Each ball is found from its flywheel dip, searched from the blocker opening until 0.8 s after it
   closes (balls still in the path land after the close). A ball counts when the speed falls at least 40 ticks/s, ends
   below the target, and the raw encoder falls too (the filtered speed overshoots after a recovery and looks like a dip).
   The contact time is where the fall starts.
2. **Video times come from each frame's own timestamp,** never frame number / fps. Phones record variable frame rate,
   and the tool checks how variable this video is. Every picture is labeled with its real time.
3. **Sync:**
   - `sync --audio` matches sound onsets (spectral flux, judged against the 3 s around them, then refined to the 1 ms
     loudness rise) against the log's balls. Every pairing proposes an offset, and the one most others agree with
     (within 60 ms) wins.
   - It says "sure" with at least 3 matches and a clear lead over the next-best offset.
   - Test (synthetic variable-frame-rate video, 25 decoy thumps): true offset +7.300 s, found +7.300 s, 15 of 16 balls,
     4 ms spread.
   - Without usable sound, mark 5 or more launch times by eye (the frame where the ball leaves the shooter) and use
     `sync --events`, 120 ms tolerance.
   - Or use an anchor: `--anchor VIDEO_S=move` (the auto's first drive), `=ball:N` or `=open:N`.
   - Other runs from the same robot boot: `sync` prints where the robot clock says their offset should be (±1.5 s).
     Confirm it with that run's own balls.
4. **Check it in the pictures.** The `shot` strip of a ball must show the ball leaving the shooter within about 0 to
   +100 ms. If balls don't line up, the sync is wrong. Fix the sync before marking anything.

## Steps for the agent

```
python -I tools/shot_video.py new  M3 tuning-data/videos/m3.mp4 <auto run> <teleop run>
python -I tools/shot_video.py sheet M3 --from 0 --to 200 --fps 0.5 --cols 6     # where's the robot, the goal, how clear is it
python -I tools/shot_video.py audio M3                                            # audio.png: sound onsets over time
python -I tools/shot_video.py sync  M3 --run TeleOp --audio                        # or --events / --anchor
python -I tools/shot_video.py balls M3                                             # every ball, its video time, distance, speeds
python -I tools/shot_video.py shot  M3 R2B07 --roi 0.55,0.05,0.95,0.5 --after 1.8  # every frame of that shot, zoomed on the goal
python -I tools/shot_video.py mark  M3 R2B07 short --conf 0.7 --note "hit the front lip"
python -I tools/shot_video.py fit                                                  # every review's marked balls
python -I tools/shot_video.py report M3
```

- **Open each picture with Read** and look at it. For every ball, view a wide strip (where it went) and a zoomed strip on
  the goal (`--roi` = x0,y0,x1,y1 as fractions of the frame).
- **Outcomes:**
  - `score`
  - `short` / `long`: along the shot line
  - `left` / `right`: aim, not used for the table
  - `rim`: hit the edge and stayed out
  - `blocked`: hit something before the goal
  - `none`: the log called it a ball but the video shows none
  - `unknown`
- **`--conf`:**
  - The phone's line of sight makes short vs long hard when it looks along the shot. Judge by where the ball is when it
    passes the goal's plane, where it lands, and how it bounces.
  - Use 0.5 when only one of those is clear, and 1 when it's obvious.
  - Leave it `unknown` rather than guess.
- **Check the log against the video:**
  - If the robot is clearly somewhere else than the log's `x, y` (count tiles), the distance is wrong: note it and lower
    the conf.
  - If the video shows a ball the log missed, note it on the nearest ball.
- `motion --roi <goal box>` plots how much the goal box changes over time. It helps find arrivals in a long video.

## The fit

- **What it uses:** each marked ball says where the speed that scores is at that distance:
  - `score`: the wheel's speed at contact was inside the window;
  - `short`: the speed that scores is above it;
  - `long`: it's below.
- **Distance:** `shooter.shot_distance`, turret to goal, the table's own key.
- **Speed at contact:** the filtered speed (`flywheel.velocity`) where the fall starts, the same thing the SHT tuner scores.
  - The encoder's own speed (a line through its last 0.15 s; it steps 20 ticks/s) is in `raw_at_contact`.
  - While the wheel is still speeding up the two can differ: 1574 filtered vs 1500 raw on the 2028-05-13 auto's first ball.
    A ball like that deserves a note.
- **The model:**
  - The speed that scores is the table's line plus an offset and a slope change.
  - Scoring window: ±(table ticks/s per inch × `SHOT_MISS_TOLERANCE_INCHES`).
  - Ball-to-ball spread: 10 ticks/s.
  - Priors: offset ±25, slope ±1/in, so a few balls only nudge it.
- **Weights:** balls from a moving robot (> 4 in/s) count half, and `--conf` scales each ball.
- **Result:** the offset and slope are then added to every table point. The robot fits a line (`FLYWHEEL_FIT_DEGREE` 1)
  through those points, so this moves that line exactly.
- **Self-test:** synthetic balls, true speed that scores +20 ticks/s and +0.5/in; found +21.7 and +0.75.
- **Old reviews stay valid.** Every review counts toward the fit, because it's the wheel's real speed against what
  happened, not the table at the time. Start over only if the shooter hardware changed (hood, wheel, compression).
- **When it changes the table:**
  - Each point moves by at most 25 ticks/s (`--max-step`).
  - It doesn't move at all when the change is under 3, or when the 80% range is wider than the window plus 20 (too few
    balls).
- **`--apply`:** writes `ShooterConstants.java` and adds a line to `tuning-data/video-reviews/CHANGES.md`. Tell the user what
  changed and why, and that it needs testing on the robot.

## Flywheel tuning, not the table

`fit` also prints how slow the wheel was at each ball of a burst (target - speed at contact) and the results.

- **Short balls hit the wheel slower than scoring ones** (by more than half the window): the wheel isn't recovering
  between balls. A table change would only hide that. Small steps, one at a time, each with its own review after:
  - `FLYWHEEL_PRE_KICK_VOLTS` +0.3 V (a push just before each ball);
  - a tighter `SHOT_GATE_HOLD_BAND`;
  - wider ball spacing.
- **`left` / `right` misses:** aim. Check the turret error it prints.

## What's tracked

`tuning-data/video-reviews/<name>/review.json` (video info, runs, offsets, the table when it started), `shots.csv` (every
ball and its mark) and `REPORT.md`, plus `CHANGES.md`. Frames, plots and videos are git-ignored.
