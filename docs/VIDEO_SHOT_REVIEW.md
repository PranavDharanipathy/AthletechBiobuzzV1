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
- **The temporal log,** from the SHT Stability Tuner or Temporal Log Red: `python tools/temporal_log.py pull` puts
  `/sdcard/FIRST/data/TemporalLog/<time>/` into `temporal-logs/`. The same session also leaves a robot-log run (and the SHT
  tuner its `ShooterHyperTuning/<time>/` folder); send the run too.
- **The match time and alliance,** if there were several matches.

## How the video and log get lined up

The phone and the robot started at different times, and the robot's clock is wrong (it reads 2028), so wall clocks are
useless. Each run gets one offset: `video seconds = log seconds + offset`.

1. **The log's balls.**
   - **Newer runs** (with `flywheel.contacts`): every ball the flywheel's observer saw hit, at the time and speed it gave it
     (`flywheel.contact_ago`, `flywheel.contact_speed`). This finds later balls of a burst too. `sht.probe` says how far
     off the table the SHT put that burst.
   - **Older runs:** each ball is found from its flywheel dip, searched from the blocker opening until 0.8 s after it
     closes (balls still in the path land after the close). A ball counts when the speed falls at least 40 ticks/s, ends
     below the target, and the raw encoder falls too (the filtered speed overshoots after a recovery and looks like a dip).
     The contact time is where the fall starts. It misses most later balls of a burst.
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
5. **The temporal log has its own clock too.** It opens at INIT and the robot log starts at the first loop, so it's lined
   up through the robot-log run of the same session: video = temporal + to_robot + offset.
   - **`link NAME --run RUN SESSION`:** the SHT tuner and Temporal Log Red drop a `sync` mark in both logs at the same instant
     every 10 s. The result is exact; the test gave −37.2500 s against a true −37.25.
   - **No marks** (older sessions): it cross-correlates the flywheel speed, which both logged every loop. That's good to
     about 5 ms; the test gave −37.246 s, with the temporal values logged 4 ms before the robot log read them.

## Temporal logs: what I look at by hand

The SHT Stability Tuner records a temporal log of everything it has (`SHTStabilityTuner.TEMPORAL_LOG`, on):
- every hardware read and write with its duration;
- every variable of the OpMode when it changes: shooter, flywheel controller terms, turret controller, blocker, intake,
  FIArm, pose, gamepads, and both hyper tuners (each knob's value, trials, keeps, the costs, every ball's contact time and
  speed);
- the shooter-related Constants.

Its cost per loop is `temporal.watch_us`. Check it, because heavier loops are part of what's being tuned.

Commands:
- `at NAME VIDEO_S [--window 0.5]`: what the robot log says at that video moment (pose, flywheel, blocker, turret, distance,
  shot miss, battery), plus every shooter change in the temporal log around it.
- `temporal NAME BALL [--before 0.4 --after 1.0] [--grep RE] [--kinds var,phase,write,read]`: every change around a ball's
  contact, each with its video time.

The agent watching the video does this itself:
- **Look at the frames.** For each burst, check when the ball really left (the frame) against the log's contact (a late
  or early contact means the dip detection or the sync is off). Then look for:
  - balls fed together or jammed;
  - a ball that rattled in the hood;
  - the robot rocking as it shot;
  - a defender's hit;
  - the turret still moving.
- **Read the temporal log at that moment:**
  - blocker write time against the first contact;
  - flywheel power and kick terms through the dip;
  - turret error and volts while the ball was in the shooter;
  - which knob values the tuner had on that burst (`sht.flywheelTuner.shots.knobs[...]`).
- **Write what it saw** into the ball's `--note`.
- **Decide what the evidence says,** and say which part was judged by eye. It can say:
  - table vs flywheel recovery vs aim vs feeding;
  - whether a tuner trial was judged on a burst the video shows was spoiled (jam, hit). Mark it in the review.

## Several videos of the same run (POVs)

Film one run from more than one spot, for example one camera side-on to the goal at its height and one behind the robot. A
ball one camera can't judge (looking along the shot) is usually clear from another.

1. **One review per video, all on the same run:** `new RUN-povA VIDEO_A RUN`, `new RUN-povB VIDEO_B RUN`.
2. **Sync each video on its own:** each phone started at a different time. Use `sync --audio`, `--events` or `link`.
3. **Mark each video's balls as that camera saw them.** Use `unknown` where it can't tell; that doesn't count against the
   other view.
4. **`compare RUN`** lists every ball seen by 2+ videos with each call side by side, disagreements first. Re-look at those.
5. **`fit` merges them automatically,** one row per ball (same run, same log time):
   - views that agree add up: conf = 1 − Π(1 − conf);
   - views that disagree go to the surer call, its conf cut by the others';
   - a near tie (under 0.3 left) is dropped.

## A match video with no SHT

It works the same: `new`, `sync --audio` (or `--events`), `balls`, `shot` / `mark`, `fit`. A match has no probes, so its
balls sit near the table's speed: they move the fast edge, the trust bounds and the ball-to-ball spread, and every review
adds up in `fit`. Later balls of each burst are in the newer logs (observer contacts), so mark those too: they show
whether recovery is still the problem.

## Steps for the agent

```
python -I tools/shot_video.py new  M3 tuning-data/videos/m3.mp4 <auto run> <teleop run>
python -I tools/shot_video.py sheet M3 --from 0 --to 200 --fps 0.5 --cols 6     # where's the robot, the goal, how clear is it
python -I tools/shot_video.py audio M3                                            # audio.png: sound onsets over time
python -I tools/shot_video.py sync  M3 --run TeleOp --audio                        # or --events / --anchor
python -I tools/shot_video.py balls M3                                             # every ball, its video time, distance, speeds
python -I tools/shot_video.py link  M3 --run TeleOp <temporal session>            # if there's a temporal log
python -I tools/shot_video.py shot  M3 R2B07 --roi 0.55,0.05,0.95,0.5 --after 1.8  # every frame of that shot, zoomed on the goal
python -I tools/shot_video.py temporal M3 R2B07                                    # everything the shooter did around it
python -I tools/shot_video.py at    M3 84.5 --window 0.3                           # any moment that looks odd in the video
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
- **What the first review (`sht-20280515`) taught:**
  - Sheets of 3 columns at `--width 700`, `--step 3`, `--before -0.12 --after 1.0` are big enough to see a ball against the
    CELL; smaller tiles weren't.
  - Balls arrive ~0.2-0.45 s after the `shot` reference time.
  - A burst is 2-4 balls ~0.3 s apart, but the log's dips often hold only the first. Mark the logged ball with what *it* did,
    and put what the extra balls did in its note.
  - Robot logs and temporal logs have different zeros (~0.7 s apart): line them up with `link` or the sync marks, never by
    raw time.
  - Newer runs log every contact the flywheel's observer sees (`flywheel.contacts`, `flywheel.contact_speed`), and `balls`
    uses them, so balls no longer go missing.
  - All balls sat near the table's speed, so they only showed where the fast edge is. How far down still scores (and so
    where the middle is) came from an assumption, exit speed proportional to wheel speed, and it moved the table 23 too far.
    The fit now keeps that unknown (below), and the SHT probes bursts off the table to measure it.

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
- **The CELL's shape counts.** The opening is a pentagon leaning back 30°, so how much too fast or too slow still goes in
  depends on the angle the ball comes from, not just the distance. From the side, the fast side shrinks (the roof is
  lower off-center) and the slow side grows. Each ball's window comes from `tools/field_geometry.py`, and the numbers are
  in [FIELD.md](./FIELD.md). The table it fits is the speed that hits the **aim point** (the top middle of the rectangle).
- **The model:**
  - The speed that hits the aim point is the table's line plus an offset and a slope change.
  - Scoring window: each ball's own, from the CELL shape (above), times the **exit gain** (the window in wheel speed over
    the window in exit speed; 1 = exit speed proportional to wheel speed). It isn't known, so every gain in `--gains`
    (0.3..1) is kept and weighed by how well it explains the balls. `--gain G` uses one. `--window N` falls back to one
    fixed ±N ticks/s for every ball.
  - Ball-to-ball spread in the fit: `--sigma` 10 ticks/s.
  - Priors: offset ±25, slope ±1/in, so a few balls only nudge it.
- **Weights:** balls from a moving robot (> 4 in/s) count half, and `--conf` scales each ball.
- **Choosing the table:** not the middle estimate. It's the line that scores most often over balls like the ones seen,
  across every gain and middle the balls allow, with `--spread` ball-to-ball spread (12: the 2028-05-15 first balls missed
  as if balls varied ~14 on their own).
  - **Trust:** it never puts the table more than `--trust` (10) ticks/s outside the speeds seen to score within 3 in of each
    scored ball. Nothing says what's past them. A table already outside that moves back in.
  - The change is added to every table point. The robot fits a line (`FLYWHEEL_FIT_DEGREE` 1) through them, so this moves
    that line exactly. It prints what the line runs at (38-58 in): read that, not the points.
- **Self-test:** synthetic balls from the real 2028-05-13 TeleOp positions, judged with pentagon windows, true center speed
  table +20: found +28 (80% +19..+37). The old fixed ±12 window on the same balls said +2.
- **What it says about the old SHT video** (sht-20280515, 48 balls): the aim-point speed at 47.5 in is 1443 at gain 1 and
  1468 at gain 0.4, every gain about as likely. The table that scores most often across them is the old one minus 8
  (1436 at 38 in, 1459 at 46, 1482 at 54).
- **The sim against the video** (`FlywheelBurstTest.oldSystemMissesLikeTheVideo`): the old controller and table miss
  65-81% of later balls at every gain, as the video did (5 of 7). Its first balls only miss as often as the video's (8 of
  41, half low-confidence rims) with ~14 ticks/s of ball-to-ball spread the wheel can't see. With that spread the new
  system still misses ~4-5% (gain 0.6-1) to 14% (gain 0.4). The SHT video says which.
- **Old reviews stay valid.** Every review counts toward the fit, because it's the wheel's real speed against what
  happened, not the table at the time. Start over only if the shooter hardware changed (hood, wheel, compression).
- **When it changes the table:**
  - Each point moves by at most 25 ticks/s (`--max-step`).
  - It doesn't move when under 8 balls are marked, when the change is under 3 (`--min-change`), or when it scores less
    than 0.2% more often.
- **`--apply`:** writes `ShooterConstants.java` and adds a line to `tuning-data/video-reviews/CHANGES.md`. Tell the user what
  changed and why, and that it needs testing on the robot.

## SHT probes: measuring the window

`SHTStabilityTuner.PROBE` (on) shoots some bursts off the table: `PROBE_OFFSETS` {0, 0, -20, 0, 0, +12, 0, 0, -35}, one
per burst in turn, changed only once a burst's balls are all in. Telemetry "Probe" shows the one in use. Some probe balls
will miss on purpose: that's the measurement.

- Mark every ball as usual. The fit uses each ball's own speed, so probe balls need nothing special. The `probe` column
  says which they were.
- Short balls at -35 and scores at -20 put the slow edge between them. That gives the exit gain, and so the middle.
- Long balls at +12 pin the fast edge.
- Then `fit` and the sim (`FlywheelBurstTest`, `GAINS` / `AIM_SPEED`) get the measured gain, and `FEED_GATE_WINDOW_SCALE`
  gets it too.

## Flywheel gains from the logs (every review)

The video says which balls missed; the logs' flywheel speed, power and supply say why. After marking:

1. `python -I tools/flywheel_fit.py fit <runs>`:
   - **What it does:** runs `FlywheelObserver`'s model over every logged loop with the volts that were sent, and fits the
     wheel (accel per volt, coast, kv, hub lag) so the hub's readings surprise it least.
   - **What it also measures:** each ball's drop, the first ball's time after the blocker opens, the gap between balls, the
     battery, and the loop times.
   - **Where it goes:** `tuning-data/flywheel-fit/plant.properties`.
   - **When the Sim uses the fitted wheel:** only when it beats the code's by 5%+ (`wheelFit=1`). The balls, battery and
     loops always come from the file.
   - **The observer's constants:** if it says `FLYWHEEL_ACCEL_PER_VOLT` / `FLYWHEEL_COAST_VOLTS` / `FLYWHEEL_OBSERVER_LAG`
     fit 5%+ better, change them a step toward the fit.
2. `FLYWHEEL_TUNE=1 ./gradlew :Sim:test --tests "*FlywheelTuning*"` (run in `Sim/`):
   - **What it tries:** steps of x0.8..x1.25 on `kp_close`, `kp_far`, `kd`, `max_p`, the observer's gain and lag.
   - **Where:** that wheel, and the wheel off from it every way the logs allow, in the match setup (no feed gate). The
     gate is checked after.
   - **What it writes:** `Sim/out/flywheel-tune.txt`.
   - **When to take it:** only when it's 3%+ better and no case misses more, with or without the gate. Put it in
     `FlywheelPIDVSCoefficients` / `FlywheelConstants` by hand; the SHT keeps tuning from there on the robot.
3. The FlywheelBurstTest report and `oldSystemMissesLikeTheVideo` must still pass.

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
