# BioBuzz autonomous workspace

Installed stack: NextFTC v2 0.3.0, Ivy Pedro commands 1.1.1, Pedro Pathing 3.0.2.

| Folder | Purpose |
| --- | --- |
| `Autos/` | Driver Station autonomous routines; `AutoPath` is the adjusted red-side seven-path routine |
| `Paths/` | Visualizer geometry; `AutoPathRoutes` keeps the supplied sequence/headings with revised clearance/range coordinates |
| `Commands/` | Shared command factories as more routines are added |
| `AutoSubsystems/` | Autonomous helpers; `AutoShotWindow` handles bounded timed firing |

Select **AutoPath Red**, group **A_Match**. It's a NextFTC `NextOpMode` (`@NextAutonomous`), so NextFTC runs the loop. Each loop runs:
1. the OpMode's `periodic` (time limit, loop and localization faults);
2. `AthletechRobot.periodic` (bulk cache clear, HardwareMonitor);
3. the mechanisms in `Systems/Mechanisms`, in order:
   - `DriveMechanism` updates the follower (localization);
   - `IntakeMechanism`;
   - `BlockerMechanism`;
   - `ShooterMechanism`: flywheel + turret, aimed by `Shooter`'s own logic off this loop's pose;
4. the Ivy commands.

`AthletechRobot`'s constructor stays hardware-free (NextFTC builds it at app start); `init(hardwareMap, start)` builds the
mechanisms each OpMode. Don't call `follower.update()` or `Scheduler.execute()` yourself.

Sequence: **1 → shoot → 2 → 3 → shoot → 4 → 5 → shoot → 6 → 7**. Intake runs during all active travel and shooting steps; the blocker stays closed during travel. Flywheel spin-up and auto aim run in parallel. Everything stops on routine completion, STOP, fatal path/loop fault, or the 29.5-second deadline.

Full procedure and required route/calibration checks: `docs/AUTO_PATH.md`. This routine has compiled and helper tests have passed, but it has not been run on the physical field.

Coordinate revision: firing positions now alternate south/north/south and fall inside the existing flywheel distance-table domain. The garden approach is backed off for the confirmed 14.134-in intake reach. Run `python3 tools/check_auto_path.py` to check the documented footprint/error assumptions. Actual hive tips and pickup success still require field validation.
