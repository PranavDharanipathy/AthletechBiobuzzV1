# BioBuzz autonomous workspace

Installed stack: NextFTC v2 0.3.0, Ivy Pedro commands 1.1.1, Pedro Pathing 3.0.2.

| Folder | Purpose |
| --- | --- |
| `Autos/` | Driver Station autonomous routines; `AutoPath` drives the seven-path route, movement only |
| `Paths/` | Visualizer geometry; `AutoPathRoutes` holds the user's export exactly, through `mirrorX(70.75)` |
| `Commands/` | Shared command factories as more routines are added |
| `AutoSubsystems/` | Autonomous helpers; `AutoShotWindow` (timed firing window, not used by the current routine) |

Select **AutoPath Red**, group **A_Match**. It's a NextFTC `NextOpMode` (`@NextAutonomous`), so NextFTC runs the loop. Each loop runs:
1. the OpMode's `periodic` (time limit, loop and localization faults);
2. `AthletechRobot.periodic` (bulk cache clear, HardwareMonitor);
3. the mechanisms in `Systems/Mechanisms`, in order (drive updates the follower first; intake, blocker and shooter idle
   because auto never starts them);
4. the Ivy commands.

`AthletechRobot`'s constructor stays hardware-free (NextFTC builds it at app start); `init(hardwareMap, start)` builds the
mechanisms each OpMode. Don't call `follower.update()` or `Scheduler.execute()` yourself.

Route, mirror and verification: `docs/AUTO_PATH.md`. Not yet run on the field.
