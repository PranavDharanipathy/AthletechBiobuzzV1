# BioBuzz autonomous workspace

Installed stack: NextFTC v2 0.3.0, Ivy Pedro commands 1.1.1, Pedro Pathing 3.0.2.

| Folder | Purpose |
| --- | --- |
| `Autos/` | The six match autos (Red/Blue Park, Preload + Park, 2 Tip Garden + Park). Each is its paths plus one Ivy sequence of robot commands, with its own `Constants/Autos/<Auto>Constants`; nothing shared between autos |
| `AutoSubsystems/` | NextFTC mechanisms with their own Ivy commands: `Drive` (`follow` = PedroCommands.follow + tolerance + timeout, `hold`), `Intake` (`run`, `stop`), `FIArm` (`deploy`, `retract`), `Blocker`, `Shooter` (`rampUp`, `rampDown`); `AutoShotWindow` (the timed firing window). Commands using several mechanisms (`shoot`, `collect`, `stow`, `autonomous`) are on `Systems/AthletechRobot` |

Every auto is a NextFTC `NextOpMode` (`@NextAutonomous`, group **A_Match**), so NextFTC runs the loop. Each loop runs these
in order:

1. the OpMode's `periodic`, which checks the time limit, loop faults and localization faults;
2. `AthletechRobot.periodic`, which clears the bulk cache and updates HardwareMonitor;
3. the mechanisms, in order:
   - drive, which updates the follower first, so the shooter aims off this loop's pose;
   - intake;
   - FIArm;
   - blocker;
   - shooter;
4. the Ivy commands.

`AthletechRobot`'s constructor stays hardware-free, because NextFTC builds it at app start. `init(hardwareMap, start)` builds
the mechanisms for each OpMode. Don't call `follower.update()` or `Scheduler.execute()` yourself.

The match autos' poses are in the turret's frame (`FieldConstants`), the same as the Pedro visualizer's BIOBUZZ field. They
live in each auto's own `Constants/Autos/<Auto>Constants`. Their rules, routes and tuning are in
`docs/AUTOS.md`, with the visualizer projects in `docs/autos/`. None of these
autos has been run on the field yet.
