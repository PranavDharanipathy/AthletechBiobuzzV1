package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.AutoShotWindow;
import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.Blocker;
import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.Drive;
import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.FIArm;
import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.Intake;
import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.Shooter;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Field.Goal;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Alliance.AllianceState;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.ShotGate;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.Telemetry;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

/// NextFTC builds this once at app start, so the constructor can't touch hardware: each NextOpMode calls init.
/// Every loop: the OpMode's periodic, then this periodic (fresh bulk reads), then the mechanisms in order (drive first,
/// so the shooter aims off this loop's pose), then the commands.
/// The mechanisms have their own commands (drive.follow, intake.run, fiArm.deploy, shooter.spinUp); the ones that take
/// several mechanisms live here (shoot, collect, stow), and autonomous() wraps an auto's steps in the time limit and the
/// fault watchdog. Autos only put these in order with their own paths and constants
public final class AthletechRobot implements NextRobot {

    public Drive drive;
    public Intake intake;
    public FIArm fiArm;
    public Blocker blocker;
    public Shooter shooter;

    private final Set<Mechanism> mechanisms = new LinkedHashSet<>();
    private List<LynxModule> hubs = Collections.emptyList();

    private final AutoShotWindow shot = new AutoShotWindow();
    private ShotGate speedGate = new ShotGate(50, 50, 100, 100, 0.1, 0.05, 0);
    private double feedSeconds = 1.5, shotTimeout = 4, readyDwell = 0.1, maxSpeed = 3, maxYaw = 10, maxLoop = 0.15;
    private double startedAt, lastLoop, timeLimit = 29.5;
    public String status = "Init", fault = "";
    public int shotsTaken, shotTimeouts;

    public AthletechRobot() {}

    public void init(HardwareMap hardwareMap, Pose start) {

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(GeneralConstants.BULK_CACHING_MODE);
        clearCaches();
        drive = new Drive(hardwareMap, start);
        //Blaze's background Pinpoint reads (after the start pose is set)
        BlazeSession.attachPinpoint(drive.follower);
        intake = new Intake(hardwareMap);
        fiArm = new FIArm(hardwareMap);
        blocker = new Blocker(hardwareMap);
        shooter = new Shooter(hardwareMap, drive);
        mechanisms.clear();
        mechanisms.add(drive);
        mechanisms.add(intake);
        mechanisms.add(fiArm);
        mechanisms.add(blocker);
        mechanisms.add(shooter);
        status = "Init";
        fault = "";
        shotsTaken = shotTimeouts = 0;
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    @Override
    public void periodic() {

        clearCaches();
        HardwareMonitor.get().update();
    }

    //hubs Blaze feeds already have fresh data
    private void clearCaches() {

        for (LynxModule hub : hubs) {
            if (!BlazeSession.feeds(hub)) hub.clearBulkCache();
        }
    }

    /// shot timing: feed time, give-up time, how long it has to be ready first (s); flywheel band (ticks/s); how still the
    /// robot has to be (in/s, deg/s)
    public void configureShots(double feedSeconds, double timeoutSeconds, double readyDwellSeconds, double flywheelBand, double maxSpeed, double maxYawDegrees) {

        this.feedSeconds = feedSeconds;
        shotTimeout = timeoutSeconds;
        readyDwell = readyDwellSeconds;
        this.maxSpeed = maxSpeed;
        maxYaw = maxYawDegrees;
        speedGate = new ShotGate(flywheelBand, flywheelBand, flywheelBand * 2, flywheelBand * 2, readyDwellSeconds, 0.05, 0);
    }

    /// shooter on for this alliance (the turret has to point straight at INIT: that's its home)
    public void start(Alliance alliance) {
        shooter.start(new AllianceState(alliance));
    }

    /// the shooter aims at the CELL for the robot's half: a pose on the wrong half would shoot the wrong CELL
    public static void requireCell(Pose shotPose, boolean close, String name) {
        if (Goal.isClose(shotPose) != close) throw new IllegalArgumentException(name + " is on the wrong half for the " + (close ? "CLOSE" : "FAR") + " cell");
    }

    /// holds at, waits for speed, aim, a still robot and a shot that scores, then opens the blocker with the intake feeding
    /// for the feed time (gives up after the timeout)
    public Command shoot(final Pose at) {

        return Command.build()
                .requiring(drive, blocker, intake)
                .setStart(() -> {
                    status = "shooting";
                    drive.follower.hold(at);
                    blocker.set(false);
                    intake.setVelocity(IntakeConstants.INTAKE_VELOCITY);
                    speedGate.reset();
                    shot.start(Clock.seconds(), feedSeconds, shotTimeout, readyDwell, maxLoop);
                })
                .setExecute(() -> {
                    double now = Clock.seconds();
                    speedGate.update(now, shooter.flywheel.getTargetVelocity(), shooter.flywheel.getCurrentVelocity(), shot.firing());
                    Velocity v = drive.follower.velocity();
                    boolean still = Math.hypot(v.vx, v.vy) <= maxSpeed && Math.abs(Math.toDegrees(v.omega)) <= maxYaw;
                    boolean ready = drive.isAt(at) && still && speedGate.isReady() && shooter.turret.isOnTarget() && shooter.shooter.isAimSolutionUsable() && org.firstinspires.ftc.teamcode.Systems.Shooter.shotWillScore();
                    shot.update(now, ready);
                    blocker.set(shot.firing());
                    if (shot.state() == AutoShotWindow.State.FAULT) fault = "shot timing fault";
                })
                .setDone(shot::finished)
                .setEnd(condition -> {
                    blocker.set(false);
                    intake.setVelocity(0);
                    shotsTaken++;
                    if (shot.state() == AutoShotWindow.State.TIMED_OUT) shotTimeouts++;
                });
    }

    /// arm down, intake on
    public Command collect() {
        return parallel(fiArm.deploy(), intake.run(), instant(() -> status = "collecting"));
    }

    /// arm up, intake off
    public Command stow() {
        return parallel(fiArm.retract(), intake.stop());
    }

    /// the auto's time limit and the slowest loop it accepts (s)
    public void configureAuto(double timeLimitSeconds, double maxLoopSeconds) {

        timeLimit = timeLimitSeconds;
        maxLoop = maxLoopSeconds;
    }

    /// an auto's steps in order, stopped by the time limit or a fault (loop too slow, localization broken, a path timed out),
    /// and everything made safe at the end either way
    public Command autonomous(Command... steps) {

        Command watchdog = Command.build()
                .setStart(() -> startedAt = lastLoop = Clock.seconds())
                .setExecute(() -> {
                    double now = Clock.seconds();
                    if (!Double.isFinite(now) || now < lastLoop || now - lastLoop > maxLoop) fault = "loop timing fault";
                    lastLoop = now;
                    Pose p = drive.pose();
                    Velocity v = drive.follower.velocity();
                    if (!(Double.isFinite(p.x()) && Double.isFinite(p.y()) && Double.isFinite(p.heading()) && Double.isFinite(v.vx) && Double.isFinite(v.vy) && Double.isFinite(v.omega))) fault = "invalid localization";
                    if (!drive.fault.isEmpty() && fault.isEmpty()) fault = drive.fault;
                })
                .setDone(() -> !fault.isEmpty() || Clock.seconds() - startedAt >= timeLimit);
        return sequential(race(sequential(steps), watchdog), instant(() -> {
            status = !fault.isEmpty() ? "stopped: " + fault : Clock.seconds() - startedAt >= timeLimit ? "auto time limit" : "done";
            stopAll();
        }));
    }

    /// everything safe: flywheel and turret off, blocker shut, intake and drive stopped
    public void stopAll() {

        if (blocker != null) blocker.set(false);
        if (intake != null) intake.stopNow();
        if (shooter != null) shooter.stopNow();
        if (drive != null) drive.stop();
    }

    public void showInit(String name, boolean shoots) {

        Telemetry.log(name);
        if (shoots) Telemetry.log("Turret must point straight at INIT. 4 POLLEN loaded.");
        Telemetry.log("Start pose", drive.pose());
        Telemetry.log("Blaze", BlazeSession.status());
    }

    public void show() {

        Telemetry.log("Status", status);
        Telemetry.log("Drive", drive.status);
        Telemetry.log("Fault", fault);
        Telemetry.log("Pose", drive.pose());
        Telemetry.log("Shots", shotsTaken + " (" + shotTimeouts + " timed out), " + shot.state());
    }
}
