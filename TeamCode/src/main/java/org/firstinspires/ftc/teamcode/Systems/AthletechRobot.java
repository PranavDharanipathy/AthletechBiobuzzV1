package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
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
import org.firstinspires.ftc.teamcode.Utility.AutoHandoff;
import org.firstinspires.ftc.teamcode.Utility.Recording.Recorder;
import org.firstinspires.ftc.teamcode.Utility.Recording.RobotRecording;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;
import org.firstinspires.ftc.teamcode.Utility.TickRateChecker;

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

public final class AthletechRobot implements NextRobot {

    public Drive drive;
    public Intake intake;
    public FIArm fiArm;
    public Blocker blocker;
    public Shooter shooter;

    private final Set<Mechanism> mechanisms = new LinkedHashSet<>();
    private List<LynxModule> hubs = Collections.emptyList();

    private double feedSeconds = 2, settleSeconds = 0.15, shotTimeout = 4, flywheelBand = 50, maxLoop = 0.15;
    private double shotStart, feedStart;
    private boolean feeding, shotDone, shotLate, handoff;
    private String autoName = "";
    private double startedAt, lastLoop, timeLimit = 29.5;
    public String status = "Init", fault = "", shotWaitingOn = "";
    public Recorder recorder;
    public int shotsTaken, shotTimeouts;

    public AthletechRobot() {}

    public void init(HardwareMap hardwareMap, Pose start) {

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(GeneralConstants.BULK_CACHING_MODE);
        clearCaches();
        drive = new Drive(hardwareMap, start);
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
        startRecording(hardwareMap);
    }

    private void startRecording(HardwareMap hardwareMap) {

        if (recorder != null) recorder.close();
        StackTraceElement[] stack = new Throwable().getStackTrace();
        String auto = stack.length > 2 ? stack[2].getClassName().substring(stack[2].getClassName().lastIndexOf('.') + 1) : "Auto";
        recorder = RobotRecording.open(auto);
        RobotRecording.context(recorder, auto, "", hardwareMap);
        RobotRecording.loop(recorder);
        RobotRecording.pose(recorder, drive.follower);
        RobotRecording.turret(recorder, shooter.turret);
        RobotRecording.flywheel(recorder, shooter.flywheel);
        RobotRecording.shooter(recorder, shooter.shooter);
        RobotRecording.blocker(recorder, blocker.blocker, false);
        RobotRecording.veloMotor(recorder, "intake_motor", intake.motor);
        RobotRecording.fiArm(recorder, fiArm.arm);
        recorder.channel("auto", "time", "s", "since the routine started", () -> startedAt > 0 ? Clock.seconds() - startedAt : Double.NaN);
        recorder.channel("auto", "shots", "", "count", () -> shotsTaken);
        recorder.channel("auto", "shot_timeouts", "", "fed before the flywheel was up", () -> shotTimeouts);
        recorder.flag("auto", "feeding", "", () -> feeding);
        recorder.watch("auto", "status", () -> status);
        recorder.watch("auto", "fault", () -> fault);
        recorder.watch("auto", "shot_waiting_on", () -> shotWaitingOn);
    }

    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    private final TickRateChecker tickRate = new TickRateChecker();

    @Override
    public void periodic() {

        tickRate.tick();
        clearCaches();
        HardwareMonitor.get().update();
        recordHandoff();
        if (recorder != null) recorder.sample();
    }

    private void clearCaches() {

        for (LynxModule hub : hubs) {
            if (!BlazeSession.feeds(hub)) hub.clearBulkCache();
        }
    }

    public void configureShots(double feedSeconds, double settleSeconds, double timeoutSeconds, double flywheelBand) {

        this.feedSeconds = feedSeconds;
        this.settleSeconds = settleSeconds;
        shotTimeout = timeoutSeconds;
        this.flywheelBand = flywheelBand;
    }

    public void start(Alliance alliance) {
        shooter.start(new AllianceState(alliance));
    }

    public void handOffAs(String autoName) {

        this.autoName = autoName;
        handoff = true;
        if (recorder != null) recorder.event("auto", "START " + autoName);
    }

    private void recordHandoff() {
        if (handoff) AutoHandoff.record(autoName, drive.pose(), shooter.turret.getPosition() - shooter.turret.startPosition);
    }

    public static void requireCell(Pose shotPose, boolean close, String name) {
        if (Goal.isClose(shotPose) != close) throw new IllegalArgumentException(name + " is on the wrong half for the " + (close ? "CLOSE" : "FAR") + " cell");
    }

    public Command shoot(final Pose at) {
        return shoot(at, Double.NaN);
    }

    //shoots where the robot already is (it stays held there), feeding for seconds: every ball the intake takes in goes straight out
    public Command shootHere(final double seconds) {
        return shoot(null, seconds);
    }

    private Command shoot(final Pose at, final double seconds) {

        final double[] feed = {feedSeconds};
        return Command.build()
                .requiring(drive, blocker, intake)
                .setStart(() -> {
                    status = at == null ? "shooting in place" : "shooting";
                    feed[0] = seconds == seconds ? seconds : feedSeconds;
                    if (at != null) drive.follower.hold(at);
                    blocker.set(false);
                    intake.setVelocity(0);
                    shotStart = Clock.seconds();
                    feeding = shotDone = shotLate = false;
                    shotWaitingOn = "flywheel speed";
                })
                .setExecute(() -> {
                    double now = Clock.seconds();
                    if (!feeding) {
                        boolean fast = shooter.flywheel.getCurrentVelocity() >= shooter.flywheel.getTargetVelocity() - flywheelBand;
                        shotLate = !fast && now - shotStart >= shotTimeout;
                        if (fast || shotLate) {
                            feeding = true;
                            feedStart = now;
                            shotWaitingOn = "";
                            blocker.set(true);
                            intake.setVelocity(IntakeConstants.INTAKE_VELOCITY);
                        }
                    }
                    //intake stops after the feed, the blocker stays open a moment so the last ball clears, then shuts before it drives off
                    else if (now - feedStart >= feed[0] + settleSeconds) {
                        blocker.set(false);
                        shotDone = true;
                    }
                    else if (now - feedStart >= feed[0]) intake.setVelocity(0);
                })
                .setDone(() -> shotDone)
                .setEnd(condition -> {
                    blocker.set(false);
                    intake.setVelocity(0);
                    shotWaitingOn = "";
                    shotsTaken++;
                    if (shotLate) shotTimeouts++;
                });
    }

    public Command collect() {
        return parallel(fiArm.deploy(), intake.run(), instant(() -> status = "collecting"));
    }

    public Command stow() {
        return parallel(fiArm.retract(), intake.stop());
    }

    public void configureAuto(double timeLimitSeconds, double maxLoopSeconds) {

        timeLimit = timeLimitSeconds;
        maxLoop = maxLoopSeconds;
    }

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
            finish();
        }));
    }

    private void finish() {

        if (blocker != null) blocker.set(false);
        if (intake != null) intake.stopNow();
        if (drive != null) drive.stop();
        if (shooter != null) shooter.home();
    }

    public void stopAll() {

        recordHandoff();
        handoff = false;
        if (recorder != null) {
            recorder.sample();
            recorder.close();
            recorder = null;
        }
        if (blocker != null) blocker.set(false);
        if (intake != null) intake.stopNow();
        if (shooter != null) shooter.stopNow();
        if (drive != null) drive.stop();
    }

    public void showInit(String name, boolean shoots) {

        Telemetry.log(name);
        Telemetry.log("Turret must point straight at INIT: that's its 0.");
        if (shoots) Telemetry.log("4 POLLEN loaded.");
        Telemetry.log("Start pose", drive.pose());
        Telemetry.log("Blaze", BlazeSession.status());
        shooter.turret.readSensors();
        Telemetry.log("Turret encoder", String.format(java.util.Locale.US, "%.1f deg", (shooter.turret.getPosition() - shooter.turret.startPosition) / ShooterConstants.TURRET_TICKS_PER_DEGREE) + (shooter.turret.hasEncoderCounted() ? "" : ", turn the turret by hand: this must change"));
    }

    public void show() {

        Telemetry.log("Status", status);
        Telemetry.log("Drive", drive.status);
        Telemetry.log("Fault", fault);
        Telemetry.log("Pose", drive.pose());
        Telemetry.log("Shots", shotsTaken + " (" + shotTimeouts + " fed before the flywheel was up)");
        Telemetry.log("Shot waiting on", shotWaitingOn);
        Telemetry.log("Loop", tickRate.describe());
    }
}
