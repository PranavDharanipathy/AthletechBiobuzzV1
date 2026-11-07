package org.firstinspires.ftc.teamcode.Autonomous.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.AutoShotWindow;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.AutoPathRoutes;
import org.firstinspires.ftc.teamcode.Constants.AutoPathConstants;
import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Alliance.AllianceState;
import org.firstinspires.ftc.teamcode.Systems.Blocker;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.ShotGate;
import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.firstinspires.ftc.teamcode.Utility.GeneralVeloMotor;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredServo;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;
import java.util.List;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Autonomous(name = "AutoPath Red", group = "A_Match")
public final class AutoPath extends LinearOpMode {

    private Follower follower;
    private GeneralVeloMotor intake;
    private Flywheel flywheel;
    private TurretBase turret;
    private Blocker blocker;
    private Shooter shooter;
    private ShotGate speedGate;
    private List<LynxModule> hubs;
    private final AutoShotWindow shot = new AutoShotWindow();
    private String stage = "Init", fault = "";
    private double pathStarted, autoStarted, lastLoop;
    private boolean finished;
    private int shotTimeouts, activeShot;
    private double autoLimit, pathTimeout, shotTimeout, feedSeconds, readyDwell, maxLoop, positionError, headingError, maxSpeed, maxYaw;
    private double intakeVelocity;

    public Command autoRoutine() {

        return sequential(
                drive(1, AutoPathRoutes.path1(), AutoPathRoutes.ONE),
                shoot(1, AutoPathRoutes.ONE),
                drive(2, AutoPathRoutes.path2(), AutoPathRoutes.TWO),
                drive(3, AutoPathRoutes.path3(), AutoPathRoutes.THREE),
                shoot(3, AutoPathRoutes.THREE),
                drive(4, AutoPathRoutes.path4(), AutoPathRoutes.FOUR),
                drive(5, AutoPathRoutes.path5(), AutoPathRoutes.FIVE),
                shoot(5, AutoPathRoutes.FIVE),
                drive(6, AutoPathRoutes.path6(), AutoPathRoutes.SIX),
                drive(7, AutoPathRoutes.path7(), AutoPathRoutes.SEVEN),
                instant(() -> { if (fault.isEmpty()) finished = true; })
        );
    }

    private Command drive(int number, Path path, Pose end) {

        return Command.build()
                .setStart(() -> {
                    if (!fault.isEmpty()) return;
                    stage = "Path " + number;
                    pathStarted = Clock.seconds();
                    blocker.setState(Blocker.BlockerState.BLOCK);
                    follower.follow(path);
                })
                .setDone(() -> {
                    if (!fault.isEmpty()) return true;
                    if (Clock.seconds() - pathStarted >= pathTimeout) {
                        fault = stage + " timed out";
                        return true;
                    }
                    return follower.atParametricEnd() && atPose(end);
                });
    }

    private Command shoot(int number, Pose end) {

        return Command.build()
                .setStart(() -> {
                    if (!fault.isEmpty()) return;
                    activeShot = number;
                    stage = "Shoot after path " + number;
                    follower.hold(end);
                    blocker.setState(Blocker.BlockerState.BLOCK);
                    speedGate.reset();
                    shot.start(Clock.seconds(), feedSeconds, shotTimeout, readyDwell, maxLoop);
                })
                .setExecute(() -> {
                    if (!fault.isEmpty()) return;
                    double now = Clock.seconds();
                    speedGate.update(now, flywheel.getTargetVelocity(), flywheel.getCurrentVelocity(), shot.firing());
                    Velocity velocity = follower.velocity();
                    boolean stationary = Math.hypot(velocity.vx, velocity.vy) <= maxSpeed && Math.abs(Math.toDegrees(velocity.omega)) <= maxYaw;
                    boolean ready = atPose(end) && stationary && speedGate.isReady() && turret.isOnTarget() && shooter.isAimSolutionUsable();
                    shot.update(now, ready);
                    blocker.setState(shot.firing() ? Blocker.BlockerState.CLEAR : Blocker.BlockerState.BLOCK);
                    if (shot.state() == AutoShotWindow.State.FAULT) fault = "Shot timing fault";
                })
                .setDone(() -> !fault.isEmpty() || shot.finished())
                .setEnd(condition -> {
                    blocker.setState(Blocker.BlockerState.BLOCK);
                    if (activeShot == number && shot.state() == AutoShotWindow.State.TIMED_OUT) shotTimeouts++;
                    activeShot = 0;
                });
    }

    private boolean atPose(Pose target) {

        Pose pose = follower.pose();
        double angle = pose.heading() - target.heading();
        return pose.distance(target) <= positionError && Math.abs(Math.toDegrees(Math.atan2(Math.sin(angle), Math.cos(angle)))) <= headingError;
    }

    private void configure() {

        autoLimit = AutoPathConstants.AUTO_LIMIT_SECONDS;
        pathTimeout = AutoPathConstants.PATH_TIMEOUT_SECONDS;
        shotTimeout = AutoPathConstants.SHOT_TIMEOUT_SECONDS;
        feedSeconds = AutoPathConstants.SHOT_FEED_SECONDS;
        readyDwell = AutoPathConstants.SHOT_READY_DWELL_SECONDS;
        maxLoop = AutoPathConstants.MAX_LOOP_SECONDS;
        positionError = AutoPathConstants.POSITION_ERROR_INCHES;
        headingError = AutoPathConstants.HEADING_ERROR_DEGREES;
        maxSpeed = AutoPathConstants.MAX_SHOOTING_SPEED_INCHES_PER_SECOND;
        maxYaw = AutoPathConstants.MAX_SHOOTING_YAW_DEGREES_PER_SECOND;
        intakeVelocity = IntakeConstants.INTAKE_VELOCITY;
        double band = AutoPathConstants.FLYWHEEL_ERROR_TICKS_PER_SECOND;
        double[] positive = {autoLimit, pathTimeout, positionError, headingError, maxSpeed, maxYaw, band, intakeVelocity};
        for (double value : positive) if (!Double.isFinite(value) || value <= 0) throw new IllegalArgumentException("Invalid AutoPath settings");
        if (autoLimit > 30) throw new IllegalArgumentException("AutoPath must end within 30 seconds");
        shot.start(Clock.seconds(), feedSeconds, shotTimeout, readyDwell, maxLoop);
        speedGate = new ShotGate(band, band, band * 2, band * 2, readyDwell, 0.05, 0);
    }

    private void initializeRobot() {

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(GeneralConstants.BULK_CACHING_MODE);
        clearCaches();
        follower = DrivetrainConstants.createFollower(hardwareMap);
        follower.holdEnd.set(true);
        follower.setPose(AutoPathRoutes.START);
        intake = new GeneralVeloMotor(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.intakeMotorDeviceName, false));
        intake.getMotor().setPower(0);
        intake.setDirection(IntakeConstants.INTAKE_MOTOR_DIRECTION);
        intake.setVelocityPDFCoefficients(IntakeConstants.INTAKE_PDF_COEFFICIENTS[0], IntakeConstants.INTAKE_PDF_COEFFICIENTS[1], IntakeConstants.INTAKE_PDF_COEFFICIENTS[2]);
        blocker = new Blocker(new MonitoredServo(hardwareMap, DeviceConstants.blockerServoDeviceName, false));
        blocker.setState(Blocker.BlockerState.BLOCK);
        flywheel = new Flywheel(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.flywheelMotorDeviceName, false));
        flywheel.initVoltageSensor(hardwareMap);
        flywheel.setVelocityPIDVSCoefficients(FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS);
        flywheel.reset();
        turret = new TurretBase(hardwareMap);
        turret.setVolts(0, 0);
        shooter = new Shooter();
        shooter.provideComponents(flywheel, turret, follower);
    }

    @Override
    public void runOpMode() {

        try {
            Scheduler.reset();
            configure();
            initializeRobot();
            Command routine = autoRoutine();
            telemetry.addLine("RED route. Turret must point straight at INIT.");
            telemetry.addLine("Intake runs throughout. Shoot after paths 1, 3 and 5.");
            telemetry.addLine("Check intake/wall clearance and cell orientation before running.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            shooter.start(new AllianceState(Alliance.RED));
            autoStarted = lastLoop = Clock.seconds();
            Scheduler.schedule(routine);
            telemetry.setAutoClear(false);
            telemetry.setMsTransmissionInterval(100);
            telemetry.addData("Stage", () -> stage);
            telemetry.addData("Fault", () -> fault);
            telemetry.addData("Shot", () -> shot.state());
            telemetry.addData("Shot open seconds", () -> shot.fedSeconds());
            telemetry.addData("Shot timeouts", () -> shotTimeouts);
            telemetry.addData("Flywheel target", () -> flywheel.getTargetVelocity());
            telemetry.addData("Flywheel actual", () -> flywheel.getCurrentVelocity());
            telemetry.addData("Aim error deg", () -> turret.getAimErrorDegrees());
            telemetry.addData("Pose", () -> follower.pose());
            while (opModeIsActive() && !finished && fault.isEmpty()) {
                double now = Clock.seconds();
                if (now - autoStarted >= autoLimit) { stage = "Auto time limit"; break; }
                if (!Double.isFinite(now) || now < lastLoop || now - lastLoop > maxLoop) { fault = "Loop timing fault"; break; }
                lastLoop = now;
                clearCaches();
                HardwareMonitor.get().update();
                follower.update();
                Pose pose = follower.pose();
                Velocity velocity = follower.velocity();
                if (!finite(pose.x(), pose.y(), pose.heading(), velocity.vx, velocity.vy, velocity.omega)) { fault = "Invalid localization"; break; }
                shooter.updateAutonomous(true);
                Scheduler.execute();
                if (finished || !fault.isEmpty() || isStopRequested()) break;
                intake.setVelocity(intakeVelocity);
                intake.update();
                telemetry.update();
                idle();
            }
            if (finished) stage = "Done";
        }
        finally {
            stopQuietly(Scheduler::reset);
            if (blocker != null) stopQuietly(() -> blocker.setState(Blocker.BlockerState.BLOCK));
            if (intake != null) stopQuietly(() -> intake.getMotor().setPower(0));
            if (flywheel != null) stopQuietly(flywheel::reset);
            if (turret != null) stopQuietly(() -> turret.setVolts(0, 0));
            if (follower != null) stopQuietly(follower::stop);
            telemetry.addData("Auto result", fault.isEmpty() ? stage : fault);
            telemetry.update();
        }
    }

    private void clearCaches() {
        if (hubs != null) for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    private static boolean finite(double a, double b, double c, double d, double e, double f) {
        return Double.isFinite(a) && Double.isFinite(b) && Double.isFinite(c) && Double.isFinite(d) && Double.isFinite(e) && Double.isFinite(f);
    }

    private static void stopQuietly(Runnable action) {

        try { action.run(); }
        catch (RuntimeException error) { RobotLog.ee("AutoPath", error, "Auto cleanup failed"); }
    }
}
