package org.firstinspires.ftc.teamcode.Autonomous.Autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.AutoShotWindow;
import org.firstinspires.ftc.teamcode.Autonomous.Paths.AutoPathRoutes;
import org.firstinspires.ftc.teamcode.Constants.AutoPathConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Alliance.AllianceState;
import org.firstinspires.ftc.teamcode.Systems.AthletechRobot;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.ShotGate;
import org.firstinspires.ftc.teamcode.Utility.Clock;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

//NextFTC runs the loop: this periodic, then AthletechRobot's (bulk reads), the mechanisms (drive localizes, intake, shooter
//aims), then these commands
@NextAutonomous(name = "AutoPath Red", group = "A_Match")
public final class AutoPath extends NextOpMode {

    private final AthletechRobot robot;
    private final AutoShotWindow shot = new AutoShotWindow();
    private final ShotGate speedGate;
    private final Command routine;
    private String stage = "Init", fault = "";
    private double pathStarted, autoStarted, lastLoop;
    private boolean finished, stopped;
    private final boolean reverseStrafe;
    private int shotTimeouts, activeShot;
    private final double autoLimit, pathTimeout, shotTimeout, feedSeconds, readyDwell, maxLoop, positionError, headingError, maxSpeed, maxYaw;

    public AutoPath(AthletechRobot robot) {

        super(robot);
        this.robot = robot;
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
        double band = AutoPathConstants.FLYWHEEL_ERROR_TICKS_PER_SECOND;
        double[] positive = {autoLimit, pathTimeout, positionError, headingError, maxSpeed, maxYaw, band, IntakeConstants.INTAKE_VELOCITY};
        for (double value : positive) if (!Double.isFinite(value) || value <= 0) throw new IllegalArgumentException("Invalid AutoPath settings");
        if (autoLimit > 30) throw new IllegalArgumentException("AutoPath must end within 30 seconds");
        speedGate = new ShotGate(band, band, band * 2, band * 2, readyDwell, 0.05, 0);
        reverseStrafe = AutoPathConstants.REVERSE_STRAFE;
        robot.init(hardwareMap, AutoPathRoutes.START, reverseStrafe);
        routine = autoRoutine();
    }

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
                instant(() -> {
                    if (fault.isEmpty()) finished = true;
                })
        );
    }

    private Command drive(final int number, final Path path, final Pose end) {

        return Command.build()
                .requiring(robot.drive)
                .setStart(() -> {
                    if (!fault.isEmpty()) return;
                    stage = "Path " + number;
                    pathStarted = Clock.seconds();
                    robot.blocker.set(false);
                    robot.drive.follower.follow(path);
                })
                .setDone(() -> {
                    if (!fault.isEmpty()) return true;
                    if (Clock.seconds() - pathStarted >= pathTimeout) {
                        fault = stage + " timed out";
                        return true;
                    }
                    return robot.drive.follower.atParametricEnd() && robot.drive.isAt(end, positionError, headingError);
                });
    }

    private Command shoot(final int number, final Pose end) {

        return Command.build()
                .requiring(robot.drive, robot.blocker)
                .setStart(() -> {
                    if (!fault.isEmpty()) return;
                    activeShot = number;
                    stage = "Shoot after path " + number;
                    robot.drive.follower.hold(end);
                    robot.blocker.set(false);
                    speedGate.reset();
                    shot.start(Clock.seconds(), feedSeconds, shotTimeout, readyDwell, maxLoop);
                })
                .setExecute(() -> {
                    if (!fault.isEmpty()) return;
                    double now = Clock.seconds();
                    speedGate.update(now, robot.shooter.flywheel.getTargetVelocity(), robot.shooter.flywheel.getCurrentVelocity(), shot.firing());
                    Velocity velocity = robot.drive.follower.velocity();
                    boolean stationary = Math.hypot(velocity.vx, velocity.vy) <= maxSpeed && Math.abs(Math.toDegrees(velocity.omega)) <= maxYaw;
                    boolean ready = robot.drive.isAt(end, positionError, headingError) && stationary && speedGate.isReady() && robot.shooter.turret.isOnTarget() && robot.shooter.shooter.isAimSolutionUsable();
                    shot.update(now, ready);
                    robot.blocker.set(shot.firing());
                    if (shot.state() == AutoShotWindow.State.FAULT) fault = "Shot timing fault";
                })
                .setDone(() -> !fault.isEmpty() || shot.finished())
                .setEnd(condition -> {
                    robot.blocker.set(false);
                    if (activeShot == number && shot.state() == AutoShotWindow.State.TIMED_OUT) shotTimeouts++;
                    activeShot = 0;
                });
    }

    @Override
    public void disabledPeriodic() {

        Telemetry.log("RED route. Turret must point straight at INIT.");
        Telemetry.log("Auto strafe reversed", reverseStrafe);
        Telemetry.log("Intake runs throughout. Shoot after paths 1, 3 and 5.");
        Telemetry.log("Check intake/wall clearance and cell orientation before running.");
    }

    @Override
    public void start() {

        shot.start(Clock.seconds(), feedSeconds, shotTimeout, readyDwell, maxLoop);
        robot.shooter.start(new AllianceState(Alliance.RED));
        robot.shooter.setSpinUp(true);
        robot.intake.setVelocity(IntakeConstants.INTAKE_VELOCITY);
        autoStarted = lastLoop = Clock.seconds();
        Scheduler.schedule(routine);
    }

    @Override
    public void periodic() {

        if (stopped) {
            show();
            return;
        }
        double now = Clock.seconds();
        if (now - autoStarted >= autoLimit) stage = "Auto time limit";
        else if (!Double.isFinite(now) || now < lastLoop || now - lastLoop > maxLoop) fault = "Loop timing fault";
        lastLoop = now;
        //last loop's pose: this loop's comes after this, in the drive's periodic
        Pose pose = robot.drive.pose();
        Velocity velocity = robot.drive.follower.velocity();
        if (!finite(pose.x(), pose.y(), pose.heading(), velocity.vx, velocity.vy, velocity.omega)) fault = "Invalid localization";
        if (finished) stage = "Done";
        if (finished || !fault.isEmpty() || stage.equals("Auto time limit")) stop();
        show();
    }

    @Override
    public void end() {
        stop();
    }

    private void stop() {

        if (stopped) return;
        stopped = true;
        Scheduler.reset();
        robot.stopAll();
    }

    private void show() {

        Telemetry.log("Stage", stage);
        Telemetry.log("Fault", fault);
        Telemetry.log("Shot", shot.state());
        Telemetry.log("Shot open seconds", shot.fedSeconds());
        Telemetry.log("Shot timeouts", shotTimeouts);
        Telemetry.log("Flywheel target", robot.shooter.flywheel.getTargetVelocity());
        Telemetry.log("Flywheel actual", robot.shooter.flywheel.getCurrentVelocity());
        Telemetry.log("Aim error deg", robot.shooter.turret.getAimErrorDegrees());
        Telemetry.log("Pose", robot.drive.pose());
    }

    private static boolean finite(double a, double b, double c, double d, double e, double f) {
        return Double.isFinite(a) && Double.isFinite(b) && Double.isFinite(c) && Double.isFinite(d) && Double.isFinite(e) && Double.isFinite(f);
    }
}
