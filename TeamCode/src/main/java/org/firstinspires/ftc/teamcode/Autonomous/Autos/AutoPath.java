package org.firstinspires.ftc.teamcode.Autonomous.Autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Autonomous.Paths.AutoPathRoutes;
import org.firstinspires.ftc.teamcode.Constants.AutoPathConstants;
import org.firstinspires.ftc.teamcode.Systems.AthletechRobot;
import org.firstinspires.ftc.teamcode.Utility.Clock;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

//movement only: drives the AutoPathRoutes points in order and stops. Shooter and intake are never started, blocker stays shut
@NextAutonomous(name = "AutoPath Red", group = "A_Match")
public final class AutoPath extends NextOpMode {

    private final AthletechRobot robot;
    private final Command routine;
    private String stage = "Init", fault = "";
    private double pathStarted, autoStarted, lastLoop;
    private boolean finished, stopped;
    private final double autoLimit, pathTimeout, maxLoop, positionError, headingError;

    public AutoPath(AthletechRobot robot) {

        super(robot);
        this.robot = robot;
        autoLimit = AutoPathConstants.AUTO_LIMIT_SECONDS;
        pathTimeout = AutoPathConstants.PATH_TIMEOUT_SECONDS;
        maxLoop = AutoPathConstants.MAX_LOOP_SECONDS;
        positionError = AutoPathConstants.POSITION_ERROR_INCHES;
        headingError = AutoPathConstants.HEADING_ERROR_DEGREES;
        double[] positive = {autoLimit, pathTimeout, maxLoop, positionError, headingError};
        for (double value : positive) if (!Double.isFinite(value) || value <= 0) throw new IllegalArgumentException("Invalid AutoPath settings");
        if (autoLimit > 30) throw new IllegalArgumentException("AutoPath must end within 30 seconds");
        robot.init(hardwareMap, AutoPathRoutes.START);
        routine = autoRoutine();
    }

    public Command autoRoutine() {

        Path[] paths = AutoPathRoutes.paths();
        Command[] steps = new Command[paths.length + 1];
        for (int i = 0; i < paths.length; i++) steps[i] = drive(i + 1, paths[i], AutoPathRoutes.ENDS[i]);
        steps[steps.length - 1] = instant(() -> {
            if (fault.isEmpty()) finished = true;
        });
        return sequential(steps);
    }

    private Command drive(final int number, final Path path, final Pose end) {

        return Command.build()
                .requiring(robot.drive)
                .setStart(() -> {
                    if (!fault.isEmpty()) return;
                    stage = "Path " + number + " of " + AutoPathRoutes.ENDS.length;
                    pathStarted = Clock.seconds();
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

    @Override
    public void disabledPeriodic() {

        Telemetry.log("Movement only: no shooting or intake. Route is mirrorX(70.75).");
        Telemetry.log("Start pose", AutoPathRoutes.START);
    }

    @Override
    public void start() {

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
        Telemetry.log("Pose", robot.drive.pose());
    }

    private static boolean finite(double a, double b, double c, double d, double e, double f) {
        return Double.isFinite(a) && Double.isFinite(b) && Double.isFinite(c) && Double.isFinite(d) && Double.isFinite(e) && Double.isFinite(f);
    }
}
