package org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Utility.Clock;

import dev.nextftc.robot.Mechanism;

/// drivetrain + localization: periodic updates the follower, so every mechanism after it sees this loop's pose.
/// follow(path) is Ivy's PedroCommands.follow, finished only once the robot is really at the end (tolerances), with a
/// timeout that faults the auto
public final class Drive implements Mechanism {

    public final Follower follower;
    private double positionTolerance = 1, headingTolerance = 3, timeout = 8;
    /// set when a path times out; the auto's watchdog stops everything on it
    public String fault = "";
    public String status = "";

    public Drive(HardwareMap hardwareMap, Pose start) {

        follower = DrivetrainConstants.createFollower(hardwareMap);
        follower.holdEnd.set(true);
        follower.setPose(start);
    }

    @Override
    public void periodic() {
        follower.update();
    }

    /// how close the end of a path has to be (in, deg) and how long a path may take (s)
    public void configure(double positionInches, double headingDegrees, double timeoutSeconds) {

        positionTolerance = positionInches;
        headingTolerance = headingDegrees;
        timeout = timeoutSeconds;
    }

    public Pose pose() {
        return follower.pose();
    }

    public Command follow(final Path path) {

        final Pose end = path.endPose();
        final double[] started = {Double.NaN};
        return PedroCommands.follow(follower, path)
                .requiring(this)
                .setExecute(() -> {
                    if (started[0] != started[0]) {
                        started[0] = Clock.seconds();
                        status = String.format(java.util.Locale.US, "to (%.0f, %.0f)", end.x(), end.y());
                    }
                })
                .setDone(() -> {
                    if (started[0] == started[0] && Clock.seconds() - started[0] >= timeout) {
                        fault = "path to (" + Math.round(end.x()) + ", " + Math.round(end.y()) + ") timed out";
                        return true;
                    }
                    return follower.atParametricEnd() && isAt(end);
                })
                .setEnd(condition -> started[0] = Double.NaN);
    }

    public Command hold(final Pose pose) {
        return instant(() -> follower.hold(pose));
    }

    public boolean isAt(Pose target) {

        Pose pose = follower.pose();
        double angle = pose.heading() - target.heading();
        return pose.distance(target) <= positionTolerance && Math.abs(Math.toDegrees(Math.atan2(Math.sin(angle), Math.cos(angle)))) <= headingTolerance;
    }

    public void stop() {
        follower.stop();
    }

    /// {x, y, heading deg} from a constants class -> Pose, checked
    public static Pose pose(double[] p) {

        if (p == null || p.length != 3) throw new IllegalArgumentException("Pose needs {x, y, heading deg}");
        for (double v : p) {
            if (!Double.isFinite(v)) throw new IllegalArgumentException("Pose isn't finite");
        }
        if (p[0] < 0 || p[0] > 144 || p[1] < 0 || p[1] > 144) throw new IllegalArgumentException("Pose is off the field");
        return new Pose(p[0], p[1], Math.toRadians(p[2]));
    }

    /// straight, heading turning evenly from one pose to the other
    public static Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    /// cubic through two controls {x1, y1, x2, y2}, heading turning evenly
    public static Path curve(Pose from, double[] controls, Pose to) {

        if (controls == null || controls.length != 4) throw new IllegalArgumentException("Controls need {x1, y1, x2, y2}");
        return Paths.curve(from, new Pose(controls[0], controls[1]), new Pose(controls[2], controls[3]), to).linear(from, to);
    }

    /// the same path capped at this speed (in/s)
    public static Path slow(Path path, double inchesPerSecond) {
        return path.with(DrivetrainConstants.foresightConfig.maxPathSpeed.at(inchesPerSecond));
    }
}
