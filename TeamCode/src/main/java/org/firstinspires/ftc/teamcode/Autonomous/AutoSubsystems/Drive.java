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

public final class Drive implements Mechanism {

    public final Follower follower;
    private double positionTolerance = 1, headingTolerance = 3, timeout = 8;
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

    public static Pose pose(double[] p) {

        if (p == null || p.length != 3) throw new IllegalArgumentException("Pose needs {x, y, heading deg}");
        for (double v : p) {
            if (!Double.isFinite(v)) throw new IllegalArgumentException("Pose isn't finite");
        }
        if (p[0] < 0 || p[0] > 144 || p[1] < 0 || p[1] > 144) throw new IllegalArgumentException("Pose is off the field");
        return new Pose(p[0], p[1], Math.toRadians(p[2]));
    }

    public static Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    public static Path curve(Pose from, double[] controls, Pose to) {

        if (controls == null || controls.length != 4) throw new IllegalArgumentException("Controls need {x1, y1, x2, y2}");
        return Paths.curve(from, new Pose(controls[0], controls[1]), new Pose(controls[2], controls[3]), to).linear(from, to);
    }

    public static Path slow(Path path, double inchesPerSecond) {
        return path.with(DrivetrainConstants.foresightConfig.maxPathSpeed.at(inchesPerSecond));
    }
}
