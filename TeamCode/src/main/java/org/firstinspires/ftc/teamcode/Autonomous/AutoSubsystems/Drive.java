package org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.firstinspires.ftc.teamcode.Utility.Recording.Recorder;

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
        final double[] started = {Double.NaN}, stillSince = {Double.NaN};
        return PedroCommands.follow(follower, path)
                .requiring(this)
                .setExecute(() -> {
                    double now = Clock.seconds();
                    if (started[0] != started[0]) {
                        started[0] = now;
                        status = String.format(java.util.Locale.US, "to (%.0f, %.0f)", end.x(), end.y());
                    }
                    Velocity v = follower.velocity();
                    boolean still = Math.hypot(v.vx, v.vy) < DrivetrainConstants.AUTO_STILL_INCHES_PER_SECOND && Math.abs(Math.toDegrees(v.omega)) < DrivetrainConstants.AUTO_STILL_DEGREES_PER_SECOND;
                    if (follower.atParametricEnd() && still && isWithin(end, DrivetrainConstants.AUTO_ARRIVE_INCHES, DrivetrainConstants.AUTO_ARRIVE_DEGREES)) {
                        if (stillSince[0] != stillSince[0]) stillSince[0] = now;
                    }
                    else stillSince[0] = Double.NaN;
                })
                .setDone(() -> {
                    if (started[0] != started[0]) return false;
                    double now = Clock.seconds();
                    if (follower.atParametricEnd() && isAt(end)) return true;
                    if (stillSince[0] == stillSince[0] && now - stillSince[0] >= DrivetrainConstants.AUTO_STILL_SECONDS) {
                        Recorder.note("auto", String.format(java.util.Locale.US, "stopped %.1f in short of (%.0f, %.0f), going on", follower.pose().distance(end), end.x(), end.y()));
                        return true;
                    }
                    if (now - started[0] >= timeout) {
                        fault = "path to (" + Math.round(end.x()) + ", " + Math.round(end.y()) + ") timed out";
                        return true;
                    }
                    return false;
                })
                .setEnd(condition -> {
                    started[0] = Double.NaN;
                    stillSince[0] = Double.NaN;
                });
    }

    //for touching things (the FLOWER with the dropdown): ends at the end, when it stops moving (it touched), or at maxSeconds, never
    //a fault; then holds where it really is, so it doesn't keep pushing into what it touched
    public Command approach(final Path path, final double stillInchesPerSecond, final double stallSeconds, final double maxSeconds) {

        final Pose end = path.endPose();
        final double[] started = {Double.NaN}, stillSince = {Double.NaN};
        return PedroCommands.follow(follower, path)
                .requiring(this)
                .setExecute(() -> {
                    double now = Clock.seconds();
                    if (started[0] != started[0]) {
                        started[0] = now;
                        status = String.format(java.util.Locale.US, "touching (%.0f, %.0f)", end.x(), end.y());
                    }
                    double speed = Math.hypot(follower.velocity().vx, follower.velocity().vy);
                    //ignore the start, it has to get going first
                    if (now - started[0] > 0.3 && speed < stillInchesPerSecond) {
                        if (stillSince[0] != stillSince[0]) stillSince[0] = now;
                    }
                    else stillSince[0] = Double.NaN;
                })
                .setDone(() -> {
                    if (started[0] != started[0]) return false;
                    double now = Clock.seconds();
                    boolean there = follower.atParametricEnd() && follower.pose().distance(end) <= 0.3;
                    boolean stalled = stillSince[0] == stillSince[0] && now - stillSince[0] >= stallSeconds;
                    return there || stalled || now - started[0] >= maxSeconds;
                })
                .setEnd(condition -> {
                    follower.hold(follower.pose());
                    started[0] = Double.NaN;
                    stillSince[0] = Double.NaN;
                });
    }

    public Command hold(final Pose pose) {
        return instant(() -> follower.hold(pose));
    }

    public boolean isAt(Pose target) {
        return isWithin(target, positionTolerance, headingTolerance);
    }

    private boolean isWithin(Pose target, double inches, double degrees) {

        Pose pose = follower.pose();
        double angle = pose.heading() - target.heading();
        return pose.distance(target) <= inches && Math.abs(Math.toDegrees(Math.atan2(Math.sin(angle), Math.cos(angle)))) <= degrees;
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
