package org.firstinspires.ftc.teamcode.Autonomous.Mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;

import dev.nextftc.robot.Mechanism;

/// drivetrain + localization: periodic updates the follower, so every mechanism after it sees this loop's pose
public final class DriveMechanism implements Mechanism {

    public final Follower follower;

    public DriveMechanism(HardwareMap hardwareMap, Pose start) {

        follower = DrivetrainConstants.createFollower(hardwareMap);
        follower.holdEnd.set(true);
        follower.setPose(start);
    }

    @Override
    public void periodic() {
        follower.update();
    }

    public Pose pose() {
        return follower.pose();
    }

    /// follows path until timeout s or it's at the end and within the given pose tolerance
    public Command follow(final Path path, final Pose end, final double positionError, final double headingErrorDeg, final double timeout) {

        final double[] started = new double[1];
        return Command.build()
                .requiring(this)
                .setStart(() -> {
                    started[0] = org.firstinspires.ftc.teamcode.Utility.Clock.seconds();
                    follower.follow(path);
                })
                .setDone(() -> org.firstinspires.ftc.teamcode.Utility.Clock.seconds() - started[0] >= timeout || follower.atParametricEnd() && isAt(end, positionError, headingErrorDeg));
    }

    public Command hold(final Pose pose) {
        return instant(() -> follower.hold(pose));
    }

    public boolean isAt(Pose target, double positionError, double headingErrorDeg) {

        Pose pose = follower.pose();
        double angle = pose.heading() - target.heading();
        return pose.distance(target) <= positionError && Math.abs(Math.toDegrees(Math.atan2(Math.sin(angle), Math.cos(angle)))) <= headingErrorDeg;
    }

    public void stop() {
        follower.stop();
    }
}
