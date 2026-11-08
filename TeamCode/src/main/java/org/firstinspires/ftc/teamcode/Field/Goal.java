package org.firstinspires.ftc.teamcode.Field;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;

public enum Goal {

    RED, BLUE;

    private double closeXOffset = 0, closeYOffset = 0, farXOffset = 0, farYOffset = 0;

    public static Goal forAlliance(Alliance alliance) {
        return alliance.isRedAlliance() ? RED : BLUE;
    }

    public static boolean isClose(Pose robotPose) {
        return robotPose.y() > ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER;
    }

    public Pose getCloseCoordinate() {
        return this == RED ? new Pose(FieldConstants.RED_GOAL_CLOSE_X + closeXOffset, FieldConstants.RED_GOAL_CLOSE_Y + closeYOffset) : new Pose(FieldConstants.BLUE_GOAL_CLOSE_X + closeXOffset, FieldConstants.BLUE_GOAL_CLOSE_Y + closeYOffset);
    }

    public Pose getFarCoordinate() {
        return this == RED ? new Pose(FieldConstants.RED_GOAL_FAR_X + farXOffset, FieldConstants.RED_GOAL_FAR_Y + farYOffset) : new Pose(FieldConstants.BLUE_GOAL_FAR_X + farXOffset, FieldConstants.BLUE_GOAL_FAR_Y + farYOffset);
    }

    public Pose getCoordinate(Pose robotPose) {
        return isClose(robotPose) ? getCloseCoordinate() : getFarCoordinate();
    }

    public static boolean isSet(Pose coordinate) {
        return MathUtil.isFinite(coordinate.x()) && MathUtil.isFinite(coordinate.y());
    }

    public void nudgeFromDriver(Pose robotPose, double up, double right) {

        double toward = this == RED ? 1 : -1;
        double dx = toward * up, dy = -toward * right;
        if (isClose(robotPose)) {
            closeXOffset += dx;
            closeYOffset += dy;
        }
        else {
            farXOffset += dx;
            farYOffset += dy;
        }
    }

    public void resetOffsets() {

        closeXOffset = 0;
        closeYOffset = 0;
        farXOffset = 0;
        farYOffset = 0;
    }
}
