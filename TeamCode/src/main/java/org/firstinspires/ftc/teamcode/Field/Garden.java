package org.firstinspires.ftc.teamcode.Field;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;

public enum Garden {

    RED, BLUE;

    public static Garden forAlliance(Alliance alliance) {
        return alliance.isRedAlliance() ? RED : BLUE;
    }

    public Pose relocalizationPose() {

        double side = FieldConstants.ROBOT_CENTER_TO_SIDE, front = FieldConstants.ROBOT_CENTER_TO_FRONT, size = FieldConstants.FIELD_SIZE;
        if (this == RED) return new Pose(side, front, -Math.PI / 2);
        return new Pose(size - side, size - front, Math.PI / 2);
    }
}
