package org.firstinspires.ftc.teamcode.Field;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Constants.FieldConstants;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;

//GARDENS (manual 9.3, Figure 9-2): red runs along the audience wall from the audience-left corner,
//blue along the far wall from the opposite corner
public enum Garden {

    RED, BLUE;

    public static Garden forAlliance(Alliance alliance) {
        return alliance.isRedAlliance() ? RED : BLUE;
    }

    /// robot pointing at its GARDEN and pushed all the way into the corner: front on the GARDEN's wall, side on the other wall
    public Pose relocalizationPose() {

        double side = FieldConstants.ROBOT_CENTER_TO_SIDE, front = FieldConstants.ROBOT_CENTER_TO_FRONT, size = FieldConstants.FIELD_SIZE;
        if (this == RED) return new Pose(side, front, -Math.PI / 2);
        return new Pose(size - side, size - front, Math.PI / 2);
    }
}
