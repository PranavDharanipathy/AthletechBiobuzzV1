package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config
public class FieldConstants {

    public static Pose START_POSE = new Pose(0, 0, 0);

    public static double FIELD_SIZE = 144;

    public static double ROBOT_CENTER_TO_FRONT = 9.72;
    public static double ROBOT_CENTER_TO_SIDE = 7.64;

    public static double RED_GOAL_CLOSE_X = 59.25, RED_GOAL_CLOSE_Y = 87.77;
    public static double RED_GOAL_FAR_X = 59.25, RED_GOAL_FAR_Y = 56.23;

    public static double BLUE_GOAL_CLOSE_X = 84.75, BLUE_GOAL_CLOSE_Y = 87.77;
    public static double BLUE_GOAL_FAR_X = 84.75, BLUE_GOAL_FAR_Y = 56.23;

}
