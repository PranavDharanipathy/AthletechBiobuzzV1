package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

@Config
public class FieldConstants {

    public static Pose START_POSE = new Pose(0, 0, 0);

    //pedro frame, inches: (0, 0) is the corner on the audience's left (red's GARDEN), x toward blue, y away from the audience,
    //heading 0 = +x, counterclockwise
    public static double FIELD_SIZE = 144;

    //from the robot's pose point to its outside, for relocalizing against walls (CAD 11-06: bash bar 247 mm, side plates 194 mm).
    //Intake up: down it sticks out to 359 mm
    public static double ROBOT_CENTER_TO_FRONT = 9.72;
    public static double ROBOT_CENTER_TO_SIDE = 7.64;

    //tipped-up CELL's opening (manual Fig 9-9/9-10): HIVES 25.5 in apart, opening 15.77 in from the pivot toward its wall.
    //CLOSE = the CELL away from the audience (y > FAR_ZONE_CLOSE_ZONE_BARRIER)
    public static double RED_GOAL_CLOSE_X = 59.25, RED_GOAL_CLOSE_Y = 87.77;
    public static double RED_GOAL_FAR_X = 59.25, RED_GOAL_FAR_Y = 56.23;

    public static double BLUE_GOAL_CLOSE_X = 84.75, BLUE_GOAL_CLOSE_Y = 87.77;
    public static double BLUE_GOAL_FAR_X = 84.75, BLUE_GOAL_FAR_Y = 56.23;

}
