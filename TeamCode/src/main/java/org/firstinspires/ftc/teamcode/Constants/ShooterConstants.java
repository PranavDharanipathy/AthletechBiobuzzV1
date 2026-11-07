package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class ShooterConstants {

    /// Y-Point that differentiates the turret pointing at the goal far/close position.
    /// <p>
    /// Middle of the field: y above it (away from the audience) aims at the CLOSE point, below it at the FAR point.
    public static double FAR_ZONE_CLOSE_ZONE_BARRIER = 72;

    //20 deg past straight back each way
    public static double MIN_TURRET_POSITION_IN_DEGREES = -200, MAX_TURRET_POSITION_IN_DEGREES = 200;
    public static double TURRET_SWING_MARGIN_DEGREES = 5; //how far the aim goes past an end before the turret swings a turn around

    public static double TURRET_TICKS_PER_DEGREE = 68.27;

    public static double TURRET_HOME_POSITION_INCREMENT = 200;

    public static double GOAL_NUDGE_INCHES = 1; //gamepad 2 dpad, per press

    public static double TURRET_POSITIONAL_OFFSET = -2.82252; //how forward the turret is from the center of the drivetrain (- is back, + is forward)

    //+1: encoder ticks go up as the turret turns counterclockwise (seen from above). -1 had it turning with the robot's spin, twice as fast
    public static double TURRET_AIM_DIRECTION = 1;

    public static double FLYWHEEL_SHOOT_VELOCITY = 1450;

    public static double TURRET_AIM_LEAD_TIME = 0;

    public static List<Double> FLYWHEEL_DISTANCES = new ArrayList<>(Arrays.asList(51.71503320693509, 56.29514009214838, 60.899456934836486, 68.99167060900542, 74.57368259902958, 80.60268703640344, 91.2495467341891, 100.4429764662134));
    public static List<Double> FLYWHEEL_VELOCITIES = new ArrayList<>(Arrays.asList(1300.0, 1330.0, 1380.0, 1440.0, 1455.0, 1485.0, 1590.0, 1665.0));
    //shoot on the move: s from leaving the turret to the goal at each FLYWHEEL_DISTANCES distance. All 0 = no SOTM
    public static List<Double> SHOT_TIMES_OF_FLIGHT = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    //times the virtual goal gets moved (its distance changes the time of flight)
    public static int SOTM_ITERATIONS = 3;

}
