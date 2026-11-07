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

    public static double MIN_TURRET_POSITION_IN_DEGREES = -190, MAX_TURRET_POSITION_IN_DEGREES = 190;
    public static double TURRET_SWING_MARGIN_DEGREES = 5; //spare room past the turret's stopping distance before the aim swings a turn around

    public static double TURRET_TICKS_PER_DEGREE = 68.27;

    public static double TURRET_HOME_POSITION_INCREMENT = 200;

    public static double GOAL_NUDGE_INCHES = 1; //gamepad 2 dpad, per press

    public static double TURRET_POSITIONAL_OFFSET = -1.775591;

    //turret axis from drive center, inches, +x forward +y left
    public static double TURRET_OFFSET_X = -2.82, TURRET_OFFSET_Y = 0;

    public static double TURRET_AIM_DIRECTION = -1;

    public static double DEFAULT_FLYWHEEL_VELOCITY = 1450;

    public static double FLYWHEEL_SPEED_ADJUSTMENT_T = 0;
    public static double SHOT_TIME_OF_FLIGHT = 0;

    public static double TURRET_AIM_LEAD_TIME = 0;

    public static List<Double> FLYWHEEL_DISTANCES = new ArrayList<>(Arrays.asList(51.71503320693509, 56.29514009214838, 60.899456934836486, 68.99167060900542, 74.57368259902958, 80.60268703640344, 91.2495467341891, 100.4429764662134));
    public static List<Double> FLYWHEEL_VELOCITIES = new ArrayList<>(Arrays.asList(1300.0, 1330.0, 1380.0, 1440.0, 1455.0, 1485.0, 1590.0, 1665.0));

}
