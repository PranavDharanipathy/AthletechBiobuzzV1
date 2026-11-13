package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class ShooterConstants {

    public static double FAR_ZONE_CLOSE_ZONE_BARRIER = 72;
    public static double CELL_SWITCH_BAND_INCHES = 3;

    public static double MIN_TURRET_POSITION_IN_DEGREES = -200, MAX_TURRET_POSITION_IN_DEGREES = 200;
    public static double TURRET_SWING_MARGIN_DEGREES = 5;

    public static double TURRET_TICKS_PER_DEGREE = 68.27;

    public static double TURRET_HOME_POSITION_INCREMENT = 200;

    public static double GOAL_NUDGE_INCHES = 1;

    public static double TURRET_POSITIONAL_OFFSET = -2.82252;

    public static double TURRET_AIM_DIRECTION = 1;

    public static boolean FLYWHEEL_DISTANCE_FIT = true;
    public static int FLYWHEEL_FIT_DEGREE = 1;
    public static double FLYWHEEL_FIT_EXTRAPOLATE_INCHES = 2.5;
    public static double FLYWHEEL_DEFAULT_SHOOT_VELOCITY = 1450;

    public static double TURRET_AIM_LEAD_TIME = 0;

    public static List<Double> FLYWHEEL_DISTANCES = new ArrayList<>(Arrays.asList(36.0, 42.0, 51.0, 53.0, 58.8));
    public static List<Double> FLYWHEEL_VELOCITIES = new ArrayList<>(Arrays.asList(1450.0, 1450.0, 1460.0, 1490.0, 1520.0));
    public static boolean SOTM_ENABLED = false;
    public static List<Double> SHOT_TIMES_OF_FLIGHT = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    public static double SHOT_LAUNCH_ANGLE_DEGREES = 72, SHOT_RISE_INCHES = 48.05;
    public static double SOTM_TIME_SCALE = 1;
    public static double FLYWHEEL_SOTM_LEAD_SECONDS = 0.15;
    public static double SOTM_ACTUAL_SPEED_BAND = 150;
    public static boolean SHOT_MISS_GATE = true;
    public static double SHOT_MISS_TOLERANCE_INCHES = 4;

}
