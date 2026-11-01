package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class ShooterConstants {

    /// Y-Point that differentiates the turret pointing at the goal far/close position.
    public static double FAR_ZONE_CLOSE_ZONE_BARRIER = -35;

    public static double MIN_TURRET_POSITION_IN_DEGREES = -90, MAX_TURRET_POSITION_IN_DEGREES = 90;

    /// Unused, last season's hood. The active hood range is HoodConstants.HOOD_ANGLER_MIN/MAX_POSITION.
    public static double HOOD_ANGLER_MIN_POSITION = 0.78;
    public static double HOOD_ANGLER_MAX_POSITION = 0;

    //arbitrary positions
    public static double HOOD_CLOSE_POSITION = 0.25;
    public static double HOOD_FAR_POSITION = 0.15;

    public static double TURRET_TICKS_PER_DEGREE = 65.738;

    public static double TURRET_HOME_POSITION_INCREMENT = 200;

    public static double GOAL_X_POSITION_INCREMENT = 1;
    public static double GOAL_Y_POSITION_INCREMENT = 1;

    public static double TURRET_POSITIONAL_OFFSET = -1.775591;

    public static List<Double> HOOD_DISTANCES = new ArrayList<>(Arrays.asList(51.71503320693509, 56.29514009214838, 60.899456934836486, 68.99167060900542, 74.57368259902958, 80.60268703640344, 91.2495467341891, 100.4429764662134));
    public static List<Double> FLYWHEEL_VELOCITIES = new ArrayList<>(Arrays.asList(1300.0, 1330.0, 1380.0, 1440.0, 1455.0, 1485.0, 1590.0, 1665.0));
    public static List<Double> HOOD_POSITIONS = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

}
