package org.firstinspires.ftc.teamcode.Constants.Autos;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class RedTwoTipGardenConstants {

    public static boolean FIRST_CELL_CLOSE = false;

    public static double[] START = {59.25, 9.72, 90};
    public static double[] GARDEN = {8.64, 22, -90};
    public static double[] GARDEN_CONTROLS = {34, 34, 34, 39};
    public static double[] SWEEP_END = {8.64, 10.72, -90};
    public static double[] PARK_CONTROLS = {26, 30, 26, 90};
    public static double[] PARK = {15, 108, 0};

    public static double SWEEP_SPEED_INCHES_PER_SECOND = 15;
    public static double SWEEP_DWELL_SECONDS = 0.6;

    public static double AUTO_LIMIT_SECONDS = 29.5;
    public static double PATH_TIMEOUT_SECONDS = 8;
    public static double MAX_LOOP_SECONDS = 0.15;
    public static double POSITION_ERROR_INCHES = 1;
    public static double HEADING_ERROR_DEGREES = 3;
    public static double SHOT_TIMEOUT_SECONDS = 4;
    public static double SHOT_FEED_SECONDS = 1.5;
    public static double FLYWHEEL_ERROR_TICKS_PER_SECOND = 50;

    private RedTwoTipGardenConstants() {}
}
