package org.firstinspires.ftc.teamcode.Constants.Autos;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class BlueTwoTipGardenConstants {

    public static boolean FIRST_CELL_CLOSE = true;

    public static double[] START = {84.75, 134.28, -90};
    public static double[] GARDEN = {135.36, 122, 90};
    public static double[] GARDEN_CONTROLS = {110, 110, 110, 105};
    public static double[] SWEEP_END = {135.36, 133.28, 90};
    public static double[] PARK_CONTROLS = {115, 114, 115, 54};
    public static double[] PARK = {129, 36, 180};

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

    private BlueTwoTipGardenConstants() {}
}
