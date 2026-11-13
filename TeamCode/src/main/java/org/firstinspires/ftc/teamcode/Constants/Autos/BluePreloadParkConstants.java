package org.firstinspires.ftc.teamcode.Constants.Autos;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class BluePreloadParkConstants {

    //the 2 Tip Garden start: in line with the upward CELL, so this one tips it too
    public static boolean FIRST_CELL_CLOSE = true;

    public static double[] START = {84.75, 134.28, -90};
    public static double[] PARK_CONTROLS = {130, 104, 114, 92};
    public static double[] PARK = {129, 36, 180};

    public static double AUTO_LIMIT_SECONDS = 29.5;
    public static double PATH_TIMEOUT_SECONDS = 8;
    public static double MAX_LOOP_SECONDS = 0.15;
    public static double POSITION_ERROR_INCHES = 1;
    public static double HEADING_ERROR_DEGREES = 3;
    public static double SHOT_TIMEOUT_SECONDS = 4;
    public static double SHOT_FEED_SECONDS = 2;
    //blocker stays open this long after the intake stops, then closes, then it drives
    public static double SHOT_SETTLE_SECONDS = 0.15;
    public static double FLYWHEEL_ERROR_TICKS_PER_SECOND = 50;

    private BluePreloadParkConstants() {}
}
