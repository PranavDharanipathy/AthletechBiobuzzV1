package org.firstinspires.ftc.teamcode.Constants.Autos;

import com.acmerobotics.dashboard.config.Config;

/// Blue Preload + Park only: nothing here is shared with another auto. Read at INIT.
/// Poses are {x in, y in, heading deg} in the turret's frame (FieldConstants), the same as the Pedro visualizer's BIOBUZZ
/// field; controls are {x1, y1, x2, y2}. Start: back on the blue wall, between the LOADING ZONE (y 24-48) and the wall FLOWER (y 91.4-98.1)
@Config
public final class BluePreloadParkConstants {

    //blue's upward CELL at setup is the CLOSE one (manual 10.3.1, 3 NECTAR in it); shots from y > 72 aim CLOSE
    public static boolean FIRST_CELL_CLOSE = true;

    public static double[] START = {134.28, 78, 180};
    //back 5 in into the LOADING ZONE and off the wall: PARK and LEAVE both count
    public static double[] PARK = {129, 36, 180};

    public static double AUTO_LIMIT_SECONDS = 29.5;
    public static double PATH_TIMEOUT_SECONDS = 8;
    public static double MAX_LOOP_SECONDS = 0.15;
    public static double POSITION_ERROR_INCHES = 1;
    public static double HEADING_ERROR_DEGREES = 3;
    public static double SHOT_TIMEOUT_SECONDS = 4;
    public static double SHOT_FEED_SECONDS = 1.5;
    public static double SHOT_READY_DWELL_SECONDS = 0.1;
    public static double FLYWHEEL_ERROR_TICKS_PER_SECOND = 50;
    public static double MAX_SHOOTING_SPEED_INCHES_PER_SECOND = 3;
    public static double MAX_SHOOTING_YAW_DEGREES_PER_SECOND = 10;

    private BluePreloadParkConstants() {}
}
