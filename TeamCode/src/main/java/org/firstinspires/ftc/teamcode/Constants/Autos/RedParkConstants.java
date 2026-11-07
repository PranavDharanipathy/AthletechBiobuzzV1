package org.firstinspires.ftc.teamcode.Constants.Autos;

import com.acmerobotics.dashboard.config.Config;

/// Red Park only: nothing here is shared with another auto. Read at INIT.
/// Poses are {x in, y in, heading deg} in the turret's frame (FieldConstants), the same as the Pedro visualizer's BIOBUZZ
/// field; controls are {x1, y1, x2, y2}. Start: back on the red wall, between the wall FLOWER (y 43.7-50.4) and the LOADING ZONE (y 96-120)
@Config
public final class RedParkConstants {

    public static double[] START = {9.72, 64, 0};
    //back 5 in into the LOADING ZONE and off the wall: PARK and LEAVE both count
    public static double[] PARK = {15, 108, 0};

    public static double AUTO_LIMIT_SECONDS = 29.5;
    public static double PATH_TIMEOUT_SECONDS = 8;
    public static double MAX_LOOP_SECONDS = 0.15;
    public static double POSITION_ERROR_INCHES = 1;
    public static double HEADING_ERROR_DEGREES = 3;

    private RedParkConstants() {}
}
