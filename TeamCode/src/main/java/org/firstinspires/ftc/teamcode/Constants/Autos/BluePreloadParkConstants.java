package org.firstinspires.ftc.teamcode.Constants.Autos;

import com.acmerobotics.dashboard.config.Config;

/// Blue Preload + Park only: nothing here is shared with another auto. Read at INIT.
/// Poses are {x in, y in, heading deg} in the turret's frame (FieldConstants), the same as the Pedro visualizer's BIOBUZZ
/// field; controls are {x1, y1, x2, y2}
@Config
public final class BluePreloadParkConstants {

    //back on the blue wall at the y of the blue goal in front (84.75, 56.23): the turret's home points straight at it
    public static double[] START = {134.28, 56.23, 180};
    //back 5 in into the LOADING ZONE and off the wall: PARK and LEAVE both count
    public static double[] PARK = {129, 36, 180};

    public static double AUTO_LIMIT_SECONDS = 29.5;
    public static double PATH_TIMEOUT_SECONDS = 8;
    public static double MAX_LOOP_SECONDS = 0.15;
    public static double POSITION_ERROR_INCHES = 1;
    public static double HEADING_ERROR_DEGREES = 3;
    //held flat for the preload (ticks/s), not the distance fit
    public static double FLYWHEEL_VELOCITY = 1465;
    //waits this long for the flywheel to be up to speed, then feeds anyway
    public static double SHOT_TIMEOUT_SECONDS = 4;
    public static double SHOT_FEED_SECONDS = 1.5;
    public static double FLYWHEEL_ERROR_TICKS_PER_SECOND = 50;

    private BluePreloadParkConstants() {}
}
