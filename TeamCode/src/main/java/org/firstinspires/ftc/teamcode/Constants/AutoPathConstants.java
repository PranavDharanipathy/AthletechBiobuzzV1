package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class AutoPathConstants {

    public static boolean REVERSE_STRAFE = true;
    public static double AUTO_LIMIT_SECONDS = 29.5;
    public static double PATH_TIMEOUT_SECONDS = 8;
    public static double SHOT_TIMEOUT_SECONDS = 4;
    public static double SHOT_FEED_SECONDS = 1.5;
    public static double SHOT_READY_DWELL_SECONDS = 0.1;
    public static double FLYWHEEL_ERROR_TICKS_PER_SECOND = 50;
    public static double POSITION_ERROR_INCHES = 1;
    public static double HEADING_ERROR_DEGREES = 3;
    public static double MAX_SHOOTING_SPEED_INCHES_PER_SECOND = 3;
    public static double MAX_SHOOTING_YAW_DEGREES_PER_SECOND = 10;
    public static double MAX_LOOP_SECONDS = 0.15;

    private AutoPathConstants() {}
}
