package org.firstinspires.ftc.teamcode.Constants;

public class BlazeConstants {

    //150-650 ms freezes about every second
    public static boolean ENABLED = false;
    //drive lag and the intake stopping; it also ignores RUN_USING_ENCODER and getPower() goes stale
    public static boolean MOTOR_ACCELERATION = false;
    public static boolean BULK_CONTROL_HUB = true, BULK_EXPANSION_HUB = true;

    public static int CONTROL_HUB_PACKETS = 1;
    public static boolean PINPOINT_ACCELERATION = true;
    public static double PINPOINT_STALE_SECONDS = 0.1;
    public static boolean PINPOINT_RAW_MM = true;

}
