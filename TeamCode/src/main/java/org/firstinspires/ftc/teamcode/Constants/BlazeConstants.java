package org.firstinspires.ftc.teamcode.Constants;

public class BlazeConstants {

    //match TeleOps: drive on each Pinpoint sample, shooter motors on each Expansion Hub bulk read (Utility/Blaze/BlazeSession)
    public static boolean ENABLED = false;
    //Blaze's own motor writes: the intake's PID sits on RUN_USING_ENCODER, which they skip (under half speed, 2028-05-12)
    public static boolean MOTOR_ACCELERATION = false;
    //1 = a read every ~2 ms, 2 = ~1 ms (the README says no higher)
    public static int EXPANSION_HUB_PACKETS = 1;

}
