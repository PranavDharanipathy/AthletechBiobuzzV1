package org.firstinspires.ftc.teamcode.Constants;

public class VisionConstants {

    //used only if the SDK's own address for the Limelight can't be read
    public static String LIMELIGHT_FALLBACK_ADDRESS = "172.29.0.1";

    //Limelight ports: 5800 stream, 5801 web UI, 5807 REST
    public static int LIMELIGHT_BRIDGE_FIRST_PORT = 5800;
    public static int LIMELIGHT_BRIDGE_PORT_COUNT = 10;
    public static int LIMELIGHT_BRIDGE_MAX_CONNECTIONS = 48;

    public static int LIMELIGHT_POLL_RATE_HZ = 100;
    public static int LIMELIGHT_LAB_START_PIPELINE = 0;

    //p<index>_<name>.py / .json in TeamCode/src/main/assets/limelight
    public static String LIMELIGHT_ASSET_DIRECTORY = "limelight";
}
