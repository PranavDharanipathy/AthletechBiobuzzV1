package org.firstinspires.ftc.teamcode.Constants;

/// BlazeFTC (Utility/Blaze/BlazeSession). Read at INIT. ENABLED off = TeleOps and tuners run on the stock SDK path; the NextFTC
/// autos extend BlazeOpMode, so NextFTC still starts Blaze for them, but with this off it only passes the SDK through

public class BlazeConstants {

    //OFF: on this robot Blaze's proxy made the SDK lose packets (retransmits, ~140 ms loop spikes every 0.85 s), NACKed the
    //Pinpoint and dropped a hub (robot log, Turret Autotuner run 20280508-052931)
    public static boolean ENABLED = false;
    //motor setPower goes straight to Blaze instead of the SDK's blocking write
    public static boolean MOTOR_ACCELERATION = true;
    //Blaze keeps each hub's bulk data fresh in the background, so the loop never waits on a bulk read
    public static boolean BULK_CONTROL_HUB = true, BULK_EXPANSION_HUB = true;
    //packets in flight on the Control Hub: 1 ~500 Hz and stable, 2 ~900 Hz (never on RS485)
    public static int CONTROL_HUB_PACKETS = 1;
    //Pinpoint read in the background; the localizer falls back to a normal SDK read if Blaze's sample is older than this
    public static boolean PINPOINT_ACCELERATION = true;
    public static double PINPOINT_STALE_SECONDS = 0.1;
    //Pinpoint registers are mm; checked against an SDK read at INIT when the robot is away from (0, 0)
    public static boolean PINPOINT_RAW_MM = true;

}
