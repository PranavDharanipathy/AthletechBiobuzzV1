package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HoodConstants {

    public static Servo.Direction LEFT_HOOD_ANGLER_SERVO_DIRECTION = Servo.Direction.REVERSE;

    /** The active hood range (HoodAngler.setSafePosition). ShooterConstants' pair is last season's. */
    public static double HOOD_ANGLER_MIN_POSITION = 1;
    public static double HOOD_ANGLER_MAX_POSITION = 0;

    /**
     * Per-servo position offsets for the two mirrored hood servos (0 = none). Tune in HoodAnglerTesting:
     * hold a position and adjust one trim until neither servo strains (lowest current, no buzz).
     */
    public static double LEFT_HOOD_TRIM = 0;
    public static double RIGHT_HOOD_TRIM = 0;
}
