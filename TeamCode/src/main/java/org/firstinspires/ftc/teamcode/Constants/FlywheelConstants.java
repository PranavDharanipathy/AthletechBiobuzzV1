package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;

public class FlywheelConstants {

    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static double FLYWHEEL_VOLTAGE_SAMPLE_PERIOD = TunedConstants.FLYWHEEL_VOLTAGE_SAMPLE_PERIOD;
    public static double FLYWHEEL_VOLTAGE_FILTER_TAU = TunedConstants.FLYWHEEL_VOLTAGE_FILTER_TAU;
    public static double FLYWHEEL_NOMINAL_VOLTS = 12.0;

    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.0686,
            0.072,
            0,
            0.002,
            0.0003,
            0.00475,
            0.42,
            35,
            110,
            110,
            0.05,
            -2, 8,
            -0.3, 0.3,
            -0.4, 0.4
    );

    public static double FLYWHEEL_LOAD_OBSERVER_TIME = 2, FLYWHEEL_MAX_LOAD_VOLTS = 1.5;
    public static double FLYWHEEL_ACCEL_PER_VOLT = 165;

    public static double FLYWHEEL_FEED_BOOST_VOLTS = 0, FLYWHEEL_SHOT_KICK_VOLTS = 0, FLYWHEEL_SHOT_KICK_TIME = 0.08, FLYWHEEL_SHOT_DETECT_DROP = 25;

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 50;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 20;

    public static double FLYWHEEL_TICKS_PER_REV = 28;

    public static boolean SHOT_GATE_ENABLED = true;

    public static double SHOT_GATE_ENTER_BAND_BELOW = TunedConstants.SHOT_GATE_ENTER_BAND, SHOT_GATE_ENTER_BAND_ABOVE = TunedConstants.SHOT_GATE_ENTER_BAND;
    public static double SHOT_GATE_HOLD_BAND_BELOW = TunedConstants.SHOT_GATE_HOLD_BAND, SHOT_GATE_HOLD_BAND_ABOVE = TunedConstants.SHOT_GATE_HOLD_BAND;
    public static double SHOT_GATE_DWELL = TunedConstants.SHOT_GATE_DWELL;
    public static double SHOT_GATE_MIN_OPEN = 0.05;
    public static double SHOT_GATE_PULSE = 0;

}
