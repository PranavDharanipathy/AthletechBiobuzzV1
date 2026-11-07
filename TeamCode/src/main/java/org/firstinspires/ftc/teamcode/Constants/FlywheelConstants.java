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
            -2, 4.5, //max P: P was maxed 88-92% of every recovery in bursts, ShotGate Tuner says up to 4.9
            -0.3, 0.3,
            -0.4, 0.4
    );

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 50;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 20;

    public static double FLYWHEEL_TICKS_PER_REV = 28;

    //TeleOp dumps the balls staged on the blocker ~0.15 s apart, so the later ones hit a flywheel 30-150 ticks/s slow
    //(session 20280507-230429). The gate holds them until it's back at speed
    public static boolean SHOT_GATE_ENABLED = true;

    public static double SHOT_GATE_ENTER_BAND_BELOW = TunedConstants.SHOT_GATE_ENTER_BAND, SHOT_GATE_ENTER_BAND_ABOVE = TunedConstants.SHOT_GATE_ENTER_BAND;
    public static double SHOT_GATE_HOLD_BAND_BELOW = TunedConstants.SHOT_GATE_HOLD_BAND, SHOT_GATE_HOLD_BAND_ABOVE = TunedConstants.SHOT_GATE_HOLD_BAND;
    public static double SHOT_GATE_DWELL = TunedConstants.SHOT_GATE_DWELL;
    public static double SHOT_GATE_MIN_OPEN = 0.05;
    public static double SHOT_GATE_PULSE = 0; //bursts: the gate alone kept every ball within -22 at 0.54 balls/s

}
