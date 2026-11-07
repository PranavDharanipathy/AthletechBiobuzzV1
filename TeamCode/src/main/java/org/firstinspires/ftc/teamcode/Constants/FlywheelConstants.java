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
            -2, 8, //max P: after a ball it pushes everything the battery has left (P passes 4.5 V only beyond ~65 ticks/s slow, so
                   //near the target nothing changes). TeleOp recovered 28% slower with the turret on: P was stuck at 3 V, power 0.85
            -0.3, 0.3,
            -0.4, 0.4
    );

    //steady load the volts reading can't see (a new actuator, drag): learned near the target from the speed it holds for the
    //volts it sent, added back. Slow on purpose: on 20280507-230429 there was none near the target (the turret's ~1 V only
    //showed in recoveries, which max P now covers) and 0.3 s just added 0.4 V of jitter. 0 time = off
    public static double FLYWHEEL_LOAD_OBSERVER_TIME = 2, FLYWHEEL_MAX_LOAD_VOLTS = 1.5;
    public static double FLYWHEEL_ACCEL_PER_VOLT = 165; //ticks/s^2 per spare volt, same session with the turret quiet

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
