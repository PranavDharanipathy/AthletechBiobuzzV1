package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;

public class FlywheelConstants {

    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static double FLYWHEEL_VOLTAGE_SAMPLE_PERIOD = 0.02;
    public static double FLYWHEEL_VOLTAGE_FILTER_TAU = 0.1;
    public static double FLYWHEEL_NOMINAL_VOLTS = 12.0;

    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.024,
            0.009,
            0,
            0.002,
            0.00035,
            0.00478,
            0.4,
            35,
            100,
            80,
            0.05,
            -2, 3,
            -0.3, 0.3,
            -0.4, 0.4
    );

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 50;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 20;

}
