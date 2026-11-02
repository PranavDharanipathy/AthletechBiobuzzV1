package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;

public class FlywheelConstants {

    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static double FLYWHEEL_VOLTAGE_SAMPLE_PERIOD = 0.02;
    public static double FLYWHEEL_VOLTAGE_FILTER_TAU = 0.1;
    public static double FLYWHEEL_NOMINAL_VOLTS = 12.0;

    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.1,//0.00125
            0.5,//0.00125
            0,
            0.0025,
            0.00035,
            0.00428,//0.00032
            0.2,
            35,
            100,
            120,//100
            0.05,
            -2, 3,
            -0.3, 0.3,
            -0.4, 0.4
    );

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 50;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 20;

}
