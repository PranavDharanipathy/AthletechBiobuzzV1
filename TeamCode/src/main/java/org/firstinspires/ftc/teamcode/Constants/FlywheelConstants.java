package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;

public class FlywheelConstants {

    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static double FLYWHEEL_VOLTAGE_SAMPLE_PERIOD = 0.02;
    public static double FLYWHEEL_VOLTAGE_FILTER_TAU = 0.1;
    public static double FLYWHEEL_NOMINAL_VOLTS = 12.0;

    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.0016,//0.00125
            0.0013,//0.00125
            0,
            0.000915,
            0.0000635,
            0.00037,//0.00032
            0,
            50,
            100,
            60,//100
            0.05,
            -1, 1,
            -0.3, 0.3,
            -0.1, 0.1
    );

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 50;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 20;

}
