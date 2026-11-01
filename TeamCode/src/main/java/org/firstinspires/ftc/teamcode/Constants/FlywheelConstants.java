package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;

public class FlywheelConstants {

    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static double FLYWHEEL_VOLTAGE_SAMPLE_PERIOD = 0.02;
    public static double FLYWHEEL_VOLTAGE_FILTER_TAU = 0.1;
    public static double FLYWHEEL_NOMINAL_VOLTS = 12.0;

    /**
     * Flywheel.update() sums p + i + d + v + s as VOLTS and divides by the battery voltage
     * (VoltageCompensator.toPower). The gains below were tuned as motor POWER (last season's
     * Flywheel used power = p + i + d + v + s directly). For example kv = 0.00037 power per tick/s is
     * ≈ 1 / 2800, the free speed of a 6000 rpm, 28-count motor; in volts that would be ≈ 12 / 2800.
     * Each power-unit value (and its clamp) is therefore multiplied by 12 V, so it means the same
     * thing at 12 V and is battery-compensated elsewhere. Gains are unchanged in feel: divide by 12
     * to compare with the old numbers.
     */
    private static final double POWER_TO_VOLTS = 12.0;

    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.0016 * POWER_TO_VOLTS,//0.00125
            0.0013 * POWER_TO_VOLTS,//0.00125
            0,
            0.000915 * POWER_TO_VOLTS,
            0.0000635 * POWER_TO_VOLTS,
            0.00037 * POWER_TO_VOLTS,//0.00032
            0,
            50,
            100,
            60,//100
            0.05,
            -1 * POWER_TO_VOLTS, 1 * POWER_TO_VOLTS,
            -0.3 * POWER_TO_VOLTS, 0.3 * POWER_TO_VOLTS,
            -0.1 * POWER_TO_VOLTS, 0.1 * POWER_TO_VOLTS
    );

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 50;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 20;

    /**
     * Fire gate (Flywheel.isReady): window-averaged speed within this fraction of the target.
     * 1 % of exit speed ≈ 1.8 in at the target from 3 m (docs/PHYSICS_ERROR_BUDGET.md). The older
     * ±50 ticks/s margin above is ±2.9 % at 3700 rpm, which spends the whole vertical window.
     */
    public static double FLYWHEEL_READY_FRACTION = 0.01;

    /** Averaging window for the ready gate, s. 100 ms gives ≈ 0.25 % quantization noise at 3700 rpm. */
    public static double FLYWHEEL_READY_WINDOW_SECONDS = 0.10;

}
