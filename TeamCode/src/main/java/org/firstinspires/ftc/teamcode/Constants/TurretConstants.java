package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBasePIDFCoefficients;
import org.firstinspires.ftc.teamcode.Utility.Encoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TurretConstants {

    /// Index 0 is the left crservo.
    /// <p>
    /// Index 1 is the right crservo.
    public static DcMotorSimple.Direction[] TURRET_BASE_DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.REVERSE
    };

    public static Encoder.Direction TURRET_ENCODER_DIRECTION = Encoder.Direction.FORWARD;

    public static double TURRET_VOLTAGE_SAMPLE_PERIOD = 0.02;
    public static double TURRET_VOLTAGE_FILTER_TAU = 0.1;
    public static double TURRET_NOMINAL_VOLTS = 12.0;

    public static List<Double> TURRET_PD_POSITIONS = new ArrayList<>(Arrays.asList(-1.0, 1.0));
    public static List<Double> TURRET_KPS = new ArrayList<>(Arrays.asList(0.0, 0.0));
    public static List<Double> TURRET_KDS = new ArrayList<>(Arrays.asList(0.0, 0.0));

    public static List<Double> TURRET_FEEDFORWARD_POSITIONS = new ArrayList<>(Arrays.asList(-1.0, 1.0));
    public static List<Double> TURRET_KFS = new ArrayList<>(Arrays.asList(0.0, 0.0));

    public static TurretBasePIDFCoefficients TURRET_POSITIONAL_COEFFICIENTS = new TurretBasePIDFCoefficients(
            0,
            0,
            new double[] {0, 0},
            new double[] {0, 0},
            0,
            0,
            0,
            0,
            new double[] {0, 0},
            0,
            new double[] {1, 1},
            new double[] {0, Integer.MAX_VALUE},
            1,
            -1, 1
    );

}
