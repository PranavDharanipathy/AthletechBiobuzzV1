package org.firstinspires.ftc.teamcode.shooter.hardware;

import org.firstinspires.ftc.teamcode.shooter.core.AimSolver;
import org.firstinspires.ftc.teamcode.shooter.core.Ballistics;
import org.firstinspires.ftc.teamcode.shooter.core.FlywheelController;
import org.firstinspires.ftc.teamcode.shooter.core.HiveTarget;
import org.firstinspires.ftc.teamcode.shooter.core.HoodModel;
import org.firstinspires.ftc.teamcode.shooter.core.MotorFeedforward;
import org.firstinspires.ftc.teamcode.shooter.core.ShooterCore;
import org.firstinspires.ftc.teamcode.shooter.core.ShotTable;
import org.firstinspires.ftc.teamcode.shooter.core.TurretController;

/**
 * Every robot-specific number for the shooter in one place. Values marked ❓ are placeholders until the
 * characterization / calibration OpModes have been run. Paste the measured values here.
 * See docs/SHOOTER_DESIGN.md §5 for the calibration order.
 */
public final class ShooterConfig {

    private ShooterConfig() {}

    // ------------------------------------------------------------------ hardware map names ❓
    public static String FLYWHEEL_MOTOR = "flywheel";
    public static String TURRET_SERVO_LEFT = "turret_left";
    public static String TURRET_SERVO_RIGHT = "turret_right";
    public static String HOOD_SERVO_LEFT = "hood_left";
    public static String HOOD_SERVO_RIGHT = "hood_right";
    /** Motor port whose encoder input carries the turret encoder (quadrature option). */
    public static String TURRET_ENCODER_PORT = "turret_encoder";
    /** Analog input for an absolute turret sensor (analog option). */
    public static String TURRET_ANALOG = "turret_abs";

    // ------------------------------------------------------------------ directions ❓
    public static boolean FLYWHEEL_REVERSED = false;
    public static boolean TURRET_LEFT_REVERSED = false;
    public static boolean TURRET_RIGHT_REVERSED = false;
    /** +1 if a positive turret encoder count means counter-clockwise (seen from above), else −1. */
    public static int TURRET_ENCODER_SIGN = 1;

    // ------------------------------------------------------------------ sensors
    public static double FLYWHEEL_COUNTS_PER_REV = 28;     // goBILDA 5202 bare motor, 1:1 belt
    public enum TurretSensorType { QUADRATURE, ANALOG }
    public static TurretSensorType TURRET_SENSOR = TurretSensorType.QUADRATURE; // ❓ encoder not decided
    /** Counts per TURRET revolution: REV Through Bore 8192 × 3 (if geared like the pinions) ❓. */
    public static double TURRET_COUNTS_PER_REV = 8192 * 3;
    /** Analog option: volts at sensor 0° and 360°, and sensor revolutions per turret revolution ❓. */
    public static double TURRET_ANALOG_MIN_V = 0.0, TURRET_ANALOG_MAX_V = 3.3, TURRET_ANALOG_RATIO = 3.0;
    /** Turret angle (deg, CCW from robot forward) at OpMode init, i.e. the position it is placed in ❓. */
    public static double TURRET_START_ANGLE_DEG = 0.0;

    // ------------------------------------------------------------------ timing
    public static double VOLTAGE_SAMPLE_PERIOD = 0.05;
    public static double VOLTAGE_FILTER_TAU = 0.05;
    public static double NOMINAL_VOLTS = 12.5;

    // ------------------------------------------------------------------ identified models ❓
    // Flywheel (volts, flywheel rad/s): priors from the goBILDA 5202 datasheet + CAD inertia estimate.
    public static double FLYWHEEL_KS = 0.33, FLYWHEEL_KV = 0.0189, FLYWHEEL_KA = 0.0166;
    // Turret (servo power, turret rad/s): priors for an Axon Max at ~172°/s turret free speed.
    public static double TURRET_KS = 0.05, TURRET_KV = 0.33, TURRET_KA = 0.0040;

    // ------------------------------------------------------------------ builders

    public static FlywheelController.Config flywheel() {
        FlywheelController.Config c = new FlywheelController.Config();
        c.model = new MotorFeedforward(FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA, 2.0);
        return c;
    }

    public static TurretController.Config turret() {
        TurretController.Config c = new TurretController.Config();
        c.model = new MotorFeedforward(TURRET_KS, TURRET_KV, TURRET_KA, 0.05);
        c.maxVelocity = 0.92 * c.model.maxVelocity(1.0 - c.preloadPower);
        return c;
    }

    public static HoodModel.Config hood() { return new HoodModel.Config(); }
    public static AimSolver.Config aim() { return new AimSolver.Config(); }
    public static HiveTarget.Config hive() { return new HiveTarget.Config(); }
    public static Ballistics.Params ball() { return new Ballistics.Params(); }
    public static ShotTable.Config shotTable() { return new ShotTable.Config(); }
    public static ShooterCore.Config core() { return new ShooterCore.Config(); }
}
