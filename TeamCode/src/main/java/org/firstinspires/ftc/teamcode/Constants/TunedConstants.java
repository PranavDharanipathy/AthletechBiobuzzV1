package org.firstinspires.ftc.teamcode.Constants;

//written by :Sim:analyzeTuning from TunerTeleOp + Turret Autotuner data on every build, don't edit by hand
public class TunedConstants {

    public static String SESSIONS = "20280507-230429";

    public static double LOOP_TIME_MS = 29;

    public static double FLYWHEEL_VOLTAGE_SAMPLE_PERIOD = 0.02;
    public static double FLYWHEEL_VOLTAGE_FILTER_TAU = 0.04;

    public static double TURRET_VOLTAGE_SAMPLE_PERIOD = 0.02;
    public static double TURRET_VOLTAGE_FILTER_TAU = 0.04;

    public static double FLYWHEEL_FILTER_MIN_BANDWIDTH = 0.25;
    public static double FLYWHEEL_FILTER_MAX_BANDWIDTH = 13.6;
    public static double FLYWHEEL_FILTER_SCORE_MEMORY = 0.8;
    public static double FLYWHEEL_FILTER_STALE_TIME = 0.112;

    public static double TURRET_FILTER_MIN_BANDWIDTH = 0.5;
    public static double TURRET_FILTER_MAX_BANDWIDTH = 32;
    public static double TURRET_FILTER_SCORE_MEMORY = 0.9;
    public static double TURRET_FILTER_STALE_TIME = 0.085;

    public static double SHOT_GATE_ENTER_BAND = 26;
    public static double SHOT_GATE_HOLD_BAND = 52;
    public static double SHOT_GATE_DWELL = 0.06;

    public static double TURRET_GAIN = 11374.8;
    public static double TURRET_TIME_CONSTANT = 0.046;
    public static double TURRET_DEAD_TIME = 0.0361;
    public static double TURRET_KINETIC_FRICTION = 1.011;
    public static double TURRET_BREAKAWAY_FRICTION = 1.163;
    public static double TURRET_BANDWIDTH = 3.2;
    public static double TURRET_DAMPING = 0.9;
    public static double TURRET_INTEGRAL_RATIO = 0.3;
    public static double TURRET_FRICTION_SCALE = 1;
    public static double TURRET_MAX_SPEED = 26233.1;
    public static double TURRET_MAX_ACCELERATION = 118456;
    public static double TURRET_START_DELAY = 0.0828;
    public static double TURRET_MAX_DECELERATION = 71701.9;
    public static double TURRET_SHAKE_BANDWIDTH = 3.2;
    public static String TURRET_AUTOTUNE = "20280507-210857-driving";

    public static double LOOP_TIME_P95_MS = 41.8;
    public static double FLYWHEEL_VIBRATION_RMS = 13.2;
    public static double TURRET_VIBRATION_RMS = Double.NaN;
}
