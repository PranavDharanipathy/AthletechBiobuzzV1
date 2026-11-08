package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utility.Encoder;

@Config
public class TurretConstants {

    //enables/disables turret
    public static boolean TURRET_ENABLED = false;

    public static DcMotorSimple.Direction[] TURRET_BASE_DIRECTIONS = {
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD
    };

    public static Encoder.Direction TURRET_ENCODER_DIRECTION = Encoder.Direction.FORWARD;

    public static boolean TURRET_VOLTAGE_COMPENSATION = true;

    public static double TURRET_VOLTAGE_SAMPLE_PERIOD = TunedConstants.TURRET_VOLTAGE_SAMPLE_PERIOD;
    public static double TURRET_VOLTAGE_FILTER_TAU = TunedConstants.TURRET_VOLTAGE_FILTER_TAU;
    public static double TURRET_NOMINAL_VOLTS = 12.0;

    public static double TURRET_GAIN = TunedConstants.TURRET_GAIN;
    public static double TURRET_TIME_CONSTANT = TunedConstants.TURRET_TIME_CONSTANT;
    public static double TURRET_DEAD_TIME = TunedConstants.TURRET_DEAD_TIME;
    public static double TURRET_KINETIC_FRICTION = TunedConstants.TURRET_KINETIC_FRICTION;
    public static double TURRET_BREAKAWAY_FRICTION = TunedConstants.TURRET_BREAKAWAY_FRICTION;

    public static double TURRET_BANDWIDTH = TunedConstants.TURRET_BANDWIDTH;
    public static double TURRET_DAMPING = TunedConstants.TURRET_DAMPING;
    public static double TURRET_INTEGRAL_RATIO = TunedConstants.TURRET_INTEGRAL_RATIO;
    public static double TURRET_FRICTION_SCALE = TunedConstants.TURRET_FRICTION_SCALE;
    public static double TURRET_MAX_SPEED = TunedConstants.TURRET_MAX_SPEED;
    public static double TURRET_MAX_ACCELERATION = TunedConstants.TURRET_MAX_ACCELERATION;
    public static double TURRET_START_DELAY = TunedConstants.TURRET_START_DELAY;
    public static double TURRET_MAX_DECELERATION = TunedConstants.TURRET_MAX_DECELERATION;

    public static double TURRET_MAX_SPEED_FRACTION = 0.8;
    public static double TURRET_ACCELERATION_FRACTION = 0.8;
    public static double TURRET_RESYNC_DEGREES = 7;
    public static double TURRET_BRAKE_DEGREES = 5;
    public static boolean TURRET_LANDING_ENABLED = false;
    public static boolean TURRET_BRAKE_LEARNING = true;
    public static double TURRET_RATE_FEEDFORWARD_SCALE = 1;
    public static double TURRET_HOLD_TOLERANCE_DEGREES = 0.5;
    public static double TURRET_FRICTION_RAMP_DEGREES = 1.0;
    public static double TURRET_MOVING_DEGREES_PER_SECOND = 1;
    public static double TURRET_INTEGRAL_ZONE_DEGREES = 3;
    public static double TURRET_MAX_INTEGRAL_VOLTS = 1.5;

    public static boolean TURRET_VIBRATION_REJECTION = true;
    public static double TURRET_VIBRATION_DEADBAND_DEGREES = 0.1;
    public static double TURRET_VIBRATION_REFERENCE_DEGREES = 0.25;
    public static double TURRET_MIN_BANDWIDTH_FRACTION = 0.5;
    public static double TURRET_MIN_BANDWIDTH = TunedConstants.TURRET_SHAKE_BANDWIDTH;

    public static boolean TURRET_AIM_GATE_ENABLED = true;
    public static double TURRET_AIM_GATE_ENTER_DEGREES = 1.0, TURRET_AIM_GATE_HOLD_DEGREES = 2.0;
    public static double TURRET_AIM_GATE_DWELL = 0.06;
    public static double TURRET_AIM_GATE_SMOOTHING = 0.05;

    public static double TURRET_HARD_LIMIT_MARGIN_DEGREES = 5;

    public static boolean TURRET_LANYARDS_ENABLED = false;
    public static boolean TURRET_LANYARD_LEARNING = false;
    public static double TURRET_LANYARD_LEARN_RATE = 3;
    public static double TURRET_LANYARD_LEARN_BAND_DEGREES = 45;
    public static double TURRET_LANYARD_STUCK_BAND_DEGREES = 15;
    public static double TURRET_LANYARD_BIN_DEGREES = 5;
    public static double TURRET_LANYARD_MAX_VOLTS = 6;
    public static double TURRET_LANYARD_SWEEP_DEGREES_PER_SECOND = 30;
    public static double TURRET_LANYARD_SWEEP_DWELL = 1.5;

    public static double TURRET_FROZEN_ENCODER_VOLTS = 3, TURRET_FROZEN_ENCODER_SECONDS = 0.5;

    public static double TURRET_STUCK_END_ZONE_DEGREES = 60, TURRET_STUCK_PUSH_RATE = 2, TURRET_STUCK_PUSH_MAX = 1.5;
}
