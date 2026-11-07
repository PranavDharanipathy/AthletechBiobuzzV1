package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utility.Encoder;

@Config
public class TurretConstants {

    /// Index 0 is the left crservo.
    /// <p>
    /// Index 1 is the right crservo.
    public static DcMotorSimple.Direction[] TURRET_BASE_DIRECTIONS = {
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD
    };

    public static Encoder.Direction TURRET_ENCODER_DIRECTION = Encoder.Direction.FORWARD;

    public static boolean TURRET_VOLTAGE_COMPENSATION = true;

    public static double TURRET_VOLTAGE_SAMPLE_PERIOD = TunedConstants.TURRET_VOLTAGE_SAMPLE_PERIOD;
    public static double TURRET_VOLTAGE_FILTER_TAU = TunedConstants.TURRET_VOLTAGE_FILTER_TAU;
    public static double TURRET_NOMINAL_VOLTS = 12.0;

    //turret model, the Turret Autotuner measures these (TunedConstants), gains get worked out from them
    public static double TURRET_GAIN = TunedConstants.TURRET_GAIN; //ticks/s per volt
    public static double TURRET_TIME_CONSTANT = TunedConstants.TURRET_TIME_CONSTANT;
    public static double TURRET_DEAD_TIME = TunedConstants.TURRET_DEAD_TIME;
    public static double TURRET_KINETIC_FRICTION = TunedConstants.TURRET_KINETIC_FRICTION; //volts
    public static double TURRET_BREAKAWAY_FRICTION = TunedConstants.TURRET_BREAKAWAY_FRICTION;

    public static double TURRET_BANDWIDTH = TunedConstants.TURRET_BANDWIDTH; //rad/s, higher = stiffer
    public static double TURRET_DAMPING = TunedConstants.TURRET_DAMPING;
    public static double TURRET_INTEGRAL_RATIO = TunedConstants.TURRET_INTEGRAL_RATIO;
    public static double TURRET_FRICTION_SCALE = TunedConstants.TURRET_FRICTION_SCALE;
    public static double TURRET_MAX_SPEED = TunedConstants.TURRET_MAX_SPEED; //ticks/s the servos top out at, 0 = not measured
    public static double TURRET_MAX_ACCELERATION = TunedConstants.TURRET_MAX_ACCELERATION; //ticks/s^2, the torque limit, 0 = not measured
    public static double TURRET_START_DELAY = TunedConstants.TURRET_START_DELAY; //s to get going from rest, TURRET_DEAD_TIME is while it's moving
    public static double TURRET_MAX_DECELERATION = TunedConstants.TURRET_MAX_DECELERATION; //ticks/s^2 it coasts down at (the servos only brake reversed), 0 = not measured

    public static double TURRET_MAX_SPEED_FRACTION = 0.8; //of measured top speed + acceleration. 0.85+ hunted on the slippery turret and 0.9 ran past the travel in sim
    public static double TURRET_HOLD_TOLERANCE_DEGREES = 0.15;
    public static double TURRET_FRICTION_RAMP_DEGREES = 0.5;
    public static double TURRET_MOVING_DEGREES_PER_SECOND = 1;
    public static double TURRET_INTEGRAL_ZONE_DEGREES = 3;
    public static double TURRET_MAX_INTEGRAL_VOLTS = 1.5;

    //the flywheel shakes the turret: while it shakes, stiffness drops so the servos stop chasing it
    public static boolean TURRET_VIBRATION_REJECTION = true;
    public static double TURRET_VIBRATION_DEADBAND_DEGREES = 0.1; //shake below this is ignored (loop jitter, encoder steps)
    public static double TURRET_VIBRATION_REFERENCE_DEGREES = 0.25; //shake past the deadband that halves the bandwidth
    public static double TURRET_MIN_BANDWIDTH_FRACTION = 0.5;
    public static double TURRET_MIN_BANDWIDTH = TunedConstants.TURRET_SHAKE_BANDWIDTH; //rad/s rejection may soften to, from Turret Autotuner DRIVING (= bandwidth: never soften)

    //aim gate: the blocker only opens once the turret has sat on target, like the shot gate for the flywheel
    public static boolean TURRET_AIM_GATE_ENABLED = true; //TeleOp fired 7 of 13 bursts with the turret up to 10.7 deg off
    public static double TURRET_AIM_GATE_ENTER_DEGREES = 1.0, TURRET_AIM_GATE_HOLD_DEGREES = 2.0;
    public static double TURRET_AIM_GATE_DWELL = 0.06;
    public static double TURRET_AIM_GATE_SMOOTHING = 0.05; //s, takes the shake out of the position it checks

    public static double TURRET_HARD_LIMIT_MARGIN_DEGREES = 5; //past the travel limit by this, it only drives back

    //lanyards: their pull at each angle gets learned (Turret Manual Tuner X sweep) and fed forward.
    //Off until the robot has lanyards. Map lives in FIRST/data/TurretLanyards/map.csv
    public static boolean TURRET_LANYARDS_ENABLED = false;
    public static boolean TURRET_LANYARD_LEARNING = false; //keep learning in every OpMode too (the sweep learns either way)
    public static double TURRET_LANYARD_LEARN_RATE = 3; //share of the feedback moved into the map per second
    public static double TURRET_LANYARD_LEARN_BAND_DEGREES = 45; //only learns while the error is under this
    public static double TURRET_LANYARD_STUCK_BAND_DEGREES = 15; //same, while stopped off target
    public static double TURRET_LANYARD_BIN_DEGREES = 5;
    public static double TURRET_LANYARD_MAX_VOLTS = 6;
    public static double TURRET_LANYARD_SWEEP_DEGREES_PER_SECOND = 30;
    public static double TURRET_LANYARD_SWEEP_DWELL = 1.5; //s stopped at each end, that's how the ends get learned
}
