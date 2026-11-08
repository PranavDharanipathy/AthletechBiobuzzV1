package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utility.Encoder;

@Config
public class TurretConstants {

    //OFF for now: the turret servos only ever get 0, in every OpMode (TeleOps, autos, tuners). The aim still runs and is recorded
    public static boolean TURRET_ENABLED = false;

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

    public static double TURRET_MAX_SPEED_FRACTION = 0.8; //of measured top speed. 1.0 ran 6 deg past the travel in sim once the aim used all of it
    public static double TURRET_ACCELERATION_FRACTION = 0.8; //of measured acceleration
    //off its own plan by this, the plan moves onto where the turret really is (it fell behind, coasted on or stalled) instead of
    //the two fighting to ~0 V. TeleOp log 20280507-230429: 31% of loops 15+ deg off, 2 s stuck 30 deg short
    public static double TURRET_RESYNC_DEGREES = 7;
    //moving: the friction push fades to full reverse over this much lead on the plan (0 = flip at 1 deg, which chopped the speed)
    public static double TURRET_BRAKE_DEGREES = 5;
    //braking ("landing"): on a big move to a still aim it cruises flat out, then predicts its own stop with the model (dead time,
    //lag, coasting, friction, reverse braking: not v^2/2d) and picks the volts that land it on the aim, learning from where it
    //rests. Off: in sim it got there ~10% sooner but overshot 3-5 deg (the old stop: 1-2), because how hard these servos brake
    //reversed has never been measured. SHT Stability Tuner tries it on the robot and keeps it only if it scores better
    public static boolean TURRET_LANDING_ENABLED = false;
    public static boolean TURRET_BRAKE_LEARNING = true;
    //share of the robot's spin the turret is told to cancel (1 = all of it). SHT Stability Tuner learns it from whether the
    //turret lags the spin or runs ahead of it
    public static double TURRET_RATE_FEEDFORWARD_SCALE = 1;
    //within this of the target it just holds (no friction push), and the push builds up over the ramp past it. 0.15 / 0.5
    //rocked it +-3-5 deg around a still target (DRIVING run 20280508-054432); sim with the robot's slide and loop: 0.5 / 1
    //holds under 0.9 deg with no volts flipping, inside the 1 deg aim gate
    public static double TURRET_HOLD_TOLERANCE_DEGREES = 0.5;
    public static double TURRET_FRICTION_RAMP_DEGREES = 1.0;
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

    //wiring: pushed this hard (V) with the encoder not changing at all for this long (s), the encoder (or its data) is dead and
    //the travel limit can't work, so the turret stops until the OpMode restarts. A frozen reading wrapped it round and round
    public static double TURRET_FROZEN_ENCODER_VOLTS = 3, TURRET_FROZEN_ENCODER_SECONDS = 0.5;

    //near the travel ends (target within TURRET_STUCK_END_ZONE_DEGREES of one) a turret stopped short of its target pushes
    //TURRET_STUCK_PUSH_RATE V/s harder, up to TURRET_STUCK_PUSH_MAX V (TurretController.stuckVolts): the lanyards there pull back
    //harder than friction. 0 max = off
    public static double TURRET_STUCK_END_ZONE_DEGREES = 60, TURRET_STUCK_PUSH_RATE = 2, TURRET_STUCK_PUSH_MAX = 1.5;
}
