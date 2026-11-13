package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utility.Encoder;

@Config
public class TurretConstants {

    //enables/disables turret
    public static boolean TURRET_ENABLED = true;

    public static DcMotorSimple.Direction[] TURRET_BASE_DIRECTIONS = {
            DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.FORWARD
    };

    public static Encoder.Direction TURRET_ENCODER_DIRECTION = Encoder.Direction.FORWARD;

    //off: both turret servos are on hub servo ports, a regulated supply, so dividing by the battery pushed ~10-20% harder late in a match
    public static boolean TURRET_VOLTAGE_COMPENSATION = false;

    public static double TURRET_VOLTAGE_SAMPLE_PERIOD = TunedConstants.TURRET_VOLTAGE_SAMPLE_PERIOD;
    public static double TURRET_VOLTAGE_FILTER_TAU = TunedConstants.TURRET_VOLTAGE_FILTER_TAU;
    public static double TURRET_NOMINAL_VOLTS = 12.0;

    //measured by the Turret Autotuner
    public static double TURRET_GAIN = TunedConstants.TURRET_GAIN, TURRET_KINETIC_FRICTION = TunedConstants.TURRET_KINETIC_FRICTION, TURRET_BREAKAWAY_FRICTION = TunedConstants.TURRET_BREAKAWAY_FRICTION;

    public static double TURRET_KP_FAR = TunedConstants.TURRET_KP_FAR, TURRET_KP_CLOSE = TunedConstants.TURRET_KP_CLOSE, TURRET_P_SWITCH_DEGREES = 18.25;
    public static double TURRET_KI_FAR = TunedConstants.TURRET_KI_FAR, TURRET_KI_CLOSE = TunedConstants.TURRET_KI_CLOSE, TURRET_I_CLOSE_DEGREES = 0.76, TURRET_I_ZONE_DEGREES = 18.25;
    public static double TURRET_KI_SMASH = 0.85, TURRET_MAX_I = TunedConstants.TURRET_MAX_I;
    public static double TURRET_KD_FAR = TunedConstants.TURRET_KD_FAR, TURRET_KD_CLOSE = TunedConstants.TURRET_KD_CLOSE, TURRET_D_SWITCH_DEGREES = 15.2;
    public static double TURRET_D_MIN_DEGREES = 0.076, TURRET_D_MAX_DEGREES = 48.7, TURRET_D_FILTER = 1;
    public static double TURRET_VELOCITY_FILTER = 1;
    public static double TURRET_K_HOLD_VOLTS = TunedConstants.TURRET_K_HOLD_VOLTS, TURRET_HOLD_DECAY_DEGREES = 350;
    public static double TURRET_STRIBECK_DEGREES_PER_SECOND = TunedConstants.TURRET_STRIBECK_DEGREES_PER_SECOND, TURRET_FRICTION_DEADBAND_DEGREES = TunedConstants.TURRET_FRICTION_DEADBAND_DEGREES;
    public static double TURRET_FRICTION_RELEASE_DEGREES = 2, TURRET_FRICTION_RAMP_DEGREES_PER_SECOND = TunedConstants.TURRET_FRICTION_RAMP_DEGREES_PER_SECOND;
    //holding near a still aim: short timed pushes, their length learned from where each one lands (real logs 2028-05-12: 50-100 ms at
    //2.3 V moved it ~0.7 deg; pushing till it's seen moving threw it ~10 deg)
    public static boolean TURRET_PULSE_ENABLED = true;
    public static double TURRET_PULSE_ZONE_DEGREES = TunedConstants.TURRET_PULSE_ZONE_DEGREES, TURRET_PULSE_AIM_RATE_DEGREES_PER_SECOND = 15, TURRET_PULSE_STILL_DEGREES_PER_SECOND = 4, TURRET_PULSE_ENTER_DEGREES_PER_SECOND = 30;
    public static double TURRET_PULSE_OVER_BREAKAWAY_VOLTS = TunedConstants.TURRET_PULSE_OVER_BREAKAWAY_VOLTS, TURRET_PULSE_SECONDS = 0.07, TURRET_PULSE_MIN_SECONDS = 0.03, TURRET_PULSE_MAX_SECONDS = 0.15;
    public static double TURRET_PULSE_WAIT_SECONDS = TunedConstants.TURRET_PULSE_WAIT_SECONDS;
    //that many pushes in a row without one count = dead encoder (pushes never stay over the cutoff for long)
    public static int TURRET_PULSE_DEAD_ENCODER_PUSHES = 8;
    //friction learned in the match from the pulses (full-length push that didn't move: up, shortest that overshot: down)
    public static double TURRET_FRICTION_LEARN_STEP_VOLTS = 0.03, TURRET_FRICTION_LEARN_MAX_VOLTS = 0.6;
    public static boolean TURRET_FRICTION_FROM_VELOCITY = true;
    //breakaway push while the encoder says it's still: there's no velocity loop to wind up and break it free
    public static boolean TURRET_STRIBECK_FROM_MEASURED = true;

    public static double TURRET_RATE_FEEDFORWARD_SCALE = TunedConstants.TURRET_RATE_FEEDFORWARD_SCALE;

    public static boolean TURRET_AIM_GATE_ENABLED = true;
    public static double TURRET_AIM_GATE_ENTER_DEGREES = 1.0, TURRET_AIM_GATE_HOLD_DEGREES = 2.0;
    public static double TURRET_AIM_GATE_DWELL = 0.06;
    public static double TURRET_AIM_GATE_SMOOTHING = 0.05;

    public static double TURRET_HARD_LIMIT_MARGIN_DEGREES = 5;
    public static double TURRET_END_DECEL_DEGREES_PER_S2 = 900, TURRET_END_LAG_SECONDS = 0.12;

    //dead encoder: pushing this much over breakaway with no count; a turret 40% stickier than tuned would trip it too
    public static double TURRET_FROZEN_ENCODER_OVER_BREAKAWAY_VOLTS = 0.5, TURRET_FROZEN_ENCODER_SECONDS = 0.3;
    //the servos ignore commands for ~1.1 s after an OpMode starts (every log since 2028-05-07): no dead-encoder check until then
    public static double TURRET_SERVO_WAKE_SECONDS = 1.3;
    //push cap until the encoder has counted once this OpMode, so an unplugged one can't spin it far before the cutoff
    public static double TURRET_UNCOUNTED_OVER_BREAKAWAY_VOLTS = 1.0;
}
