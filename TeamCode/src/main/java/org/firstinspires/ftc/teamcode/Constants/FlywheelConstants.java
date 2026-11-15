package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;

public class FlywheelConstants {

    public static DcMotorSimple.Direction FLYWHEEL_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static double FLYWHEEL_VOLTAGE_SAMPLE_PERIOD = TunedConstants.FLYWHEEL_VOLTAGE_SAMPLE_PERIOD;
    public static double FLYWHEEL_VOLTAGE_FILTER_TAU = TunedConstants.FLYWHEEL_VOLTAGE_FILTER_TAU;
    public static double FLYWHEEL_NOMINAL_VOLTS = 12.0;

    //kp close 0.072 -> 0.09, kd 0.0003 -> 0.00027 (2026-11-14, Sim on the 2028-05-15 logs' balls: FlywheelTuning)
    public static FlywheelPIDVSCoefficients FLYWHEEL_PIDVS_COEFFICIENTS = new FlywheelPIDVSCoefficients(
            0.0686,
            0.072,
            0,
            0.002,
            0.0003,
            0.00475,
            0.42,
            35,
            110,
            110,
            0.05,
            -2, 8,
            -0.3, 0.3,
            -0.4, 0.4
    );

    public static double FLYWHEEL_LOAD_OBSERVER_TIME = 2, FLYWHEEL_MAX_LOAD_VOLTS = 1.5;
    public static double FLYWHEEL_ACCEL_PER_VOLT = 165;

    //control on FlywheelObserver's speed (volts sent + the hub's reading carried over its lag) instead of the filtered one: the filter
    //lags and runs past the end of a recovery, and the hub can't brake the wheel back down (2028-05-15: +30..+55 for ~0.4 s after balls)
    public static boolean FLYWHEEL_OBSERVER = false;
    //cut the power and it only coasts down (~100 ticks/s^2): the most the model lets it slow, in volts' worth
    public static double FLYWHEEL_COAST_VOLTS = 0.7;
    //the hub's speed is a 50 ms average read a bit later; gain: how much of each surprise goes into the speed, load gain into the load
    public static double FLYWHEEL_OBSERVER_LAG = 0.035, FLYWHEEL_OBSERVER_GAIN = 0.35, FLYWHEEL_OBSERVER_LOAD_GAIN = 0.0005;
    //a reading this far under the model while a ball can hit = a ball (all of it goes into the speed); balls past the blocker hit up to
    //CONTACT_AFTER_CLOSE after it shuts
    public static double FLYWHEEL_CONTACT_DROP = 45, FLYWHEEL_CONTACT_AFTER_CLOSE = 0.6;

    //FeedGate: holds a ball back when the wheel won't be inside the shot's window when that ball gets there. Off in the match
    //TeleOps until an SHT run backs it (the SHT Stability Tuner turns it on): the old ShotGate stuttered the autos (2026-11-07)
    public static boolean FEED_GATE = false;
    public static double FEED_GATE_COMMIT = 0.12, FEED_GATE_EARLY = 0.05;
    //the window: CellGeometry's (drag-free, aim at the top middle of the rectangle) x WINDOW_SCALE, kept in MIN..MAX; DEFAULT when
    //the model has no way in from there. SCALE is exit gain: 0.3-1 all fit the 2028-05-15 video, 0.6 until the SHT probes say
    public static double FEED_GATE_WINDOW_SCALE = 0.6, FEED_GATE_MIN_BAND = 25, FEED_GATE_MAX_BAND = 80;
    public static double FEED_GATE_DEFAULT_BELOW = 60, FEED_GATE_DEFAULT_ABOVE = 40;

    public static double FLYWHEEL_FEED_BOOST_VOLTS = 0, FLYWHEEL_SHOT_KICK_VOLTS = 0, FLYWHEEL_SHOT_KICK_TIME = 0.08, FLYWHEEL_SHOT_DETECT_DROP = 25;
    //a push just before each ball should hit, from when the blocker opened (logs 2028-05-10..12: first ball 0.24 s after the trigger,
    //then one every 0.34 s, medians of 131 bursts); the times are learned from every contact, the SHT tuner tunes the push
    //Flywheel: while feeding, aim this share of the shot's fast-side window over the target (0 = off). Balls come ~0.17 s apart
    //(2028-05-15 logs), so every one after the first hits slow, but with the table's own error in, the Sim says it only trades first
    //balls long for later ones short (14.7% out at 0, 16.1% at 0.3). The SHT tries it (burst_lift)
    public static double FLYWHEEL_BURST_LIFT_SHARE = 0;
    //Flywheel: one continuous shot. While balls flow, push this share of what they take on average (learned drop / learned gap,
    //~2.5 V at 70 ticks/s every 0.17 s), from LEAD before the first ball is due until they stop coming (0 = off). Sim on the logs'
    //balls: it centers later balls (-15 -> -1 at 0.75) but the gaps (0.08-0.25 s) and drops (40-180) scatter them, so misses
    //don't fall (14.7% at 0, 15.0% at 0.5, 17.1% at 0.75). The SHT tries it (burst_feed)
    public static double FLYWHEEL_BURST_FEED_SHARE = 0, FLYWHEEL_BURST_FEED_LEAD = 0.02;
    public static double FLYWHEEL_PRE_KICK_VOLTS = 0, FLYWHEEL_PRE_KICK_LEAD = 0.04, FLYWHEEL_PRE_KICK_HOLD = 0.08;
    public static double FLYWHEEL_FIRST_CONTACT_SECONDS = 0.24, FLYWHEEL_BALL_INTERVAL_SECONDS = 0.34;

    public static double FLYWHEEL_VELOCITY_MARGIN_OF_ERROR = 50;
    public static double FLYWHEEL_STABILITY_MARGIN_OF_ERROR = 20;

    public static double FLYWHEEL_TICKS_PER_REV = 28;

    public static boolean SHOT_GATE_ENABLED = true;

    public static double SHOT_GATE_ENTER_BAND_BELOW = TunedConstants.SHOT_GATE_ENTER_BAND, SHOT_GATE_ENTER_BAND_ABOVE = TunedConstants.SHOT_GATE_ENTER_BAND;
    public static double SHOT_GATE_HOLD_BAND_BELOW = TunedConstants.SHOT_GATE_HOLD_BAND, SHOT_GATE_HOLD_BAND_ABOVE = TunedConstants.SHOT_GATE_HOLD_BAND;
    public static double SHOT_GATE_DWELL = TunedConstants.SHOT_GATE_DWELL;
    public static double SHOT_GATE_MIN_OPEN = 0.05;
    public static double SHOT_GATE_PULSE = 0;

}
