package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class ShooterConstants {

    /// Y-Point that differentiates the turret pointing at the goal far/close position.
    /// <p>
    /// Middle of the field: y above it (away from the audience) aims at the CLOSE point, below it at the FAR point.
    public static double FAR_ZONE_CLOSE_ZONE_BARRIER = 72;
    public static double CELL_SWITCH_BAND_INCHES = 3; //the aim's CELL only switches this far past the line

    //20 deg past straight back each way
    public static double MIN_TURRET_POSITION_IN_DEGREES = -200, MAX_TURRET_POSITION_IN_DEGREES = 200;
    public static double TURRET_SWING_MARGIN_DEGREES = 5; //how far the aim goes past an end before the turret swings a turn around

    public static double TURRET_TICKS_PER_DEGREE = 68.27;

    public static double TURRET_HOME_POSITION_INCREMENT = 200;

    public static double GOAL_NUDGE_INCHES = 1; //gamepad 2 dpad, per press

    public static double TURRET_POSITIONAL_OFFSET = -2.82252; //how forward the turret is from the center of the drivetrain (- is back, + is forward)

    //+1: encoder ticks go up as the turret turns counterclockwise (seen from above). -1 had it turning with the robot's spin, twice as fast
    public static double TURRET_AIM_DIRECTION = 1;

    //flywheel speed from the distance to the (virtual) goal: a least-squares polynomial through FLYWHEEL_DISTANCES/VELOCITIES
    //(Systems/Flywheel/FlywheelVelocityModel), not a table lookup. Degree 1 = a line: on the 2026-11 points it predicts a
    //left-out point best (16 ticks/s RMS, against 18 for 2 and 30 for 3). Off = FLYWHEEL_SHOOT_VELOCITY everywhere
    public static boolean FLYWHEEL_DISTANCE_FIT = true;
    public static int FLYWHEEL_FIT_DEGREE = 1;
    public static double FLYWHEEL_FIT_EXTRAPOLATE_INCHES = 2.5; //past the tuned distances it follows the fit this far, then holds
    //no goal (or no fit): this speed
    public static double FLYWHEEL_DEFAULT_SHOOT_VELOCITY = 1450;

    public static double TURRET_AIM_LEAD_TIME = 0;

    public static List<Double> FLYWHEEL_DISTANCES = new ArrayList<>(Arrays.asList(0.0, 10.0));
    public static List<Double> FLYWHEEL_VELOCITIES = new ArrayList<>(Arrays.asList(1450.0, 1450.0));
    //shoot on the move (Systems/Flywheel/ShotSolver). The robot's speed only adds to the ball's sideways speed; its climb, so
    //its flight time T, comes from the flywheel alone. So a moving shot gets the flywheel speed of the still distance
    //D = |goal - v T(D)| (slower driving at the goal, faster away or across), the turret aims along goal - v T, and T is
    //taken from the speed the flywheel is actually at when the ball goes
    //off: the turret always points at the hive itself. On, it aimed at goal - velocity x flight time (~0.8 s), so driving
    //swung it far off the hive and back when stopping
    public static boolean SOTM_ENABLED = false;
    //measured s from leaving the turret to the CELL at each FLYWHEEL_DISTANCES distance. All 0 = use the projectile below
    public static List<Double> SHOT_TIMES_OF_FLIGHT = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    //projectile: 72 deg fits the tuned speed table best with exit speed proportional to flywheel speed (13.6 ticks/s RMS),
    //not measured (68-76 fit almost as well: flight time +-15%). Rise: CELL opening center 1.5126 m - exit 0.292 m (profile)
    public static double SHOT_LAUNCH_ANGLE_DEGREES = 72, SHOT_RISE_INCHES = 48.05;
    //field tuning: moving shots land where the robot was heading (too little lead) = raise it, the other way = lower it
    public static double SOTM_TIME_SCALE = 1;
    //flywheel lags its target: set it for where the robot will be this soon
    public static double FLYWHEEL_SOTM_LEAD_SECONDS = 0.15;
    //the aim uses the flywheel's actual speed once it's this close to target (ticks/s), the target's speed before that
    public static double SOTM_ACTUAL_SPEED_BAND = 150;
    //the blocker only opens while a ball fired at the flywheel's actual speed lands within this of the goal along the aim (in).
    //4 in is about 29 ticks/s on the 2026-11 fit; the CELL opening is 20 x 14 in
    public static boolean SHOT_MISS_GATE = true;
    public static double SHOT_MISS_TOLERANCE_INCHES = 4;

}
