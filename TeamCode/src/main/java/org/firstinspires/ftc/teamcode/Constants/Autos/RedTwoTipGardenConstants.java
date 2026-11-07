package org.firstinspires.ftc.teamcode.Constants.Autos;

import com.acmerobotics.dashboard.config.Config;

/// Red 2 Tip Garden + Park only: nothing here is shared with another auto. Read at INIT.
/// Poses are {x in, y in, heading deg} in the turret's frame (FieldConstants), the same as the Pedro visualizer's BIOBUZZ
/// field; controls are {x1, y1, x2, y2}. Project: docs/autos/biobuzz_red_2tip_garden.pp
@Config
public final class RedTwoTipGardenConstants {

    //red's upward CELL at setup is the FAR one (manual 10.3.1, 3 NECTAR in it); shots from y < 72 aim FAR
    public static boolean FIRST_CELL_CLOSE = false;

    //back on the audience wall straight under red's FAR CELL (59.25, 56.23): a straight shot up into it
    public static double[] START = {59.25, 9.72, 90};
    //red GARDEN: the audience wall from the audience-left corner, x 0-23. Side 1 in off the red wall
    public static double[] GARDEN = {8.64, 22, -90};
    public static double[] GARDEN_CONTROLS = {34, 34, 34, 39};
    //front 1 in off the wall: the intake's own reach, the FIArm stays up
    public static double[] SWEEP_END = {8.64, 10.72, -90};
    public static double[] PARK_CONTROLS = {26, 30, 26, 90};
    //LOADING ZONE, on the other half: tip 2 goes in the CELL tip 1 brought up
    public static double[] PARK = {15, 108, 0};

    public static double SWEEP_SPEED_INCHES_PER_SECOND = 15;
    public static double SWEEP_DWELL_SECONDS = 0.6;

    public static double AUTO_LIMIT_SECONDS = 29.5;
    public static double PATH_TIMEOUT_SECONDS = 8;
    public static double MAX_LOOP_SECONDS = 0.15;
    public static double POSITION_ERROR_INCHES = 1;
    public static double HEADING_ERROR_DEGREES = 3;
    public static double SHOT_TIMEOUT_SECONDS = 4;
    public static double SHOT_FEED_SECONDS = 1.5;
    public static double SHOT_READY_DWELL_SECONDS = 0.1;
    public static double FLYWHEEL_ERROR_TICKS_PER_SECOND = 50;
    public static double MAX_SHOOTING_SPEED_INCHES_PER_SECOND = 3;
    public static double MAX_SHOOTING_YAW_DEGREES_PER_SECOND = 10;

    private RedTwoTipGardenConstants() {}
}
