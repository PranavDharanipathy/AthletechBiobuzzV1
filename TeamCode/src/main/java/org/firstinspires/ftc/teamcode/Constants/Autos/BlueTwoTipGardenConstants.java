package org.firstinspires.ftc.teamcode.Constants.Autos;

import com.acmerobotics.dashboard.config.Config;

/// Blue 2 Tip Garden + Park only: nothing here is shared with another auto. Read at INIT.
/// Poses are {x in, y in, heading deg} in the turret's frame (FieldConstants), the same as the Pedro visualizer's BIOBUZZ
/// field; controls are {x1, y1, x2, y2}. Project: docs/autos/biobuzz_blue_2tip_garden.pp
@Config
public final class BlueTwoTipGardenConstants {

    //blue's upward CELL at setup is the CLOSE one (manual 10.3.1, 3 NECTAR in it); shots from y > 72 aim CLOSE
    public static boolean FIRST_CELL_CLOSE = true;

    //back on the far wall straight over blue's CLOSE CELL (84.75, 87.77): a straight shot down into it
    public static double[] START = {84.75, 134.28, -90};
    //blue GARDEN: the far wall from the far-right corner, x 121-144. Side 1 in off the blue wall
    public static double[] GARDEN = {135.36, 122, 90};
    public static double[] GARDEN_CONTROLS = {110, 110, 110, 105};
    //front 1 in off the wall: the intake's own reach, the FIArm stays up
    public static double[] SWEEP_END = {135.36, 133.28, 90};
    public static double[] PARK_CONTROLS = {115, 114, 115, 54};
    //LOADING ZONE, on the other half: tip 2 goes in the CELL tip 1 brought up
    public static double[] PARK = {129, 36, 180};

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

    private BlueTwoTipGardenConstants() {}
}
