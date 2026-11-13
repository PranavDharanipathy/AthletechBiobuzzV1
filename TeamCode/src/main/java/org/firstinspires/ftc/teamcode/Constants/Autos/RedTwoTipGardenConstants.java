package org.firstinspires.ftc.teamcode.Constants.Autos;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class RedTwoTipGardenConstants {

    public static boolean FIRST_CELL_CLOSE = false;

    public static double[] START = {59.25, 9.72, 90};
    //both FLOWERS on our half. At each: stop at the stage (far enough out that the dropdown swinging down, 13.77 in ahead of the drive
    //center mid-swing, misses the FLOWER), drop it, creep in until its roller (12.19 in ahead when down) touches, take the POLLEN, back
    //out to the stage, raise it
    //Red Alliance FLOWER (red wall, face x 3.8, y 47): stage 3.8 + 13.77 + 1.9, touch 3.8 + 12.19 - 3.5
    public static double[] ALLIANCE_FLOWER_CONTROLS = {48, 24, 24, 40};
    public static double[] ALLIANCE_FLOWER_STAGE = {19.47, 47, 180};
    public static double[] ALLIANCE_FLOWER_TOUCH = {12.49, 47, 180};
    //tip 2 is shot at this FLOWER: dropdown down at the stage, in to the shot pose (closer to the FLOWER, farther from the goal) to shoot
    //what the alliance FLOWER gave, then sitting in it every ball as it comes in
    //Scoring-Side FLOWER (far wall, face y 137.7, x 46.75): stage 137.7 - 13.77 - 1.9, touch 137.7 - 12.19 + 5.5,
    //shot 137.7 - 12.19 - 1 (roller 1 in short of the face, dropdown already down)
    public static double[] SIDE_FLOWER_CONTROLS = {24, 116, 40, 108};
    public static double[] SIDE_FLOWER_STAGE = {46.75, 122.03, 90};
    public static double[] SIDE_FLOWER_SHOT = {46.75, 124.51, 90};
    public static double[] SIDE_FLOWER_TOUCH = {46.75, 131.01, 90};
    public static double[] PARK_CONTROLS = {16, 108, 32, 116};
    public static double[] PARK = {15, 108, 0};

    //slow enough that touching the FLOWER can't hurt the dropdown; the touch ends when it's stopped this long
    public static double FLOWER_SPEED_INCHES_PER_SECOND = 6, FLOWER_STILL_INCHES_PER_SECOND = 1, FLOWER_STALL_SECONDS = 0.2, FLOWER_TOUCH_SECONDS = 3;
    public static double FIARM_SETTLE_SECONDS = 0.4, FLOWER_DWELL_SECONDS = 1.2;
    //sitting in the side FLOWER, blocker open and intake on: long enough for balls that bounce off the walls to roll in and go out
    public static double FLOWER_SHOOT_SECONDS = 4;

    public static double AUTO_LIMIT_SECONDS = 29.5;
    public static double PATH_TIMEOUT_SECONDS = 8;
    public static double MAX_LOOP_SECONDS = 0.15;
    public static double POSITION_ERROR_INCHES = 1;
    public static double HEADING_ERROR_DEGREES = 3;
    public static double SHOT_TIMEOUT_SECONDS = 4;
    public static double SHOT_FEED_SECONDS = 2;
    //blocker stays open this long after the intake stops, then closes, then it drives
    public static double SHOT_SETTLE_SECONDS = 0.15;
    public static double FLYWHEEL_ERROR_TICKS_PER_SECOND = 50;

    private RedTwoTipGardenConstants() {}
}
