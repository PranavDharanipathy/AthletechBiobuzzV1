package org.firstinspires.ftc.teamcode.Autonomous.Autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.Drive;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.AthletechRobot;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeHubHook;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;

import dev.nextftc.robot.opmode.BlazeOpMode;
import dev.nextftc.robot.opmode.NextAutonomous;

import static org.firstinspires.ftc.teamcode.Constants.Autos.BluePreloadParkConstants.*;

@NextAutonomous(name = "Blue Preload + Park", group = "A_Match")
public final class BluePreloadPark extends BlazeOpMode {

    private final AthletechRobot robot;
    private final Command routine;

    public BluePreloadPark(AthletechRobot robot) {

        super(robot, new BlazeHubHook(true), new BlazeHubHook(false));
        BlazeSession.beginNext(hardwareMap);
        this.robot = robot;

        Pose start = Drive.pose(START);
        Pose park = Drive.pose(PARK);
        AthletechRobot.requireCell(start, FIRST_CELL_CLOSE, "START");
        robot.init(hardwareMap, start);
        robot.configureAuto(AUTO_LIMIT_SECONDS, MAX_LOOP_SECONDS);
        robot.drive.configure(POSITION_ERROR_INCHES, HEADING_ERROR_DEGREES, PATH_TIMEOUT_SECONDS);
        robot.configureShots(SHOT_FEED_SECONDS, SHOT_TIMEOUT_SECONDS, SHOT_READY_DWELL_SECONDS, FLYWHEEL_ERROR_TICKS_PER_SECOND, MAX_SHOOTING_SPEED_INCHES_PER_SECOND, MAX_SHOOTING_YAW_DEGREES_PER_SECOND);

        routine = robot.autonomous(
                robot.shooter.spinUp(),
                robot.shoot(start),
                robot.drive.follow(Drive.line(start, park))
        );
    }

    @Override
    public void disabledPeriodic() {
        robot.showInit("Blue Preload + Park", true);
    }

    @Override
    public void start() {

        robot.start(Alliance.BLUE);
        Scheduler.schedule(routine);
    }

    @Override
    public void periodic() {
        robot.show();
    }

    @Override
    public void end() {
        robot.stopAll();
    }
}
