package org.firstinspires.ftc.teamcode.Autonomous.Autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.Drive;
import org.firstinspires.ftc.teamcode.Systems.AthletechRobot;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeHubHook;
import org.firstinspires.ftc.teamcode.Utility.Blaze.BlazeSession;

import dev.nextftc.robot.opmode.BlazeOpMode;
import dev.nextftc.robot.opmode.NextAutonomous;

import static org.firstinspires.ftc.teamcode.Constants.Autos.BlueParkConstants.*;

@NextAutonomous(name = "Blue Park", group = "A_Match")
public final class BluePark extends BlazeOpMode {

    private final AthletechRobot robot;
    private final Command routine;

    public BluePark(AthletechRobot robot) {

        super(robot, new BlazeHubHook(true), new BlazeHubHook(false));
        BlazeSession.beginNext(hardwareMap);
        this.robot = robot;

        Pose start = Drive.pose(START);
        Pose park = Drive.pose(PARK);
        robot.init(hardwareMap, start);
        robot.configureAuto(AUTO_LIMIT_SECONDS, MAX_LOOP_SECONDS);
        robot.drive.configure(POSITION_ERROR_INCHES, HEADING_ERROR_DEGREES, PATH_TIMEOUT_SECONDS);

        routine = robot.autonomous(
                robot.drive.follow(Drive.line(start, park))
        );
    }

    @Override
    public void disabledPeriodic() {
        robot.showInit("Blue Park", false);
    }

    @Override
    public void start() {
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
