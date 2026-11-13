package org.firstinspires.ftc.teamcode.Autonomous.Autos;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.Drive;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.AthletechRobot;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

import static org.firstinspires.ftc.teamcode.Constants.Autos.RedPreloadParkConstants.*;

@NextAutonomous(name = "Red Preload + Park", group = "A_Match")
public final class RedPreloadPark extends NextOpMode {

    private final AthletechRobot robot;
    private final Command routine;

    public RedPreloadPark(AthletechRobot robot) {

        super(robot);
        this.robot = robot;

        Pose start = Drive.pose(START);
        Pose park = Drive.pose(PARK);
        AthletechRobot.requireCell(start, FIRST_CELL_CLOSE, "START");
        robot.init(hardwareMap, start);
        robot.configureAuto(AUTO_LIMIT_SECONDS, MAX_LOOP_SECONDS);
        robot.drive.configure(POSITION_ERROR_INCHES, HEADING_ERROR_DEGREES, PATH_TIMEOUT_SECONDS);
        robot.configureShots(SHOT_FEED_SECONDS, SHOT_SETTLE_SECONDS, SHOT_TIMEOUT_SECONDS, FLYWHEEL_ERROR_TICKS_PER_SECOND);

        routine = robot.autonomous(
                robot.shooter.rampUp(),
                robot.shoot(start),
                robot.drive.follow(Drive.curve(start, PARK_CONTROLS, park))
        );
    }

    @Override
    public void disabledPeriodic() {
        robot.showInit("Red Preload + Park", true);
    }

    @Override
    public void start() {

        robot.handOffAs("Red Preload + Park");
        robot.start(Alliance.RED);
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
