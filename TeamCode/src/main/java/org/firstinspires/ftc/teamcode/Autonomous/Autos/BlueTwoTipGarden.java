package org.firstinspires.ftc.teamcode.Autonomous.Autos;

import com.pedropathing.paths.Path;
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

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static org.firstinspires.ftc.teamcode.Constants.Autos.BlueTwoTipGardenConstants.*;

@NextAutonomous(name = "Blue 2 Tip Garden + Park", group = "A_Match")
public final class BlueTwoTipGarden extends BlazeOpMode {

    private final AthletechRobot robot;
    private final Command routine;

    public BlueTwoTipGarden(AthletechRobot robot) {

        super(robot, new BlazeHubHook(true), new BlazeHubHook(false));
        BlazeSession.beginNext(hardwareMap);
        this.robot = robot;

        Pose start = Drive.pose(START);
        Pose garden = Drive.pose(GARDEN);
        Pose sweepEnd = Drive.pose(SWEEP_END);
        Pose park = Drive.pose(PARK);
        AthletechRobot.requireCell(start, FIRST_CELL_CLOSE, "START");
        AthletechRobot.requireCell(park, !FIRST_CELL_CLOSE, "PARK");
        robot.init(hardwareMap, start);
        robot.configureAuto(AUTO_LIMIT_SECONDS, MAX_LOOP_SECONDS);
        robot.drive.configure(POSITION_ERROR_INCHES, HEADING_ERROR_DEGREES, PATH_TIMEOUT_SECONDS);
        robot.configureShots(SHOT_FEED_SECONDS, SHOT_TIMEOUT_SECONDS, SHOT_READY_DWELL_SECONDS, FLYWHEEL_ERROR_TICKS_PER_SECOND, MAX_SHOOTING_SPEED_INCHES_PER_SECOND, MAX_SHOOTING_YAW_DEGREES_PER_SECOND);

        Path toGarden = Drive.curve(start, GARDEN_CONTROLS, garden);
        Path sweep = Drive.slow(Drive.line(garden, sweepEnd), SWEEP_SPEED_INCHES_PER_SECOND);
        Path toLoadingZone = Drive.curve(sweepEnd, PARK_CONTROLS, park);

        routine = robot.autonomous(
                robot.fiArm.retract(),
                robot.shooter.rampUp(),
                robot.shoot(start),
                robot.intake.run(),
                robot.drive.follow(toGarden),
                robot.drive.follow(sweep),
                waitMs(SWEEP_DWELL_SECONDS * 1000),
                robot.drive.follow(toLoadingZone),
                robot.shoot(park),
                robot.intake.stop()
        );
    }

    @Override
    public void disabledPeriodic() {
        robot.showInit("Blue 2 Tip Garden + Park", true);
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
