package org.firstinspires.ftc.teamcode.Autonomous.Autos;

import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.AutoSubsystems.Drive;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.AthletechRobot;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static org.firstinspires.ftc.teamcode.Constants.Autos.BlueTwoTipGardenConstants.*;

@NextAutonomous(name = "Blue 2 Tip Flowers + Park", group = "A_Match")
public final class BlueTwoTipGarden extends NextOpMode {

    private final AthletechRobot robot;
    private final Command routine;

    public BlueTwoTipGarden(AthletechRobot robot) {

        super(robot);
        this.robot = robot;

        Pose start = Drive.pose(START);
        Pose allianceStage = Drive.pose(ALLIANCE_FLOWER_STAGE);
        Pose allianceTouch = Drive.pose(ALLIANCE_FLOWER_TOUCH);
        Pose sideStage = Drive.pose(SIDE_FLOWER_STAGE);
        Pose sideShot = Drive.pose(SIDE_FLOWER_SHOT);
        Pose sideTouch = Drive.pose(SIDE_FLOWER_TOUCH);
        Pose park = Drive.pose(PARK);
        AthletechRobot.requireCell(start, FIRST_CELL_CLOSE, "START");
        AthletechRobot.requireCell(sideShot, !FIRST_CELL_CLOSE, "SIDE_FLOWER_SHOT");
        robot.init(hardwareMap, start);
        robot.configureAuto(AUTO_LIMIT_SECONDS, MAX_LOOP_SECONDS);
        robot.drive.configure(POSITION_ERROR_INCHES, HEADING_ERROR_DEGREES, PATH_TIMEOUT_SECONDS);
        robot.configureShots(SHOT_FEED_SECONDS, SHOT_SETTLE_SECONDS, SHOT_TIMEOUT_SECONDS, FLYWHEEL_ERROR_TICKS_PER_SECOND);

        Path toAllianceFlower = Drive.curve(start, ALLIANCE_FLOWER_CONTROLS, allianceStage);
        Path touchAllianceFlower = Drive.slow(Drive.line(allianceStage, allianceTouch), FLOWER_SPEED_INCHES_PER_SECOND);
        Path backFromAllianceFlower = Drive.slow(Drive.line(allianceTouch, allianceStage), FLOWER_SPEED_INCHES_PER_SECOND);
        Path toSideFlower = Drive.curve(allianceStage, SIDE_FLOWER_CONTROLS, sideStage);
        Path toSideShot = Drive.slow(Drive.line(sideStage, sideShot), FLOWER_SPEED_INCHES_PER_SECOND);
        Path touchSideFlower = Drive.slow(Drive.line(sideShot, sideTouch), FLOWER_SPEED_INCHES_PER_SECOND);
        Path backFromSideFlower = Drive.slow(Drive.line(sideTouch, sideStage), FLOWER_SPEED_INCHES_PER_SECOND);
        Path toLoadingZone = Drive.curve(sideStage, PARK_CONTROLS, park);

        routine = robot.autonomous(
                robot.shooter.rampUp(),
                robot.shoot(start),
                robot.intake.run(),
                robot.drive.follow(toAllianceFlower),
                robot.fiArm.deploy(),
                waitMs(FIARM_SETTLE_SECONDS * 1000),
                robot.drive.approach(touchAllianceFlower, FLOWER_STILL_INCHES_PER_SECOND, FLOWER_STALL_SECONDS, FLOWER_TOUCH_SECONDS),
                waitMs(FLOWER_DWELL_SECONDS * 1000),
                robot.drive.follow(backFromAllianceFlower),
                robot.fiArm.retract(),
                robot.drive.follow(toSideFlower),
                robot.fiArm.deploy(),
                waitMs(FIARM_SETTLE_SECONDS * 1000),
                robot.drive.follow(toSideShot),
                robot.shoot(sideShot),
                robot.intake.run(),
                robot.drive.approach(touchSideFlower, FLOWER_STILL_INCHES_PER_SECOND, FLOWER_STALL_SECONDS, FLOWER_TOUCH_SECONDS),
                robot.shootHere(FLOWER_SHOOT_SECONDS),
                robot.drive.follow(backFromSideFlower),
                robot.fiArm.retract(),
                robot.drive.follow(toLoadingZone),
                robot.intake.stop()
        );
    }

    @Override
    public void disabledPeriodic() {
        robot.showInit("Blue 2 Tip Flowers + Park", true);
    }

    @Override
    public void start() {

        robot.handOffAs("Blue 2 Tip Flowers + Park");
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
