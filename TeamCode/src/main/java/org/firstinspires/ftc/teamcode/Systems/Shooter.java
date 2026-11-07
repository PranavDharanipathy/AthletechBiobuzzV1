package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.Constants.GeneralConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Field.Garden;
import org.firstinspires.ftc.teamcode.Field.Goal;
import org.firstinspires.ftc.teamcode.Systems.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.Systems.Alliance.AllianceState;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretRoute;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;
import org.firstinspires.ftc.teamcode.Utility.Subsystem;

import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.FLYWHEEL_DISTANCES;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.FLYWHEEL_VELOCITIES;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.SHOT_TIMES_OF_FLIGHT;

import java.util.List;

public class Shooter extends Subsystem {

    private BetterGamepad controller1, controller2;

    public Flywheel flywheel;

    public TurretBase turret;

    private Follower follower;

    public void provideComponents(Flywheel flywheel, TurretBase turret, Follower follower, BetterGamepad controller1, BetterGamepad controller2) {

        this.flywheel = flywheel;

        this.turret = turret;

        this.follower = follower;

        this.controller1 = controller1;
        this.controller2 = controller2;
    }

    private double turretStartPosition;

    private Goal goal;

    public Goal accessGoal() {
        return goal;
    }

    private AllianceState alliance;

    public void switchAlliance(Alliance alliance) {
        this.alliance.setAlliance(alliance);
    }

    public void start(AllianceState alliance) {

        this.alliance = alliance;

        Goal.RED.resetOffsets();
        Goal.BLUE.resetOffsets();

        goal = Goal.forAlliance(alliance.getAlliance());

        turretAimPosition = turretStartPosition = turret.startPosition;

        flywheel.reset();
    }

    private boolean shooterToggle = false;

    private double turretAimPosition;
    private final TurretRoute turretRoute = new TurretRoute();

    public double tt;

    public double turretAimVelocity;

    public Pose goalCoordinate;

    public Pose currentRobotPose;
    public Pose turretPose;

    public double distanceToGoal;
    public Pose virtualGoal;

    public double flywheelTargetVelocity;

    private boolean autoAim = true;

    @Override
    public void update() {

        goal = Goal.forAlliance(alliance.getAlliance());

        if (controller1.main_buttonHasJustBeenPressed) relocalization(Garden.forAlliance(alliance.getAlliance()).relocalizationPose());

        currentRobotPose = follower.pose();
        Velocity robotVelocity = follower.velocity();

        goalAimUpdate();

        turretPose = getTurretPoseFromBotPose(currentRobotPose);

        goalCoordinate = goal.getCoordinate(currentRobotPose);
        boolean goalSet = Goal.isSet(goalCoordinate);

        //time table SOTM: aim (and pick the speed) for where the goal is relative to the turret when the ball gets there
        virtualGoal = goalSet ? getVirtualGoalCoordinate(turretPose, robotVelocity, goalCoordinate) : goalCoordinate;
        distanceToGoal = goalSet ? turretPose.distance(virtualGoal) : Double.NaN;

        //flywheel
        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;

        flywheelTargetVelocity = goalSet ? getFlywheelTargetVelocityFromInterpolation(distanceToGoal) : ShooterConstants.DEFAULT_FLYWHEEL_VELOCITY;

        if (shooterToggle) flywheel.setVelocity(flywheelTargetVelocity, false);
        else flywheel.setVelocity(0, true);

        //turret
        if (controller2.right_bumperHasJustBeenPressed) autoAim = !autoAim;

        if (autoAim && goalSet) {

            Pose aimBotPose = getFutureBotPose(ShooterConstants.TURRET_AIM_LEAD_TIME, currentRobotPose, robotVelocity);
            Pose aimTurretPose = getTurretPoseFromBotPose(aimBotPose);

            double angleToGoal = Math.toDegrees(Math.atan2(virtualGoal.y() - aimTurretPose.y(), virtualGoal.x() - aimTurretPose.x()));

            double rawtt = MathUtil.normalizeAngleDeg(angleToGoal - Math.toDegrees(aimBotPose.heading()));
            double turretNow = ShooterConstants.TURRET_AIM_DIRECTION * (turret.getPosition() - turretStartPosition) / ShooterConstants.TURRET_TICKS_PER_DEGREE;
            double aimRate = getTurretAimRate(aimBotPose, robotVelocity, virtualGoal);
            double min = ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, max = ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES;
            tt = turretRoute.update(rawtt, aimRate, turretNow, min, max, turretLead(), turretStopDeceleration(), ShooterConstants.TURRET_SWING_MARGIN_DEGREES);

            turretAimPosition = ShooterConstants.TURRET_AIM_DIRECTION * tt * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;

            //no feedforward while it waits at an end
            turretAimVelocity = turretRoute.isOutside(min, max) ? 0 : ShooterConstants.TURRET_AIM_DIRECTION * aimRate * ShooterConstants.TURRET_TICKS_PER_DEGREE;
        }
        else {
            turretRoute.reset();
            tt = 0;
            turretAimPosition = turretStartPosition;
            turretAimVelocity = 0;
        }

        turret.setPosition(turretAimPosition);
        turret.setTargetVelocity(turretAimVelocity);

        //updating
        flywheel.update();
        turret.update();
    }

    public static double getFlywheelTargetVelocityFromInterpolation(double distanceToGoal) {
        return lookUp(FLYWHEEL_DISTANCES, FLYWHEEL_VELOCITIES, distanceToGoal);
    }

    public static double getTimeOfFlight(double distanceToGoal) {
        return lookUp(FLYWHEEL_DISTANCES, SHOT_TIMES_OF_FLIGHT, distanceToGoal);
    }

    //linear between the table's points, held at the ends
    private static double lookUp(List<Double> distances, List<Double> values, double distance) {

        int last = Math.min(distances.size(), values.size()) - 1;
        if (distance <= distances.get(0)) return values.get(0);
        if (distance >= distances.get(last)) return values.get(last);
        int i = 1;
        while (distances.get(i) < distance) i++;
        double s = (distance - distances.get(i - 1)) / (distances.get(i) - distances.get(i - 1));
        return values.get(i - 1) + s * (values.get(i) - values.get(i - 1));
    }

    public static Pose getTurretPoseFromBotPose(Pose botPose) {

        double[] offset = MathUtil.rotateCartesian(ShooterConstants.TURRET_OFFSET_X, ShooterConstants.TURRET_OFFSET_Y, botPose.heading());

        return new Pose(botPose.x() + offset[0], botPose.y() + offset[1], botPose.heading());
    }

    public static Pose getFutureBotPose(double t, Pose botPose, Velocity robotVelocity) {
        return new Pose(botPose.x() + robotVelocity.vx * t, botPose.y() + robotVelocity.vy * t, botPose.heading() + robotVelocity.omega * t);
    }

    /// the goal moved back by the turret's field velocity times the ball's time of flight, which depends on the distance
    /// to that moved goal, so it goes round SOTM_ITERATIONS times
    public static Pose getVirtualGoalCoordinate(Pose turretPose, Velocity robotVelocity, Pose goalCoordinate) {

        double[] offset = MathUtil.rotateCartesian(ShooterConstants.TURRET_OFFSET_X, ShooterConstants.TURRET_OFFSET_Y, turretPose.heading());
        double vx = robotVelocity.vx - robotVelocity.omega * offset[1];
        double vy = robotVelocity.vy + robotVelocity.omega * offset[0];
        double x = goalCoordinate.x(), y = goalCoordinate.y();
        for (int k = 0; k < ShooterConstants.SOTM_ITERATIONS; k++) {
            double t = getTimeOfFlight(Math.hypot(x - turretPose.x(), y - turretPose.y()));
            x = goalCoordinate.x() - vx * t;
            y = goalCoordinate.y() - vy * t;
        }
        return new Pose(x, y);
    }

    /// degrees per second the turret has to turn (relative to the robot) to stay on the goal
    public static double getTurretAimRate(Pose botPose, Velocity robotVelocity, Pose goalCoordinate) {

        double[] offset = MathUtil.rotateCartesian(ShooterConstants.TURRET_OFFSET_X, ShooterConstants.TURRET_OFFSET_Y, botPose.heading());

        double turretVx = robotVelocity.vx - robotVelocity.omega * offset[1];
        double turretVy = robotVelocity.vy + robotVelocity.omega * offset[0];

        double dx = goalCoordinate.x() - (botPose.x() + offset[0]);
        double dy = goalCoordinate.y() - (botPose.y() + offset[1]);
        double distanceSquared = dx * dx + dy * dy;

        double bearingRate = distanceSquared < 1 ? 0 : (dy * turretVx - dx * turretVy) / distanceSquared;

        return Math.toDegrees(bearingRate - robotVelocity.omega);
    }

    //s before the turret reacts to a new aim: dead time, plus the loop it's held for and the loop it's decided in
    private double turretLead() {

        double loop = turret.getController().getLoopTime();
        return TurretConstants.TURRET_DEAD_TIME + 2 * (Double.isNaN(loop) ? 0.03 : loop);
    }

    //deg/s^2 the controller plans its stops with (0.8 of the coasting deceleration)
    private static double turretStopDeceleration() {

        double ticks = TurretConstants.TURRET_MAX_DECELERATION > 0 ? TurretConstants.TURRET_MAX_DECELERATION : TurretConstants.TURRET_MAX_ACCELERATION > 0 ? TurretConstants.TURRET_MAX_ACCELERATION : 1000 * ShooterConstants.TURRET_TICKS_PER_DEGREE;
        return 0.8 * ticks / ShooterConstants.TURRET_TICKS_PER_DEGREE;
    }

    private void goalAimUpdate() {

        if (controller2.xHasJustBeenPressed) {
            turretStartPosition-=ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
        }
        else if (controller2.bHasJustBeenPressed) {
            turretStartPosition+=ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
        }

        //dpad as the driver sees it, Goal turns it into field x/y for the half the robot's in
        double up = (controller2.dpad_upHasJustBeenPressed ? 1 : 0) - (controller2.dpad_downHasJustBeenPressed ? 1 : 0);
        double right = (controller2.dpad_rightHasJustBeenPressed ? 1 : 0) - (controller2.dpad_leftHasJustBeenPressed ? 1 : 0);
        if (up != 0 || right != 0) goal.nudgeFromDriver(currentRobotPose, up * ShooterConstants.GOAL_NUDGE_INCHES, right * ShooterConstants.GOAL_NUDGE_INCHES);
    }

    private int relocalizations;

    private void relocalization(Pose reZeroPose) {

        follower.setPose(reZeroPose);
        relocalizations++;
        controller1.rumble(GeneralConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
    }

    /// how many times gamepad 1's main button relocalized: if it doesn't go up on a press, the press isn't reaching the robot
    public int getRelocalizations() {
        return relocalizations;
    }

    public boolean isShooterOn() {
        return shooterToggle;
    }

    public boolean isAutoAimOn() {
        return autoAim;
    }

    public String getCurrentZoneBasedOnLocation() {
        return Goal.isClose(currentRobotPose) ? "CLOSE" : "FAR";
    }

}
