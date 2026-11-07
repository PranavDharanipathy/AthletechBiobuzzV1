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
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelVelocityModel;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.ShotSolver;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretRoute;
import org.firstinspires.ftc.teamcode.Utility.BetterGamepad;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;
import org.firstinspires.ftc.teamcode.Utility.Subsystem;

import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.FLYWHEEL_DISTANCES;
import static org.firstinspires.ftc.teamcode.Constants.ShooterConstants.FLYWHEEL_VELOCITIES;

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
    /// NaN = the fitted speed; a number = hold this flywheel speed (ticks/s) instead
    public double manualFlywheelVelocity = Double.NaN;

    private boolean autoAim = true;

    @Override
    public void update() {

        goal = Goal.forAlliance(alliance.getAlliance());

        if (controller1.main_buttonHasJustBeenPressed) relocalization(Garden.forAlliance(alliance.getAlliance()).relocalizationPose());

        currentRobotPose = follower.pose();
        goalAimUpdate();
        if (controller1.left_bumperHasJustBeenPressed) shooterToggle = !shooterToggle;
        if (controller2.right_bumperHasJustBeenPressed) autoAim = !autoAim;
        updateMechanisms(shooterToggle, autoAim);
    }

    public void provideComponents(Flywheel flywheel, TurretBase turret, Follower follower) {
        provideComponents(flywheel, turret, follower, null, null);
    }

    //autonomous: no gamepads, always aims
    public void updateAutonomous(boolean spinUp) {

        goal = Goal.forAlliance(alliance.getAlliance());
        currentRobotPose = follower.pose();
        updateMechanisms(spinUp, true);
    }

    public boolean isAimSolutionUsable() {
        return goalCoordinate != null && Goal.isSet(goalCoordinate) && MathUtil.isFinite(turretAimPosition) && !turretRoute.isOutside(ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) && !turretRoute.hasSwung();
    }

    private void updateMechanisms(boolean spinUp, boolean aimEnabled) {

        Velocity robotVelocity = follower.velocity();

        turretPose = getTurretPoseFromBotPose(currentRobotPose);

        goalCoordinate = goal.getCoordinate(currentRobotPose);
        boolean goalSet = Goal.isSet(goalCoordinate);

        //shoot on the move (ShotSolver): the turret point's field velocity, robot's plus its spin swinging the turret round
        double vx = robotVelocity.vx - robotVelocity.omega * offsetY(turretPose.heading());
        double vy = robotVelocity.vy + robotVelocity.omega * offsetX(turretPose.heading());

        //flywheel: the still distance this moving shot needs, for where the turret will be once the flywheel's caught up
        if (goalSet) {
            Pose lead = getTurretPoseFromBotPose(getFutureBotPose(ShooterConstants.FLYWHEEL_SOTM_LEAD_SECONDS, currentRobotPose, robotVelocity));
            shotDistance = shot.equivalentDistance(goalCoordinate.x() - lead.x(), goalCoordinate.y() - lead.y(), vx, vy);
        }
        else shotDistance = Double.NaN;
        double fitted = goalSet && ShooterConstants.FLYWHEEL_DISTANCE_FIT ? getFlywheelTargetVelocityFromInterpolation(shotDistance) : Double.NaN;
        flywheelTargetVelocity = fitted == fitted ? fitted : ShooterConstants.FLYWHEEL_DEFAULT_SHOOT_VELOCITY;
        //tuners (FlywheelInterpolationTuner) hold their own speed while the rest of the shooter aims as in a match
        if (manualFlywheelVelocity == manualFlywheelVelocity) flywheelTargetVelocity = manualFlywheelVelocity;

        //aim: a ball fired now leaves at the flywheel's actual speed, so its flight time (and whether it scores) come from that
        double gx = goalSet ? goalCoordinate.x() - turretPose.x() : 0;
        double gy = goalSet ? goalCoordinate.y() - turretPose.y() : 0;
        double actual = flywheel.getCurrentVelocity();
        boolean near = Math.abs(actual - flywheelTargetVelocity) <= ShooterConstants.SOTM_ACTUAL_SPEED_BAND;
        shotMissInches = goalSet && near ? shot.missInches(gx, gy, vx, vy, actual, ShooterConstants.FLYWHEEL_FIT_EXTRAPOLATE_INCHES) : Double.NaN;
        //spinning up (or past the fit): aim with the target speed's flight time meanwhile
        if (shotMissInches != shotMissInches) shot.equivalentDistance(gx, gy, vx, vy);
        shotTime = shot.time;
        publishMiss(shotMissInches);
        virtualGoal = goalSet ? new Pose(goalCoordinate.x() - vx * shotTime, goalCoordinate.y() - vy * shotTime) : goalCoordinate;
        distanceToGoal = goalSet ? turretPose.distance(virtualGoal) : Double.NaN;

        if (spinUp) flywheel.setVelocity(flywheelTargetVelocity, false);
        else flywheel.setVelocity(0, true);

        //turret
        if (aimEnabled && goalSet) {

            Pose aimBotPose = getFutureBotPose(ShooterConstants.TURRET_AIM_LEAD_TIME, currentRobotPose, robotVelocity);
            Pose aimTurretPose = getTurretPoseFromBotPose(aimBotPose);

            double angleToGoal = Math.toDegrees(Math.atan2(virtualGoal.y() - aimTurretPose.y(), virtualGoal.x() - aimTurretPose.x()));

            double rawtt = MathUtil.normalizeAngleDeg(angleToGoal - Math.toDegrees(aimBotPose.heading()));
            double turretNow = ShooterConstants.TURRET_AIM_DIRECTION * (turret.getPosition() - turretStartPosition) / ShooterConstants.TURRET_TICKS_PER_DEGREE;
            double aimRate = getTurretAimRate(aimBotPose, robotVelocity, virtualGoal);
            double min = ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, max = ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES;
            tt = turretRoute.update(rawtt, aimRate, turretNow, min, max, ShooterConstants.TURRET_SWING_MARGIN_DEGREES);

            turretAimPosition = ShooterConstants.TURRET_AIM_DIRECTION * tt * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;

            //no feedforward while it waits at an end
            turretAimVelocity = turretRoute.isOutside(min, max) ? 0 : ShooterConstants.TURRET_AIM_DIRECTION * TurretConstants.TURRET_RATE_FEEDFORWARD_SCALE * aimRate * ShooterConstants.TURRET_TICKS_PER_DEGREE;
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

    private static final FlywheelVelocityModel FLYWHEEL_MODEL = new FlywheelVelocityModel();
    private final ShotSolver shot = new ShotSolver(FLYWHEEL_MODEL);
    /// still distance the flywheel target is for (in), ball flight time used for the aim (s), a ball fired now lands this long
    /// (+) / short (-) along the aim (in, NaN = can't tell yet: flywheel far off its target or no goal)
    public double shotDistance, shotTime, shotMissInches = Double.NaN;

    //the blocker reads this; stamped so a stale value (no Shooter running, e.g. a tuner) never gates anything
    private static double lastMiss = Double.NaN;
    private static long lastMissNs = Long.MIN_VALUE / 2;

    private static void publishMiss(double miss) {

        lastMiss = miss;
        lastMissNs = System.nanoTime();
    }

    /// true when a ball fired now scores by ShotSolver (or the gate is off, or no Shooter has run in the last 0.2 s)
    public static boolean shotWillScore() {

        if (!ShooterConstants.SHOT_MISS_GATE || System.nanoTime() - lastMissNs > 200_000_000L) return true;
        return Math.abs(lastMiss) <= ShooterConstants.SHOT_MISS_TOLERANCE_INCHES;
    }

    /// fitted flywheel speed (ticks/s) for a distance (in), NaN if there's no fit. Loop thread only
    public static double getFlywheelTargetVelocityFromInterpolation(double distanceToGoal) {
        return FLYWHEEL_MODEL.velocity(FLYWHEEL_DISTANCES, FLYWHEEL_VELOCITIES, ShooterConstants.FLYWHEEL_FIT_DEGREE, ShooterConstants.FLYWHEEL_FIT_EXTRAPOLATE_INCHES, distanceToGoal);
    }

    public static FlywheelVelocityModel flywheelModel() {
        return FLYWHEEL_MODEL;
    }

    //turret axis from the drive center in polar: TURRET_POSITIONAL_OFFSET along the robot's heading
    private static double offsetX(double heading) {
        return ShooterConstants.TURRET_POSITIONAL_OFFSET * Math.cos(heading);
    }

    private static double offsetY(double heading) {
        return ShooterConstants.TURRET_POSITIONAL_OFFSET * Math.sin(heading);
    }

    public static Pose getTurretPoseFromBotPose(Pose botPose) {
        return new Pose(botPose.x() + offsetX(botPose.heading()), botPose.y() + offsetY(botPose.heading()), botPose.heading());
    }

    public static Pose getFutureBotPose(double t, Pose botPose, Velocity robotVelocity) {
        return new Pose(botPose.x() + robotVelocity.vx * t, botPose.y() + robotVelocity.vy * t, botPose.heading() + robotVelocity.omega * t);
    }

    /// degrees per second the turret has to turn (relative to the robot) to stay on the goal
    public static double getTurretAimRate(Pose botPose, Velocity robotVelocity, Pose goalCoordinate) {

        double ox = offsetX(botPose.heading()), oy = offsetY(botPose.heading());

        double turretVx = robotVelocity.vx - robotVelocity.omega * oy;
        double turretVy = robotVelocity.vy + robotVelocity.omega * ox;

        double dx = goalCoordinate.x() - (botPose.x() + ox);
        double dy = goalCoordinate.y() - (botPose.y() + oy);
        double distanceSquared = dx * dx + dy * dy;

        double bearingRate = distanceSquared < 1 ? 0 : (dy * turretVx - dx * turretVy) / distanceSquared;

        return Math.toDegrees(bearingRate - robotVelocity.omega);
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
