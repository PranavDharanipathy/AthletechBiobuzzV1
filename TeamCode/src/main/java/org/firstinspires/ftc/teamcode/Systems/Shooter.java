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
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;
import org.firstinspires.ftc.teamcode.Utility.Recording.Recorder;
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


    private double turretAimPosition;
    private final TurretRoute turretRoute = new TurretRoute();

    public double tt;
    public double rawAimDegrees = Double.NaN, aimRateDegrees;

    public double turretAimVelocity;

    public Pose goalCoordinate;

    public Pose currentRobotPose;
    public Pose turretPose;

    public double distanceToGoal;
    public Pose virtualGoal;

    public double flywheelTargetVelocity;
    public double manualFlywheelVelocity = Double.NaN;
    //a live tuner's TURRET_RATE_FEEDFORWARD_SCALE, NaN = the constant
    public double turretRateScale = Double.NaN;

    private boolean autoAim = true;
    private boolean turretLocked;
    private boolean closeCell, cellChosen;
    private int swings;

    @Override
    public void update() {

        goal = Goal.forAlliance(alliance.getAlliance());

        if (controller1.main_buttonHasJustBeenPressed) relocalization(Garden.forAlliance(alliance.getAlliance()).relocalizationPose());

        currentRobotPose = follower.pose();
        goalAimUpdate();
        if (controller2.right_bumperHasJustBeenPressed) {
            autoAim = !autoAim;
            Recorder.note("driver", autoAim ? "auto aim on" : "auto aim off");
        }
        if (controller1.yHasJustBeenPressed) {
            turretLocked = !turretLocked;
            Recorder.note("driver", turretLocked ? "turret locked at 0" : "turret aiming");
        }
        //flywheel always on from START
        updateMechanisms(true, autoAim && !turretLocked);
    }

    public void provideComponents(Flywheel flywheel, TurretBase turret, Follower follower) {
        provideComponents(flywheel, turret, follower, null, null);
    }

    public void updateAutonomous(boolean rampUp) {
        updateAutonomous(rampUp, true);
    }

    public void updateAutonomous(boolean rampUp, boolean aim) {

        goal = Goal.forAlliance(alliance.getAlliance());
        currentRobotPose = follower.pose();
        updateMechanisms(rampUp, aim);
    }

    public boolean isAimSolutionUsable() {
        return goalCoordinate != null && Goal.isSet(goalCoordinate) && MathUtil.isFinite(turretAimPosition) && !turretRoute.isOutside(ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) && !turretRoute.hasSwung();
    }

    private void updateMechanisms(boolean rampUp, boolean aimEnabled) {

        Velocity robotVelocity = follower.velocity();

        turretPose = getTurretPoseFromBotPose(currentRobotPose);

        double y = currentRobotPose.y(), line = ShooterConstants.FAR_ZONE_CLOSE_ZONE_BARRIER, band = ShooterConstants.CELL_SWITCH_BAND_INCHES;
        boolean wasClose = closeCell;
        if (y > line + band) closeCell = true;
        else if (y < line - band) closeCell = false;
        else if (!cellChosen) closeCell = y > line;
        if (cellChosen && wasClose != closeCell) Recorder.note("shooter", closeCell ? "cell CLOSE" : "cell FAR", y);
        cellChosen = true;
        goalCoordinate = closeCell ? goal.getCloseCoordinate() : goal.getFarCoordinate();
        boolean goalSet = Goal.isSet(goalCoordinate);

        double vx = robotVelocity.vx - robotVelocity.omega * offsetY(turretPose.heading());
        double vy = robotVelocity.vy + robotVelocity.omega * offsetX(turretPose.heading());

        if (goalSet) {
            Pose lead = getTurretPoseFromBotPose(getFutureBotPose(ShooterConstants.FLYWHEEL_SOTM_LEAD_SECONDS, currentRobotPose, robotVelocity));
            shotDistance = shot.equivalentDistance(goalCoordinate.x() - lead.x(), goalCoordinate.y() - lead.y(), vx, vy);
        }
        else shotDistance = Double.NaN;
        double fitted = goalSet && ShooterConstants.FLYWHEEL_DISTANCE_FIT ? getFlywheelTargetVelocityFromInterpolation(shotDistance) : Double.NaN;
        flywheelTargetVelocity = fitted == fitted ? fitted : ShooterConstants.FLYWHEEL_DEFAULT_SHOOT_VELOCITY;
        if (manualFlywheelVelocity == manualFlywheelVelocity) flywheelTargetVelocity = manualFlywheelVelocity;

        double gx = goalSet ? goalCoordinate.x() - turretPose.x() : 0;
        double gy = goalSet ? goalCoordinate.y() - turretPose.y() : 0;
        double actual = flywheel.getCurrentVelocity();
        boolean near = Math.abs(actual - flywheelTargetVelocity) <= ShooterConstants.SOTM_ACTUAL_SPEED_BAND;
        shotMissInches = goalSet && near ? shot.missInches(gx, gy, vx, vy, actual, ShooterConstants.FLYWHEEL_FIT_EXTRAPOLATE_INCHES) : Double.NaN;
        if (shotMissInches != shotMissInches) shot.equivalentDistance(gx, gy, vx, vy);
        shotTime = shot.time;
        double extrapolate = ShooterConstants.FLYWHEEL_FIT_EXTRAPOLATE_INCHES;
        boolean covered = FLYWHEEL_MODEL.isValid() && Math.abs(FLYWHEEL_MODEL.evaluate(FLYWHEEL_MODEL.maxDistance()) - FLYWHEEL_MODEL.evaluate(FLYWHEEL_MODEL.minDistance())) > 1 && shotDistance >= FLYWHEEL_MODEL.minDistance() - extrapolate && shotDistance <= FLYWHEEL_MODEL.maxDistance() + extrapolate;
        publishMiss(shotMissInches, covered);
        virtualGoal = goalSet ? new Pose(goalCoordinate.x() - vx * shotTime, goalCoordinate.y() - vy * shotTime) : goalCoordinate;
        distanceToGoal = goalSet ? turretPose.distance(virtualGoal) : Double.NaN;

        if (rampUp) flywheel.setVelocity(flywheelTargetVelocity, false);
        else flywheel.setVelocity(0, true);

        if (aimEnabled && goalSet) {

            Pose aimBotPose = getFutureBotPose(ShooterConstants.TURRET_AIM_LEAD_TIME, currentRobotPose, robotVelocity);
            Pose aimTurretPose = getTurretPoseFromBotPose(aimBotPose);

            double angleToGoal = Math.toDegrees(Math.atan2(virtualGoal.y() - aimTurretPose.y(), virtualGoal.x() - aimTurretPose.x()));

            double rawtt = MathUtil.normalizeAngleDeg(angleToGoal - Math.toDegrees(aimBotPose.heading()));
            double turretNow = ShooterConstants.TURRET_AIM_DIRECTION * (turret.getPosition() - turretStartPosition) / ShooterConstants.TURRET_TICKS_PER_DEGREE;
            double aimRate = getTurretAimRate(aimBotPose, robotVelocity, virtualGoal);
            double min = ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, max = ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES;
            tt = turretRoute.update(rawtt, aimRate, turretNow, min, max, ShooterConstants.TURRET_SWING_MARGIN_DEGREES, Clock.seconds());
            rawAimDegrees = rawtt;
            aimRateDegrees = aimRate;
            if (turretRoute.hasSwung()) {
                swings++;
                Recorder.note("turret", "swing a full turn to", tt);//keep cuz wrap around can overshoot and take a long time to settle so use its log to see how we can improve tuning
            }

            turretAimPosition = ShooterConstants.TURRET_AIM_DIRECTION * tt * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretStartPosition;

            turretAimVelocity = turretRoute.isOutside(min, max) ? 0 : ShooterConstants.TURRET_AIM_DIRECTION * (turretRateScale == turretRateScale ? turretRateScale : TurretConstants.TURRET_RATE_FEEDFORWARD_SCALE) * aimRate * ShooterConstants.TURRET_TICKS_PER_DEGREE;
        }
        else {
            turretRoute.reset();
            tt = 0;
            rawAimDegrees = Double.NaN;
            aimRateDegrees = 0;
            turretAimPosition = turretStartPosition;
            turretAimVelocity = 0;
        }

        turret.setPosition(turretAimPosition);
        turret.setTargetVelocity(turretAimVelocity);

        flywheel.update();
        turret.update();
    }

    private static final FlywheelVelocityModel FLYWHEEL_MODEL = new FlywheelVelocityModel();
    private final ShotSolver shot = new ShotSolver(FLYWHEEL_MODEL);
    public double shotDistance, shotTime, shotMissInches = Double.NaN;

    private static double lastMiss = Double.NaN;
    private static long lastMissNs = Long.MIN_VALUE / 2;

    private static boolean lastMissCovered;

    private static void publishMiss(double miss, boolean covered) {

        lastMiss = miss;
        lastMissCovered = covered;
        lastMissNs = System.nanoTime();
    }

    public static boolean shotWillScore() {

        if (!ShooterConstants.SHOT_MISS_GATE || !lastMissCovered || System.nanoTime() - lastMissNs > 200_000_000L) return true;
        return Math.abs(lastMiss) <= ShooterConstants.SHOT_MISS_TOLERANCE_INCHES;
    }

    public static double getFlywheelTargetVelocityFromInterpolation(double distanceToGoal) {
        return FLYWHEEL_MODEL.velocity(FLYWHEEL_DISTANCES, FLYWHEEL_VELOCITIES, ShooterConstants.FLYWHEEL_FIT_DEGREE, ShooterConstants.FLYWHEEL_FIT_EXTRAPOLATE_INCHES, distanceToGoal);
    }

    public static FlywheelVelocityModel flywheelModel() {
        return FLYWHEEL_MODEL;
    }

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

        if (controller2.bHasJustBeenPressed) {
            turretStartPosition-=ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
        }
        else if (controller2.xHasJustBeenPressed) {
            turretStartPosition+=ShooterConstants.TURRET_HOME_POSITION_INCREMENT;
        }

        double up = (controller2.dpad_upHasJustBeenPressed ? 1 : 0) - (controller2.dpad_downHasJustBeenPressed ? 1 : 0);
        double right = (controller2.dpad_rightHasJustBeenPressed ? 1 : 0) - (controller2.dpad_leftHasJustBeenPressed ? 1 : 0);
        if (up != 0 || right != 0) goal.nudgeFromDriver(currentRobotPose, up * ShooterConstants.GOAL_NUDGE_INCHES, right * ShooterConstants.GOAL_NUDGE_INCHES);
    }

    private int relocalizations;

    private void relocalization(Pose reZeroPose) {

        follower.setPose(reZeroPose);
        relocalizations++;
        Recorder.note("driver", "relocalized", relocalizations);
        controller1.rumble(GeneralConstants.NORMAL_CONTROLLER_RUMBLE_TIME);
    }

    public boolean isCloseCell() {
        return closeCell;
    }

    public int getSwings() {
        return swings;
    }

    public boolean isTurretLocked() {
        return turretLocked;
    }

    public int getRelocalizations() {
        return relocalizations;
    }

    public boolean isShooterOn() {
        return true;
    }

    public boolean isAutoAimOn() {
        return autoAim;
    }

    public String getCurrentZoneBasedOnLocation() {
        return Goal.isClose(currentRobotPose) ? "CLOSE" : "FAR";
    }

}
