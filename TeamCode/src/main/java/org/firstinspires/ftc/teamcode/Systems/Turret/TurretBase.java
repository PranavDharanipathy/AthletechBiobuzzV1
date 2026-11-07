package org.firstinspires.ftc.teamcode.Systems.Turret;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TunedConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Utility.Encoder;
import org.firstinspires.ftc.teamcode.Utility.HubVoltage;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredCRServo;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;
import org.firstinspires.ftc.teamcode.Utility.VoltageCompensator;

public class TurretBase {

    private final CRServo leftTurretBase;
    private final CRServo rightTurretBase;

    private final Encoder encoder;

    private final TurretController controller = new TurretController();

    public double p, i, d, f, v;

    private double targetVelocity = 0;

    /// ticks per second
    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = MathUtil.isFinite(targetVelocity) ? targetVelocity : 0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    private double power = 0, leftPower = 0, rightPower = 0;

    private final VoltageSensor batteryVoltageSensor;

    private final VoltageCompensator voltageCompensator = new VoltageCompensator(TurretConstants.TURRET_VOLTAGE_SAMPLE_PERIOD, TurretConstants.TURRET_VOLTAGE_FILTER_TAU, TurretConstants.TURRET_NOMINAL_VOLTS);

    private boolean voltageCompensationEnabled = TurretConstants.TURRET_VOLTAGE_COMPENSATION;

    public void setVoltageCompensation(boolean enabled) {
        voltageCompensationEnabled = enabled;
    }

    public boolean isVoltageCompensationEnabled() {
        return voltageCompensationEnabled;
    }

    public VoltageSensor getVoltageSensor() {
        return batteryVoltageSensor;
    }

    public double getCompensationVolts() {
        return voltageCompensationEnabled ? voltageCompensator.volts() : TurretConstants.TURRET_NOMINAL_VOLTS;
    }

    /// resets the encoder, so the turret must point straight (home) at INIT
    public TurretBase(HardwareMap hardwareMap) {
        this (hardwareMap, null);
    }

    /// resets the encoder and says the turret is at currentPosition now
    /// @param currentPosition ticks from home where the turret is right now (like the end of auto), null = at home
    public TurretBase(HardwareMap hardwareMap, Double currentPosition) {

        this(hardwareMap, new MonitoredCRServo(hardwareMap, DeviceConstants.leftTurretCRServoDeviceName, false), currentPosition);
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.clearBulkCache(); //MANUAL bulk cache still has the count from before the reset
    }

    private TurretBase(HardwareMap hardwareMap, CRServo left, Double currentPosition) {
        this(left, new MonitoredCRServo(hardwareMap, DeviceConstants.rightTurretCRServoDeviceName, false), new Encoder(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.turretEncoderDeviceName, false)), HubVoltage.forDevice(hardwareMap, left), currentPosition);
    }

    public TurretBase(CRServo left, CRServo right, Encoder angleEncoder, VoltageSensor voltageSensor, Double currentPosition) {

        leftTurretBase = java.util.Objects.requireNonNull(left, "left servo");
        rightTurretBase = java.util.Objects.requireNonNull(right, "right servo");

        leftTurretBase.setDirection(TurretConstants.TURRET_BASE_DIRECTIONS[0]);
        rightTurretBase.setDirection(TurretConstants.TURRET_BASE_DIRECTIONS[1]);

        encoder = java.util.Objects.requireNonNull(angleEncoder, "encoder");
        encoder.setDirection(TurretConstants.TURRET_ENCODER_DIRECTION);
        encoder.setVelocityFilterParameters(
                TunedConstants.TURRET_FILTER_MIN_BANDWIDTH, TunedConstants.TURRET_FILTER_MAX_BANDWIDTH,
                TunedConstants.TURRET_FILTER_SCORE_MEMORY, TunedConstants.TURRET_FILTER_STALE_TIME
        );

        batteryVoltageSensor = java.util.Objects.requireNonNull(voltageSensor, "voltage sensor");

        //home is always 0 now, the turret holds where it is until told otherwise
        double now = currentPosition != null && MathUtil.isFinite(currentPosition) ? wrapIntoTravel(currentPosition) : 0;
        encoder.reset((int) Math.round(now));
        startPosition = 0;
        lastCurrentPosition = this.currentPosition = lastTargetPosition = targetPosition = Math.round(now);
        applyConstants();
    }

    /// a turn either way is the same spot, so a position past the travel gets a turn taken off (350 deg is -10)
    public static double wrapIntoTravel(double ticks) {

        double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE, margin = TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES;
        double degrees = ShooterConstants.TURRET_AIM_DIRECTION * ticks / tpd;
        while (degrees > ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + margin) degrees -= 360;
        while (degrees < ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES - margin) degrees += 360;
        return ShooterConstants.TURRET_AIM_DIRECTION * degrees * tpd;
    }

    /// dashboard edits to TurretConstants land on the next update
    public void applyConstants() {

        double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE;
        controller.configure(TurretConstants.TURRET_GAIN, TurretConstants.TURRET_TIME_CONSTANT, TurretConstants.TURRET_KINETIC_FRICTION, TurretConstants.TURRET_BREAKAWAY_FRICTION, TurretConstants.TURRET_DEAD_TIME, TurretConstants.TURRET_BANDWIDTH, TurretConstants.TURRET_DAMPING, TurretConstants.TURRET_INTEGRAL_RATIO, TurretConstants.TURRET_FRICTION_SCALE);
        controller.setCapability(TurretConstants.TURRET_MAX_SPEED, TurretConstants.TURRET_MAX_ACCELERATION, TurretConstants.TURRET_MAX_DECELERATION);
        controller.startDelay = TurretConstants.TURRET_START_DELAY > 0 ? TurretConstants.TURRET_START_DELAY : Double.NaN;
        controller.vibrationRejection = TurretConstants.TURRET_VIBRATION_REJECTION;
        controller.vibrationReference = TurretConstants.TURRET_VIBRATION_REFERENCE_DEGREES * tpd;
        controller.vibrationDeadband = TurretConstants.TURRET_VIBRATION_DEADBAND_DEGREES * tpd;
        controller.minimumBandwidthFraction = TurretConstants.TURRET_MIN_BANDWIDTH_FRACTION;
        controller.minimumBandwidth = TurretConstants.TURRET_MIN_BANDWIDTH;
        controller.lanyards = TurretConstants.TURRET_LANYARDS_ENABLED ? getLanyardMap() : null;
        controller.lanyardLearning = TurretConstants.TURRET_LANYARD_LEARNING || forceLanyardLearning;
        controller.lanyardHome = startPosition;
        controller.lanyardTicksPerDegree = tpd;
        controller.lanyardLearnRate = TurretConstants.TURRET_LANYARD_LEARN_RATE;
        controller.lanyardLearnBand = TurretConstants.TURRET_LANYARD_LEARN_BAND_DEGREES * tpd;
        controller.lanyardMaxVolts = TurretConstants.TURRET_LANYARD_MAX_VOLTS;
        controller.lanyardStuckBand = TurretConstants.TURRET_LANYARD_STUCK_BAND_DEGREES * tpd;
        controller.setLimits(TurretConstants.TURRET_MAX_SPEED_FRACTION, TurretConstants.TURRET_INTEGRAL_ZONE_DEGREES * tpd, TurretConstants.TURRET_MAX_INTEGRAL_VOLTS, TurretConstants.TURRET_FRICTION_RAMP_DEGREES * tpd, TurretConstants.TURRET_MOVING_DEGREES_PER_SECOND * tpd, TurretConstants.TURRET_HOLD_TOLERANCE_DEGREES * tpd);
        controller.setTravel(startPosition + ShooterConstants.TURRET_AIM_DIRECTION * ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES * tpd, startPosition + ShooterConstants.TURRET_AIM_DIRECTION * ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES * tpd);
    }

    public TurretController getController() {
        return controller;
    }

    //one map for every OpMode, so what TeleOp learned is there next match too (and on file for after a restart)
    private static LanyardMap lanyardMap;
    private static boolean lanyardLearned = false;

    /// the lanyard learning sweep turns learning on even when TURRET_LANYARD_LEARNING is off
    public boolean forceLanyardLearning = false;

    public static File lanyardFile() {
        return new File(new File(AppUtil.ROBOT_DATA_DIR, "TurretLanyards"), "map.csv");
    }

    /// loads the saved map the first time, and redoes the bins if the range or bin size got changed on the dashboard
    public static LanyardMap getLanyardMap() {

        double margin = TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES;
        double low = Math.min(ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, -ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES) - margin;
        double high = Math.max(ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES, -ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES) + margin;
        LanyardMap m = lanyardMap;
        if (m != null && m.minDegrees == low && m.maxDegrees == high && m.binDegrees == Math.max(TurretConstants.TURRET_LANYARD_BIN_DEGREES, 0.5)) return m;
        LanyardMap fresh = new LanyardMap(low, high, TurretConstants.TURRET_LANYARD_BIN_DEGREES);
        if (m != null) fresh.fromCsv(m.toCsv());
        else fresh.load(lanyardFile());
        lanyardMap = fresh;
        return fresh;
    }

    /// call when the OpMode stops; only writes when lanyards are on and something could have been learned
    public void saveLanyards() {

        if (lanyardMap == null || !TurretConstants.TURRET_LANYARDS_ENABLED || !(controller.lanyardLearning || lanyardLearned)) return;
        lanyardMap.save(lanyardFile());
    }

    /// forgets everything learned, in memory and on file
    public static void clearLanyards() {

        getLanyardMap().clear();
        lanyardLearned = true;
        lanyardMap.save(lanyardFile());
    }

    /// volts the map added this loop
    public double getLanyardVolts() {
        return controller.lanyard;
    }

    public double startPosition;
    private double lastTargetPosition;
    private double targetPosition;

    private double currentPosition;
    private double lastCurrentPosition;

    public void setPosition(double position) {

        if (targetPosition != position) {

            lastTargetPosition = targetPosition;
            targetPosition = position;

            initialError = null; //null means that it's to be determined
        }
    }

    public double getLastTargetPosition() {
        return lastTargetPosition;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getLastCurrentPosition() {
        return lastCurrentPosition;
    }

    public double getCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    /// what readSensors last saw
    public double getPosition() {
        return currentPosition;
    }

    public double getReference() {
        return controller.getReference();
    }

    private double error;
    private Double initialError = null;
    private double prevTime, currTime, dt;

    private final ElapsedTime timer = new ElapsedTime();

    public void update() {

        readSensors();
        applyConstants();

        double volts = controller.update(getLoopDt(), targetPosition, targetVelocity, currentPosition, velocity, getCompensationVolts());
        p = controller.p;
        i = controller.i;
        d = controller.d;
        f = controller.f;
        v = controller.v;

        //wiring: past the travel limit it may only come back
        double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE;
        double aimDegrees = ShooterConstants.TURRET_AIM_DIRECTION * (currentPosition - startPosition) / tpd, outward = ShooterConstants.TURRET_AIM_DIRECTION * volts;
        double margin = TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES;
        if ((aimDegrees > ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + margin && outward > 0) || (aimDegrees < ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES - margin && outward < 0)) volts = 0;

        setVolts(volts, volts);
    }

    /// update() calls this, only call it yourself when driving the servos with setVolts
    public void readSensors() {

        prevTime = currTime;
        currTime = timer.milliseconds();
        dt = currTime - prevTime;

        double now = MathUtil.millisecondsToSeconds(currTime);
        if (voltageCompensationEnabled && voltageCompensator.shouldSample(now)) {
            voltageCompensator.addSample(batteryVoltageSensor.getVoltage(), now);
        }

        lastCurrentPosition = currentPosition;
        currentPosition = getCurrentPosition();

        rawVelocity = encoder.getInternalVelocity();
        velocity = encoder.updateFilteredVelocity(now, rawVelocity);

        error = targetPosition - currentPosition;

        double aimDt = MathUtil.millisecondsToSeconds(dt);
        smoothPosition = Double.isNaN(smoothPosition) || !(aimDt > 0) || aimDt > 0.25 ? currentPosition : smoothPosition + (currentPosition - smoothPosition) * Math.min(1, aimDt / TurretConstants.TURRET_AIM_GATE_SMOOTHING);
        double aimError = Math.abs(getAimErrorDegrees());
        if (aimReady) {
            if (aimError > TurretConstants.TURRET_AIM_GATE_HOLD_DEGREES) aimReady = false;
        }
        else if (aimError <= TurretConstants.TURRET_AIM_GATE_ENTER_DEGREES) {
            if (Double.isNaN(aimInsideSince)) aimInsideSince = now;
            if (now - aimInsideSince >= TurretConstants.TURRET_AIM_GATE_DWELL) aimReady = true;
        }
        else aimInsideSince = Double.NaN;
        if (initialError == null) initialError = error;
    }

    public void setVolts(double leftVolts, double rightVolts) {

        leftPower = toPower(leftVolts);
        rightPower = toPower(rightVolts);
        power = 0.5 * (leftPower + rightPower);
        leftTurretBase.setPower(leftPower);
        rightTurretBase.setPower(rightPower);
    }

    private double toPower(double volts) {

        if (voltageCompensationEnabled) return voltageCompensator.toPower(volts);
        return MathUtil.isFinite(volts) ? MathUtil.clamp(volts / TurretConstants.TURRET_NOMINAL_VOLTS, -1, 1) : 0;
    }

    private double smoothPosition = Double.NaN, aimInsideSince = Double.NaN;
    private boolean aimReady = false;

    /// target minus where it points, shake smoothed out
    public double getAimErrorDegrees() {
        return Double.isNaN(smoothPosition) ? Double.NaN : (targetPosition - smoothPosition) / ShooterConstants.TURRET_TICKS_PER_DEGREE;
    }

    /// stayed within TURRET_AIM_GATE_ENTER_DEGREES for the dwell, until it leaves TURRET_AIM_GATE_HOLD_DEGREES
    public boolean isOnTarget() {
        return aimReady;
    }

    public double getVibrationDegrees() {
        return controller.vibration / ShooterConstants.TURRET_TICKS_PER_DEGREE;
    }

    public double getEffectiveBandwidth() {
        return controller.effectiveBandwidth;
    }

    public double getErrorMagnitude() {
        return Math.abs(error);
    }

    public double getError() {
        return error;
    }

    /// @return What the error was initially when the PID started working towards the new target position
    public double getInitialError() {
        return initialError;
    }

    private double rawVelocity = 0, velocity = 0;

    public double getVelocity() {
        return velocity;
    }

    public double getRawVelocity() {
        return rawVelocity;
    }

    public double getFilterBandwidth() {
        return encoder.getFilterBandwidth();
    }

    public double getPower() {
        return power;
    }

    public double getLoopDt() {
        return MathUtil.millisecondsToSeconds(dt);
    }

    public double[] getServoPowers() {
        return new double[] {leftPower, rightPower};
    }

}
