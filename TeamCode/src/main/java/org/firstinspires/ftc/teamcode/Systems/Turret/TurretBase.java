package org.firstinspires.ftc.teamcode.Systems.Turret;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public TurretBase(HardwareMap hardwareMap) {
        this (hardwareMap, null);
    }

    public TurretBase(HardwareMap hardwareMap, Double currentPosition) {

        this(hardwareMap, new MonitoredCRServo(hardwareMap, DeviceConstants.leftTurretCRServoDeviceName, false), currentPosition);
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.clearBulkCache(); //MANUAL bulk cache still has the old count
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

        double now = currentPosition != null && MathUtil.isFinite(currentPosition) ? wrapIntoTravel(currentPosition) : 0;
        encoder.reset((int) Math.round(now));
        startPosition = 0;
        lastCurrentPosition = this.currentPosition = lastTargetPosition = targetPosition = Math.round(now);
        applyConstants();
    }

    public static double wrapIntoTravel(double ticks) {

        final double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE, margin = TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES;

        double degrees = ShooterConstants.TURRET_AIM_DIRECTION * ticks / tpd;
        while (degrees > ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + margin) degrees -= 360d;
        while (degrees < ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES - margin) degrees += 360d;
        
        return ShooterConstants.TURRET_AIM_DIRECTION * degrees * tpd;
    }

    public void applyConstants() {
        controller.applyConstants(ShooterConstants.TURRET_TICKS_PER_DEGREE);
    }

    //runs right after the constants every loop, so a live tuner's values stick
    public java.util.function.Consumer<TurretController> overrides;

    public TurretController getController() {
        return controller;
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

            initialError = null;
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

    public double getPosition() {
        return currentPosition;
    }

    private double error;
    private Double initialError = null;
    private double prevTime, currTime, dt;

    private final ElapsedTime timer = new ElapsedTime();

    public void update() {

        readSensors();
        applyConstants();
        if (overrides != null) overrides.accept(controller);

        double volts = controller.update(getLoopDt(), targetPosition, targetVelocity, currentPosition);
        p = controller.p / controller.gain;
        i = controller.i / controller.gain;
        d = controller.d / controller.gain;
        f = controller.f + controller.s;
        v = controller.v - (controller.p + controller.i + controller.d) / controller.gain;

        double tpd = ShooterConstants.TURRET_TICKS_PER_DEGREE;
        double aimDegrees = ShooterConstants.TURRET_AIM_DIRECTION * (currentPosition - startPosition) / tpd, outward = ShooterConstants.TURRET_AIM_DIRECTION * volts;
        double margin = TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES;
        if ((aimDegrees > ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + margin && outward > 0) || (aimDegrees < ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES - margin && outward < 0)) volts = 0;

        double nowSeconds = MathUtil.millisecondsToSeconds(currTime);
        if (Math.abs(currentPosition - firstPosition) >= 3) encoderCounted = true;
        if (Double.isNaN(firstUpdate)) firstUpdate = nowSeconds;
        boolean awake = nowSeconds - firstUpdate >= TurretConstants.TURRET_SERVO_WAKE_SECONDS;
        if (awake && !wasAwake) controller.reset();
        wasAwake = awake;
        if (!awake || currentPosition != lastCurrentPosition || Math.abs(lastVolts) < TurretConstants.TURRET_BREAKAWAY_FRICTION + TurretConstants.TURRET_FROZEN_ENCODER_OVER_BREAKAWAY_VOLTS) frozenSince = Double.NaN;
        else if (Double.isNaN(frozenSince)) frozenSince = nowSeconds;
        else if (nowSeconds - frozenSince >= TurretConstants.TURRET_FROZEN_ENCODER_SECONDS) encoderFrozen = true;
        if (awake && controller.pulsesWithoutCounts >= TurretConstants.TURRET_PULSE_DEAD_ENCODER_PUSHES) encoderFrozen = true;
        if (!encoderCounted) volts = MathUtil.clamp(volts, -(TurretConstants.TURRET_BREAKAWAY_FRICTION + TurretConstants.TURRET_UNCOUNTED_OVER_BREAKAWAY_VOLTS), (TurretConstants.TURRET_BREAKAWAY_FRICTION + TurretConstants.TURRET_UNCOUNTED_OVER_BREAKAWAY_VOLTS));
        if (encoderFrozen) volts = 0;
        lastVolts = volts;

        setVolts(volts, volts);
    }

    private double frozenSince = Double.NaN, lastVolts, firstPosition = Double.NaN, firstUpdate = Double.NaN;
    private boolean encoderFrozen, encoderCounted, wasAwake;

    public boolean hasEncoderCounted() {
        return encoderCounted;
    }

    public double getVolts() {
        return lastVolts;
    }

    public boolean isEncoderFrozen() {
        return encoderFrozen;
    }

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
        if (Double.isNaN(firstPosition)) firstPosition = currentPosition;

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

        boolean on = TurretConstants.TURRET_ENABLED;
        leftPower = on ? toPower(leftVolts) : 0;
        rightPower = on ? toPower(rightVolts) : 0;
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

    public double getAimErrorDegrees() {
        return Double.isNaN(smoothPosition) ? Double.NaN : (targetPosition - smoothPosition) / ShooterConstants.TURRET_TICKS_PER_DEGREE;
    }

    public boolean isOnTarget() {
        return aimReady;
    }

    public double getErrorMagnitude() {
        return Math.abs(error);
    }

    public double getError() {
        return error;
    }

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
