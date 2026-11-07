package org.firstinspires.ftc.teamcode.Systems.Flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Constants.TunedConstants;
import org.firstinspires.ftc.teamcode.Utility.Clock;
import org.firstinspires.ftc.teamcode.Utility.DynamicTrapezoidalSum;
import org.firstinspires.ftc.teamcode.Utility.Encoder;
import org.firstinspires.ftc.teamcode.Utility.HubVoltage;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;
import org.firstinspires.ftc.teamcode.Utility.VoltageCompensator;

public final class Flywheel {

    private final Encoder encoder;

    public Encoder getEncoder() {
        return encoder;
    }

    private final DcMotorEx flywheelMotor;

    private VoltageSensor batteryVoltageSensor;

    private final VoltageCompensator voltageCompensator = new VoltageCompensator(FlywheelConstants.FLYWHEEL_VOLTAGE_SAMPLE_PERIOD, FlywheelConstants.FLYWHEEL_VOLTAGE_FILTER_TAU, FlywheelConstants.FLYWHEEL_NOMINAL_VOLTS);

    public Flywheel(DcMotorEx flywheelMotor) {

        this.flywheelMotor = flywheelMotor;

        this.flywheelMotor.setDirection(FlywheelConstants.FLYWHEEL_MOTOR_DIRECTION);

        encoder = new Encoder(this.flywheelMotor);
        encoder.setDirection(Encoder.Direction.FORWARD);
        encoder.setVelocityFilterParameters(
                TunedConstants.FLYWHEEL_FILTER_MIN_BANDWIDTH, TunedConstants.FLYWHEEL_FILTER_MAX_BANDWIDTH,
                TunedConstants.FLYWHEEL_FILTER_SCORE_MEMORY, TunedConstants.FLYWHEEL_FILTER_STALE_TIME
        );

        this.flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void initVoltageSensor(HardwareMap hardwareMap) {
        batteryVoltageSensor = HubVoltage.forDevice(hardwareMap, flywheelMotor);
    }

    public VoltageSensor getVoltageSensor() {
        return batteryVoltageSensor;
    }

    public void setVoltageFilter(double samplePeriod, double filterTau) {
        voltageCompensator.setFilter(samplePeriod, filterTau);
    }

    public double getFilteredVolts() {
        return voltageCompensator.volts();
    }

    public double kp;
    public double ki;
    public double kISmash;
    public double kd;
    public double kv;
    public double ks;

    private double ksSignWidth;

    private double targetVelocity;
    private double currentVelocity, velocityEstimate;

    private double lastCurrentVelocity = 0;

    public double p = 0, i = 0, d = 0;
    private DynamicTrapezoidalSum errorSum = new DynamicTrapezoidalSum();
    public double v = 0;
    public double s = 0;

    public double minP, maxP;
    public double minI, maxI;
    public double minD, maxD;

    public double[] getPIDVS() {
        return new double[] {p, i, d, v, s};
    }

    private FlywheelPIDVSCoefficients coefficients;

    public FlywheelPIDVSCoefficients getCoefficients() {
        return coefficients;
    }

    public void setVelocityPIDVSCoefficients(FlywheelPIDVSCoefficients coefficients) {

        this.coefficients = coefficients;

        kd = coefficients.kd;

        ks = coefficients.ks;
        ksSignWidth = coefficients.ksSignWidth;

        kISmash = coefficients.kISmash;

        minP = coefficients.minP;
        maxP = coefficients.maxP;
        minI = coefficients.minI;
        maxI = coefficients.maxI;
        minD = coefficients.minD;
        maxD = coefficients.maxD;
    }

    /// Setting variables that do in fact change
    private void chooseCoefficientsInternal() {

        kp = coefficients.kp(targetVelocity, currentVelocity);

        ki = coefficients.ki(targetVelocity, currentVelocity, errorSum);

        kv = coefficients.kv;
    }

    private double currentTime = 0;

    private double prevTime = 0, prevError = 0, error;

    /// @param velocity in ticks per second
    public void setVelocity(double velocity, boolean allowIntegralReset) {

        if (allowIntegralReset && targetVelocity != velocity) {

            resetIntegral(); //resetting integral when target velocity changes to prevent integral windup
        }

        targetVelocity = velocity;
    }

    private double position = 0;
    private boolean firstTick = true;
    private double startTime;

    private double dt;

    private double getSeconds() {
        return Clock.seconds();
    }

    private double power = 0;

    public void update() {

        boolean wasFirstTick = firstTick;

        //setting start time
        if (firstTick) {

            startTime = getSeconds();
            firstTick = false;
        }

        double now = getSeconds();
        if (voltageCompensator.shouldSample(now)) {
            voltageCompensator.addSample(batteryVoltageSensor.getVoltage(), now);
        }

        double lastPosition = position;
        position = encoder.getCurrentPosition();
        prevTime = currentTime;
        currentTime = getSeconds() - startTime;
        dt = currentTime - prevTime;

        lastCurrentVelocity = currentVelocity;
        velocityEstimate = wasFirstTick || dt <= 0 ? 0 : (position - lastPosition) / dt;
        rawVelocity = encoder.getInternalVelocity();
        currentVelocity = encoder.updateFilteredVelocity(now, rawVelocity);

        chooseCoefficientsInternal();

        error = targetVelocity - currentVelocity;

        //proportional
        p = kp * error;
        p = MathUtil.clamp(p, minP, maxP);

        if (ki != 0 && MathUtil.isFinite(error * dt) && targetVelocity != 0) errorSum.updateSum(dt, error);
        else errorSum.setSum(0); //integral is reset if it's NaN or if targetVelocity is equal to 0

        //i smashing
        if (Math.signum(error) != Math.signum(prevError)) {
            errorSum.setSum(errorSum.getSum() * kISmash);
        }

        //i is prevented from getting too high or too low
        if (ki == 0) {
            errorSum.setSum(0);
            i = 0;
        }
        else {
            double bound0 = minI / ki;
            double bound1 = maxI / ki;
            errorSum.setRawSum(MathUtil.clamp(errorSum.getSum(), Math.min(bound0, bound1), Math.max(bound0, bound1)));
            i = ki * errorSum.getSum();
        }

        //derivative
        d = dt > 0 ? kd * (error - prevError) / dt : 0;
        double batteryVolts = voltageCompensator.volts();
        if (!MathUtil.valueWithinRangeIncludingPoles(d, -batteryVolts, batteryVolts)) d = 0;
        d = MathUtil.clamp(d, minD, maxD);

        //velocity feedforward
        v = kv * targetVelocity;

        //static friction
        s = targetVelocity != 0 ? ks * MathUtil.smoothSign(currentVelocity, ksSignWidth) : 0;

        power = voltageCompensator.toPower(p + i + d + v + s);

        if (targetVelocity == 0) power = 0;

        if (isPIDEnabled) {
            flywheelMotor.setPower(power);
        }

        prevError = error;
    }

    public enum RunningMotor {

        DISABLE(false), ENABLE(true);

        private boolean value;

        RunningMotor(boolean enableOrDisable) {
            value = enableOrDisable;
        }

        public boolean getValue() {
            return value;
        }
    }

    public void setPower(double power) {

        if (isPIDEnabled) throw new IllegalArgumentException("Must disable PID mode!");

        this.power = MathUtil.isFinite(power) ? MathUtil.clamp(power, -1, 1) : 0;
        flywheelMotor.setPower(this.power);
    }

    //default mode is enabled
    private boolean isPIDEnabled = true;

    /// @return if motor is enabled
    public boolean getPIDEnabled() {
        return isPIDEnabled;
    }

    public void runMotor(RunningMotor isPIDEnabled) {
        this.isPIDEnabled = isPIDEnabled.getValue();
    }

    private double rawVelocity = 0;

    public double getRawVelocity() {
        return rawVelocity;
    }

    public double getVelocityEstimate() {
        return velocityEstimate;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getLastCurrentVelocity() {
        return lastCurrentVelocity;
    }

    public double getError() {
        return error;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /// @param velocityMarginOfError Acceptable variation in velocity.
    /// @param stabilityMarginOfError Acceptable variation in stability.
    public boolean isAtVelocityAndStable(double velocityMarginOfError, double stabilityMarginOfError) {

        if (firstTick || !isPIDEnabled || targetVelocity == 0 || !MathUtil.isFinite(targetVelocity) || !MathUtil.isFinite(currentVelocity) || !MathUtil.isFinite(lastCurrentVelocity) || !MathUtil.isFinite(velocityMarginOfError) || velocityMarginOfError < 0 || !MathUtil.isFinite(stabilityMarginOfError) || stabilityMarginOfError < 0) return false;

        boolean isAtVelocity = Math.abs(targetVelocity - currentVelocity) <= velocityMarginOfError;

        boolean isStable = Math.abs(currentVelocity - lastCurrentVelocity) <= stabilityMarginOfError;

        return isAtVelocity && isStable;
    }

    public void reset() {

        encoder.resetVelocityFilter();

        flywheelMotor.setPower(0);
        power = 0;
        p = d = v = s = 0;
        error = velocityEstimate = dt = 0;

        firstTick = true;
        currentTime = 0;
        position = encoder.getCurrentPosition();

        prevError = 0;
        prevTime = 0;

        targetVelocity = 0;
        currentVelocity = 0;

        lastCurrentVelocity = 0;

        setVelocity(0, false); //allowIntegralReset is false to speed up computation because of how '||' works - probably negligible
        resetIntegral(); //integral reset
    }

    private void resetIntegral() {

        errorSum.setSum(0);
        i = 0;
    }

    public double getPower() {
        return power;
    }

    public double getMotorPower() {
        return flywheelMotor.getPower();
    }

    /// @return the dt used in the loop that ran most recently
    public double getLoopDt() {
        return dt;
    }

}
