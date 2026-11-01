package org.firstinspires.ftc.teamcode.Systems.Flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Utility.DynamicTrapezoidalSum;
import org.firstinspires.ftc.teamcode.Utility.Encoder;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;
import org.firstinspires.ftc.teamcode.Utility.VoltageCompensator;

public final class Flywheel {

    private final Encoder encoder;

    public Encoder getEncoder() {
        return encoder;
    }

    private final DcMotorEx flywheelMotor;

    private VoltageSensor batteryVoltageSensor;

    private final VoltageCompensator voltageCompensator = new VoltageCompensator(
            FlywheelConstants.FLYWHEEL_VOLTAGE_SAMPLE_PERIOD,
            FlywheelConstants.FLYWHEEL_VOLTAGE_FILTER_TAU,
            FlywheelConstants.FLYWHEEL_NOMINAL_VOLTS);

    public Flywheel(DcMotorEx flywheelMotor) {

        this.flywheelMotor = flywheelMotor;

        this.flywheelMotor.setDirection(FlywheelConstants.FLYWHEEL_MOTOR_DIRECTION);

        encoder = new Encoder(this.flywheelMotor);
        encoder.setDirection(Encoder.Direction.FORWARD);

        this.flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void initVoltageSensor(HardwareMap hardwareMap) {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
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
        return System.nanoTime() * 1e-9;
    }

    private double power = 0;

    /**
     * Runs the controller, timing the encoder sample right after it is read. Prefer
     * {@link #update(long)} when the caller knows when this loop's bulk read happened.
     */
    public void update() {
        updateInternal(Double.NaN);
    }

    /**
     * Runs the controller with the encoder sample time supplied by the caller.
     *
     * @param sampleNanos {@link System#nanoTime()} taken right after this loop's bulk read, i.e. the
     *                    moment the encoder count was latched (see TeleOpBaseOpMode#getBulkReadNanos)
     */
    public void update(long sampleNanos) {
        updateInternal(sampleNanos * 1e-9);
    }

    private void updateInternal(double sampleSeconds) {

        boolean wasFirstTick = firstTick;

        // Velocity = Δcounts / Δt only holds if Δt is the time between the two samples. Read the
        // encoder (which may trigger the bulk read) and stamp the time immediately; never let a
        // blocking call such as the battery read sit between the sample and its timestamp.
        double lastPosition = position;
        position = encoder.getCurrentPosition();
        double stamp = Double.isNaN(sampleSeconds) ? getSeconds() : sampleSeconds;

        //setting start time
        if (firstTick) {

            startTime = stamp;
            firstTick = false;
            windowCount = 0;
        }

        prevTime = currentTime;
        currentTime = stamp - startTime;
        dt = currentTime - prevTime;

        lastCurrentVelocity = currentVelocity;
        velocityEstimate = wasFirstTick || dt <= 0 ? 0 : (position - lastPosition) / dt;
        currentVelocity = velocityEstimate;
        recordWindowSample(currentTime, position);

        // Battery voltage is not part of the bulk data: it is a separate blocking hub transaction.
        double now = getSeconds();
        if (voltageCompensator.shouldSample(now)) {
            voltageCompensator.addSample(batteryVoltageSensor.getVoltage(), now);
        }

        chooseCoefficientsInternal();

        error = targetVelocity - currentVelocity;

        //proportional
        p = kp * error;
        p = MathUtil.clamp(p, minP, maxP);

        if (ki != 0 && MathUtil.isFinite(error * dt) && targetVelocity != 0) errorSum.updateSum(dt, error);
        else errorSum.setSum(0); //integral is reset if it's NaN or if targetVelocity is equal to 0

        // i smashing
        if (Math.signum(error) != Math.signum(prevError)) {
            errorSum.setSum(errorSum.getSum() * kISmash);
        }

        // i is prevented from getting too high or too low
        if (ki == 0) {
            errorSum.setSum(0);
            i = 0;
        } else {
            double bound0 = minI / ki;
            double bound1 = maxI / ki;
            errorSum.setRawSum(MathUtil.clamp(errorSum.getSum(),
                    Math.min(bound0, bound1), Math.max(bound0, bound1)));
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

    // default mode is enabled
    private boolean isPIDEnabled = true;

    /// @return if motor is enabled
    public boolean getPIDEnabled() {
        return isPIDEnabled;
    }

    public void runMotor(RunningMotor isPIDEnabled) {
        this.isPIDEnabled = isPIDEnabled.getValue();
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
        if (firstTick || !isPIDEnabled || targetVelocity == 0
                || !MathUtil.isFinite(targetVelocity) || !MathUtil.isFinite(currentVelocity)
                || !MathUtil.isFinite(lastCurrentVelocity)
                || !MathUtil.isFinite(velocityMarginOfError) || velocityMarginOfError < 0
                || !MathUtil.isFinite(stabilityMarginOfError) || stabilityMarginOfError < 0) return false;

        boolean isAtVelocity = Math.abs(targetVelocity - currentVelocity) <= velocityMarginOfError;

        boolean isStable = Math.abs(currentVelocity - lastCurrentVelocity) <= stabilityMarginOfError;

        return isAtVelocity && isStable;
    }

    // Recent (time, position) samples for a window-averaged speed. Fixed arrays: no allocation per loop.
    private static final int WINDOW_CAPACITY = 128;
    private final double[] windowTime = new double[WINDOW_CAPACITY];
    private final double[] windowPosition = new double[WINDOW_CAPACITY];
    private int windowHead = 0, windowCount = 0;

    private void recordWindowSample(double time, double pos) {
        windowHead = (windowHead + 1) % WINDOW_CAPACITY;
        windowTime[windowHead] = time;
        windowPosition[windowHead] = pos;
        if (windowCount < WINDOW_CAPACITY) windowCount++;
    }

    /**
     * Average speed over at least {@code windowSeconds}, ticks/s: Δcounts over the span from the
     * newest sample back to the first sample at least that old.
     *
     * <p>A single-loop difference of the 28-count encoder has σ ≈ 0.41 count / (counts per loop):
     * about 1 % at 3700 rpm and 25 ms. Over 100 ms it is about 0.25 %, small enough to gate at 1 %.
     *
     * @return NaN until the samples span the window
     */
    public double getWindowVelocity(double windowSeconds) {
        if (windowCount < 2 || !MathUtil.isFinite(windowSeconds) || windowSeconds <= 0) return Double.NaN;
        double newestTime = windowTime[windowHead];
        for (int k = 1; k < windowCount; k++) {
            int index = (windowHead - k + WINDOW_CAPACITY) % WINDOW_CAPACITY;
            double span = newestTime - windowTime[index];
            if (span >= windowSeconds) return (windowPosition[windowHead] - windowPosition[index]) / span;
        }
        return Double.NaN;
    }

    /** {@link #getWindowVelocity(double)} over {@link FlywheelConstants#FLYWHEEL_READY_WINDOW_SECONDS}. */
    public double getWindowVelocity() {
        return getWindowVelocity(FlywheelConstants.FLYWHEEL_READY_WINDOW_SECONDS);
    }

    /**
     * Ready to fire: the window-averaged speed is within {@code fractionTolerance} of the target.
     * At 3 m, 1 % of exit speed moves the ball ≈ 1.8 in at the target (docs/PHYSICS_ERROR_BUDGET.md),
     * so a tolerance in percent is the physically meaningful gate, not a fixed ticks/s band.
     */
    public boolean isReady(double fractionTolerance) {
        if (firstTick || !isPIDEnabled || targetVelocity == 0 || !MathUtil.isFinite(targetVelocity)
                || !MathUtil.isFinite(fractionTolerance) || fractionTolerance < 0) return false;
        double windowVelocity = getWindowVelocity();
        return MathUtil.isFinite(windowVelocity)
                && Math.abs(targetVelocity - windowVelocity) <= fractionTolerance * Math.abs(targetVelocity);
    }

    /** {@link #isReady(double)} at {@link FlywheelConstants#FLYWHEEL_READY_FRACTION}. */
    public boolean isReady() {
        return isReady(FlywheelConstants.FLYWHEEL_READY_FRACTION);
    }

    public void reset() {

        flywheelMotor.setPower(0);
        power = 0;
        p = d = v = s = 0;
        error = velocityEstimate = dt = 0;
        windowCount = 0;

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
