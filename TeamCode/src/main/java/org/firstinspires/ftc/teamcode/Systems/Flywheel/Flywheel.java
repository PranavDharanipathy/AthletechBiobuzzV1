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

        observer.accelPerVolt = FlywheelConstants.FLYWHEEL_ACCEL_PER_VOLT;
        observer.coastVolts = FlywheelConstants.FLYWHEEL_COAST_VOLTS;
        observer.lag = FlywheelConstants.FLYWHEEL_OBSERVER_LAG;
        observer.gain = FlywheelConstants.FLYWHEEL_OBSERVER_GAIN;
        observer.loadGain = FlywheelConstants.FLYWHEEL_OBSERVER_LOAD_GAIN;
        observer.maxLoad = FlywheelConstants.FLYWHEEL_MAX_LOAD_VOLTS;
        observer.contactDrop = FlywheelConstants.FLYWHEEL_CONTACT_DROP;
    }

    //always runs (ball contacts and logs come from it); the controller uses it instead of the filtered speed when on
    public final FlywheelObserver observer = new FlywheelObserver();
    public boolean useObserver = FlywheelConstants.FLYWHEEL_OBSERVER;
    //this shot's window, ticks/s under / over the target that still go in (Shooter, from CellGeometry)
    public double windowBelow = FlywheelConstants.FEED_GATE_DEFAULT_BELOW, windowAbove = FlywheelConstants.FEED_GATE_DEFAULT_ABOVE;

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

    public double load = 0;

    public double feedBoostVolts = FlywheelConstants.FLYWHEEL_FEED_BOOST_VOLTS, shotKickVolts = FlywheelConstants.FLYWHEEL_SHOT_KICK_VOLTS;
    public double shotKickTime = FlywheelConstants.FLYWHEEL_SHOT_KICK_TIME, shotDetectDrop = FlywheelConstants.FLYWHEEL_SHOT_DETECT_DROP;
    public double boost = 0;
    public int shotsSeen = 0;
    public double preKickVolts = FlywheelConstants.FLYWHEEL_PRE_KICK_VOLTS, preKickLead = FlywheelConstants.FLYWHEEL_PRE_KICK_LEAD, preKickHold = FlywheelConstants.FLYWHEEL_PRE_KICK_HOLD;
    public double firstContactSeconds = FlywheelConstants.FLYWHEEL_FIRST_CONTACT_SECONDS, ballIntervalSeconds = FlywheelConstants.FLYWHEEL_BALL_INTERVAL_SECONDS;
    public double expectedContact = Double.NaN;
    private double feedStart = Double.NaN, lastContact = Double.NaN;
    private int burstContacts;
    private boolean wasFeedingTick;
    private boolean feeding = false;
    private double feedEnd = Double.NEGATIVE_INFINITY, shutAt = Double.NEGATIVE_INFINITY;
    private boolean cleanStart = true;
    private int observerContacts;
    public double lastContactTime = Double.NaN, lastContactSpeed = Double.NaN, lastContactError = Double.NaN;
    private double kickUntil = -1;
    //while feeding it aims this share of the shot's fast-side window over the target: balls come ~0.17 s apart (2028-05-15 logs),
    //faster than it can see a ball and win the speed back, so every ball after the first hits it slow
    public double burstLiftShare = FlywheelConstants.FLYWHEEL_BURST_LIFT_SHARE;
    //one continuous shot: this share of what the balls take on average (learned drop / learned gap) pushed from lead before the
    //first ball until they stop coming
    public double burstFeedShare = FlywheelConstants.FLYWHEEL_BURST_FEED_SHARE, burstFeedLead = FlywheelConstants.FLYWHEEL_BURST_FEED_LEAD;
    private double aim;

    public void setFeeding(boolean feeding) {
        this.feeding = feeding;
    }

    public boolean isFeeding() {
        return feeding;
    }
    private double lastSentVolts = 0;
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
        observer.kv = coefficients.kv;
        observer.ks = coefficients.ks;
        observer.ksSignWidth = coefficients.ksSignWidth;

        kISmash = coefficients.kISmash;

        minP = coefficients.minP;
        maxP = coefficients.maxP;
        minI = coefficients.minI;
        maxI = coefficients.maxI;
        minD = coefficients.minD;
        maxD = coefficients.maxD;
    }

    private void chooseCoefficientsInternal() {

        kp = coefficients.kp(targetVelocity, currentVelocity);

        ki = coefficients.ki(targetVelocity, currentVelocity, errorSum);

        kv = coefficients.kv;
    }

    private double currentTime = 0;

    private double prevTime = 0, prevError = 0, error;

    public void setVelocity(double velocity, boolean allowIntegralReset) {

        if (allowIntegralReset && targetVelocity != velocity) {

            resetIntegral();
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
        double lastFiltered = filteredVelocity;
        filteredVelocity = encoder.updateFilteredVelocity(now, rawVelocity);
        if (feeding) feedEnd = now;
        //balls already past the blocker still hit after it closes
        observer.update(now, rawVelocity, now - feedEnd <= FlywheelConstants.FLYWHEEL_CONTACT_AFTER_CLOSE);
        currentVelocity = useObserver ? observer.velocity : filteredVelocity;
        boolean contact = observer.contacts != observerContacts;
        observerContacts = observer.contacts;
        if (contact) {
            lastContactTime = observer.contactTime;
            lastContactSpeed = observer.contactSpeed;
            lastContactError = observer.contactSpeed - targetVelocity;
        }

        chooseCoefficientsInternal();

        boolean lifting = targetVelocity != 0 && now - feedEnd <= 0.15 && MathUtil.isFinite(windowAbove);
        aim = targetVelocity + (lifting ? burstLiftShare * windowAbove : 0);
        error = aim - currentVelocity;

        v = kv * aim;

        s = targetVelocity != 0 ? ks * MathUtil.smoothSign(currentVelocity, ksSignWidth) : 0;

        p = kp * error;
        p = MathUtil.clamp(p, Math.min(minP, -(v + s + Math.max(i, 0))), maxP);

        if (ki != 0 && MathUtil.isFinite(error * dt) && targetVelocity != 0) errorSum.updateSum(dt, error);
        else errorSum.setSum(0);

        if (Math.signum(error) != Math.signum(prevError)) {
            errorSum.setSum(errorSum.getSum() * kISmash);
        }

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

        d = dt > 0 ? kd * (error - prevError) / dt : 0;
        double batteryVolts = voltageCompensator.volts();
        if (!MathUtil.valueWithinRangeIncludingPoles(d, -batteryVolts, batteryVolts)) d = 0;
        d = MathUtil.clamp(d, minD, maxD);

        double observerTime = FlywheelConstants.FLYWHEEL_LOAD_OBSERVER_TIME;
        if (useObserver) load = targetVelocity == 0 ? 0 : observer.load;
        else if (targetVelocity == 0 || !(observerTime > 0)) load = 0;
        else if (!wasFirstTick && dt > 0 && Math.abs(error) <= FlywheelConstants.FLYWHEEL_VELOCITY_MARGIN_OF_ERROR && Math.abs(power) < 0.98) {

            double accel = (currentVelocity - lastCurrentVelocity) / dt;
            double needed = kv * currentVelocity + ks * MathUtil.smoothSign(currentVelocity, ksSignWidth) + accel / FlywheelConstants.FLYWHEEL_ACCEL_PER_VOLT;
            load += (lastSentVolts - needed - load) * Math.min(1, dt / observerTime);
            load = MathUtil.clamp(load, -FlywheelConstants.FLYWHEEL_MAX_LOAD_VOLTS, FlywheelConstants.FLYWHEEL_MAX_LOAD_VOLTS);
        }

        if (feeding && !wasFeedingTick) {
            //reopened with balls still on the way (the feed gate): the first ball to hit isn't one this opening sent
            cleanStart = now - shutAt >= FlywheelConstants.FLYWHEEL_CONTACT_AFTER_CLOSE;
            feedStart = now;
            burstContacts = 0;
            lastContact = Double.NaN;
        }
        if (!feeding && wasFeedingTick) shutAt = now;
        wasFeedingTick = feeding;
        boost = 0;
        if (targetVelocity != 0 && feeding) {
            boost = feedBoostVolts;
            boolean dropped = !wasFirstTick && lastFiltered - filteredVelocity > shotDetectDrop;
            if (dropped) {
                kickUntil = now + shotKickTime;
                shotsSeen++;
            }
            //the observer dates a ball best; the filtered drop also fires on recovery wobble, so it's only for when the observer is off.
            //One dip spans a few loops: a new ball only after 80 ms
            boolean ball = useObserver ? contact : dropped;
            double at = contact ? observer.contactTime : now;
            if (ball && (burstContacts == 0 || at - lastContact > 0.08)) {
                if (burstContacts == 0) {
                    if (cleanStart) firstContactSeconds += 0.3 * (MathUtil.clamp(at - feedStart, 0.05, 1.5) - firstContactSeconds);
                }
                else ballIntervalSeconds += 0.3 * (MathUtil.clamp(at - lastContact, 0.08, 1.5) - ballIntervalSeconds);
                burstContacts++;
                lastContact = at;
            }
            expectedContact = burstContacts == 0 ? feedStart + firstContactSeconds : lastContact + ballIntervalSeconds;
            if (now >= kickUntil && now >= expectedContact - preKickLead && now < expectedContact + preKickHold) boost += preKickVolts;
        }
        else expectedContact = Double.NaN;
        if (targetVelocity != 0 && now < kickUntil) boost += shotKickVolts;
        //balls come faster than it can see one and catch up (~60-85 ms), so it carries them instead
        if (targetVelocity != 0 && burstFeedShare > 0) {
            boolean flowing = feeding ? now >= feedStart + firstContactSeconds - burstFeedLead : burstContacts > 0 && now - feedEnd <= 0.15;
            boolean stopped = burstContacts > 0 && now - lastContact > 1.5 * ballIntervalSeconds;
            if (flowing && !stopped) boost += burstFeedShare * observer.ballDrop / (Math.max(0.08, ballIntervalSeconds) * observer.accelPerVolt);
        }

        power = voltageCompensator.toPower(p + i + d + v + s + load + boost);
        lastSentVolts = power * voltageCompensator.volts();

        if (targetVelocity == 0) power = 0;
        if (isPIDEnabled) observer.setVolts(power * voltageCompensator.volts());

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
        observer.setVolts(this.power * voltageCompensator.volts());
    }

    private boolean isPIDEnabled = true;

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

    private double filteredVelocity = 0;

    //the encoder's filtered speed, whichever one the controller uses
    public double getFilteredVelocity() {
        return filteredVelocity;
    }

    public double getObserverVelocity() {
        return observer.velocity;
    }

    //speed `seconds` from now if the controller does its job and balls hit at the first `balls` of `hits`, for the feed gate
    public double predictVelocity(double seconds, double[] hits, int balls) {
        return observer.predict(aim != 0 ? aim : targetVelocity, seconds, kp, voltageCompensator.volts(), hits, balls);
    }

    public int getContacts() {
        return observer.contacts;
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

    public boolean isAtVelocityAndStable(double velocityMarginOfError, double stabilityMarginOfError) {

        if (firstTick || !isPIDEnabled || targetVelocity == 0 || !MathUtil.isFinite(targetVelocity) || !MathUtil.isFinite(currentVelocity) || !MathUtil.isFinite(lastCurrentVelocity) || !MathUtil.isFinite(velocityMarginOfError) || velocityMarginOfError < 0 || !MathUtil.isFinite(stabilityMarginOfError) || stabilityMarginOfError < 0) return false;

        boolean isAtVelocity = Math.abs(targetVelocity - currentVelocity) <= velocityMarginOfError;

        boolean isStable = Math.abs(currentVelocity - lastCurrentVelocity) <= stabilityMarginOfError;

        return isAtVelocity && isStable;
    }

    public void reset() {

        encoder.resetVelocityFilter();
        observer.reset();
        observerContacts = observer.contacts;

        flywheelMotor.setPower(0);
        power = 0;
        p = d = v = s = load = lastSentVolts = 0;
        error = velocityEstimate = dt = 0;

        firstTick = true;
        currentTime = 0;
        position = encoder.getCurrentPosition();

        prevError = 0;
        prevTime = 0;

        targetVelocity = 0;
        currentVelocity = filteredVelocity = 0;

        lastCurrentVelocity = 0;

        setVelocity(0, false);
        resetIntegral();
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

    public double getLoopDt() {
        return dt;
    }

}
