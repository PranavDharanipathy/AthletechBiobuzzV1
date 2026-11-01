package org.firstinspires.ftc.teamcode.Systems.Turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Utility.DynamicTrapezoidalSum;
import org.firstinspires.ftc.teamcode.Utility.Encoder;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredCRServo;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.MathUtil;
import org.firstinspires.ftc.teamcode.Utility.VoltageCompensator;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_PD_POSITIONS;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KPS;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KDS;

import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_FEEDFORWARD_POSITIONS;
import static org.firstinspires.ftc.teamcode.Constants.TurretConstants.TURRET_KFS;

import java.util.Collections;

public class TurretBase {

    private final CRServo leftTurretBase;
    private final CRServo rightTurretBase;

    private final Encoder encoder;

    public double kp, kiFar, kiClose, kd, kISmash;
    public double ki, kHold;
    public double holdDecay;

    private double maxI = 1;
    private double minI = -1;

    public double[] dActivation = {0, (double) Integer.MAX_VALUE};

    private double[] iSwitch;

    private DynamicTrapezoidalSum errorSum = new DynamicTrapezoidalSum();

    public double p, i, d = 0, f;

    private double power = 0;

    private final VoltageSensor batteryVoltageSensor;

    private final VoltageCompensator voltageCompensator = new VoltageCompensator(
            TurretConstants.TURRET_VOLTAGE_SAMPLE_PERIOD,
            TurretConstants.TURRET_VOLTAGE_FILTER_TAU,
            TurretConstants.TURRET_NOMINAL_VOLTS);

    public TurretBase(HardwareMap hardwareMap) {

        this (hardwareMap, null); //using encoder's current position
    }

    /// @param turretStartPosition for re-zeroing the turret compensating for the home position not always being at 0
    public TurretBase(HardwareMap hardwareMap, Double turretStartPosition) {
        this(new MonitoredCRServo(hardwareMap, DeviceConstants.leftTurretCRServoDeviceName, false),
                new MonitoredCRServo(hardwareMap, DeviceConstants.rightTurretCRServoDeviceName, false),
                new Encoder(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.turretEncoderDeviceName, false)),
                hardwareMap.voltageSensor.iterator().next(), turretStartPosition);
    }

    /** Device injection keeps controller tests independent of Android hardware discovery. */
    public TurretBase(CRServo left, CRServo right, Encoder angleEncoder,
                      VoltageSensor voltageSensor, Double turretStartPosition) {
        leftTurretBase = java.util.Objects.requireNonNull(left, "left servo");
        rightTurretBase = java.util.Objects.requireNonNull(right, "right servo");

        leftTurretBase.setDirection(TurretConstants.TURRET_BASE_DIRECTIONS[0]);
        rightTurretBase.setDirection(TurretConstants.TURRET_BASE_DIRECTIONS[1]);

        encoder = java.util.Objects.requireNonNull(angleEncoder, "encoder");
        encoder.setDirection(TurretConstants.TURRET_ENCODER_DIRECTION);

        batteryVoltageSensor = java.util.Objects.requireNonNull(voltageSensor, "voltage sensor");

        // first targetPosition is the start position
        double tsp = turretStartPosition != null ? turretStartPosition : encoder.getCurrentPosition();
        lastCurrentPosition = currentPosition = lastTargetPosition = targetPosition = startPosition = tsp;
    }

    private boolean reversed = false;

    private static DcMotorSimple.Direction flip(DcMotorSimple.Direction direction) {
        return direction == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    /// Call after setting PIDFS coefficients
    public void reverse() {

        leftTurretBase.setDirection(flip(leftTurretBase.getDirection()));
        rightTurretBase.setDirection(flip(rightTurretBase.getDirection()));
        encoder.setDirection(encoder.getDirection() == Encoder.Direction.FORWARD ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);

        startPosition = -startPosition;
        targetPosition = -targetPosition;
        lastTargetPosition = -lastTargetPosition;
        currentPosition = -currentPosition;
        lastCurrentPosition = -lastCurrentPosition;

        TURRET_PD_POSITIONS.replaceAll(i -> -i);
        TURRET_FEEDFORWARD_POSITIONS.replaceAll(i -> -i);

        Collections.reverse(TURRET_PD_POSITIONS);
        Collections.reverse(TURRET_FEEDFORWARD_POSITIONS);

        Collections.reverse(TURRET_KPS);
        Collections.reverse(TURRET_KDS);
        Collections.reverse(TURRET_KFS);

        reversed = true;
    }

    public TurretBasePIDFCoefficients coefficients;

    public void setPositionalCoefficients(TurretBasePIDFCoefficients coefficients) {

        this.coefficients = coefficients;

        //setting variables that do not change

        iSwitch = coefficients.iSwitch;

        holdDecay = coefficients.holdDecay;

        dActivation = coefficients.dActivation;

        minI = coefficients.minI;
        maxI = coefficients.maxI;
    }

    /// @param tuning true means that turret's in tuning mode while false means that turret is in normal mode.
    /// If the object isn't initialized, nothing will happen and the method will deal with the error.
    public void setTuning(boolean tuning) {
        coefficients.setTuning(tuning);
    }

    public enum PD_INTERPOLATION_MODE {

        NONE, P, D, BOTH;

        /**
         * <p>"00" - Indicates NONE
         * <p>"10" - Indicates P
         * <p>"01" - Indicates D
         * <p>"11" - Indicates BOTH
         * @throws IllegalArgumentException If an invalid mode is inputted
         */
        public static PD_INTERPOLATION_MODE fromString(String mode) {

            switch (mode) {

                case "00":
                    return NONE;

                case "10":
                    return P;

                case "01":
                    return D;

                case "11":
                    return BOTH;

                default:
                    throw new IllegalArgumentException(mode + " is an invalid mode string!");
            }
        }
    }

    public PD_INTERPOLATION_MODE pdInterpolationMode = PD_INTERPOLATION_MODE.BOTH;

    /// To be able to set from FTC Dashboard
    public void setPdInterpolationMode(PD_INTERPOLATION_MODE mode) {
        pdInterpolationMode = mode;
    }

    /// Setting variables that do in fact change
    private void chooseCoefficientsInternal(TurretBasePIDFCoefficients.TurretSide side) {

        if (pdInterpolationMode.equals(PD_INTERPOLATION_MODE.BOTH)) {

            double[] kpAndKd = coefficients.kpAndKd(targetPosition, currentPosition, startPosition);
            kp = kpAndKd[0];
            kd = kpAndKd[1];
        }
        else { //if pdInterpolationMode equals PD_INTERPOLATION_MODE.P, PD_INTERPOLATION_MODE.D, or PD_INTERPOLATION_MODE.NONE

            kp = coefficients.kp(targetPosition, currentPosition, startPosition);
            kd = coefficients.kd(targetPosition, currentPosition, startPosition);
        }

        kiFar = coefficients.kiFar(side);
        kiClose = coefficients.kiClose(side);
        kHold = coefficients.kHold(targetPosition, startPosition);

        kISmash = coefficients.kISmash(side);
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

    private double prevError, error;
    private Double initialError = null;
    private double prevTime, currTime, dt;

    /** Clock origin (System.nanoTime) so times are ms since construction, as with the old ElapsedTime. */
    private final long clockOriginNanos = System.nanoTime();

    /**
     * Runs the controller, timing the encoder sample right after it is read. Prefer
     * {@link #update(long)} when the caller knows when this loop's bulk read happened.
     */
    public void update() {
        updateInternal(Double.NaN);
    }

    /**
     * @param sampleNanos {@link System#nanoTime()} taken right after this loop's bulk read, i.e. the
     *                    moment the encoder count was latched (see TeleOpBaseOpMode#getBulkReadNanos)
     */
    public void update(long sampleNanos) {
        updateInternal((sampleNanos - clockOriginNanos) / 1e6);
    }

    private void updateInternal(double sampleMillis) {

        // Read the encoder (which may trigger the bulk read), then stamp the time: dt must be the
        // time between samples, with no blocking call (e.g. a battery read) in between.
        lastCurrentPosition = currentPosition;
        currentPosition = getCurrentPosition();

        currTime = Double.isNaN(sampleMillis) ? (System.nanoTime() - clockOriginNanos) / 1e6 : sampleMillis;
        dt = currTime - prevTime;

        // CR servos run off the hub's regulated servo rail, not the battery, so battery-scaling their
        // command changes the loop gain (±8–9 % over 11–13 V) for no physical reason. The battery
        // read is also a blocking hub transaction. Only done if explicitly re-enabled.
        if (TurretConstants.TURRET_BATTERY_COMPENSATION) {
            double now = MathUtil.millisecondsToSeconds(currTime);
            if (voltageCompensator.shouldSample(now)) {
                voltageCompensator.addSample(batteryVoltageSensor.getVoltage(), now);
            }
        }

        error = targetPosition - currentPosition;

        if (initialError == null) initialError = error;

        chooseCoefficientsInternal(TurretBasePIDFCoefficients.TurretSide.getSide(targetPosition, startPosition, reversed));

        boolean fineTune = Math.abs(error) < iSwitch[0];

        //proportional
        p = kp * error;

        //integral
        if (Math.abs(error) > iSwitch[1]) ki = 0;
        else if (fineTune) ki = kiClose;
        else ki = kiFar;

        if (ki != 0 && dt > 0 && MathUtil.isFinite(error * dt)) errorSum.updateSum(dt, error);
        if (Math.signum(error) != Math.signum(prevError)) {
            errorSum.setSum(errorSum.getSum() * kISmash);
        }
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
        d = dt > 0 && MathUtil.valueWithinRange(Math.abs(error), dActivation[0], dActivation[1]) ? kd * (error - prevError) / dt : 0;

        //feedforward
        if (kHold == 0 || error == 0) {
            f = 0;
        } else {
            // Zero smoothing width is the limiting step function, not 0/0 at rest.
            double activation = holdDecay == 0 ? 1 :
                    -Math.expm1(-Math.abs(error) / (holdDecay * ShooterConstants.TURRET_TICKS_PER_DEGREE));
            f = kHold * Math.signum(error) * activation;
        }

        // Gains stay in "volts at the nominal 12 V" so existing tuning keeps its meaning.
        double command = p + i + d + f;
        if (TurretConstants.TURRET_BATTERY_COMPENSATION) {
            power = voltageCompensator.toPower(command);
        } else {
            power = MathUtil.isFinite(command)
                    ? MathUtil.clamp(command / TurretConstants.TURRET_NOMINAL_VOLTS, -1, 1) : 0;
        }

        leftTurretBase.setPower(power);
        rightTurretBase.setPower(power);

        prevTime = currTime;
        prevError = error;
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

    /**
     * Hub velocity, ticks/s, with its 16-bit wrap removed. At 68.27 ticks/° the raw value wraps
     * above ≈ 480 °/s of turret, which the 1:1 Axon Max drive can approach.
     */
    public double getVelocity() {
        double estimate = dt > 0 ? (currentPosition - lastCurrentPosition) / MathUtil.millisecondsToSeconds(dt)
                : Double.NaN;
        return encoder.getInternalVelocity(estimate);
    }

    public double getPower() {
        return power;
    }

    public double getLoopDt() {
        return MathUtil.millisecondsToSeconds(dt);
    }

    public double[] getServoPowers() {
        return new double[] {leftTurretBase.getPower(), rightTurretBase.getPower()};
    }

}
