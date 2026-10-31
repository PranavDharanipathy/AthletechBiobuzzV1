package org.firstinspires.ftc.teamcode.Systems.Turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        leftTurretBase = new MonitoredCRServo(hardwareMap, DeviceConstants.leftTurretCRServoDeviceName, false);
        rightTurretBase = new MonitoredCRServo(hardwareMap, DeviceConstants.rightTurretCRServoDeviceName, false);

        leftTurretBase.setDirection(TurretConstants.TURRET_BASE_DIRECTIONS[0]);
        rightTurretBase.setDirection(TurretConstants.TURRET_BASE_DIRECTIONS[1]);

        encoder = new Encoder(new MonitoredDcMotorEx(hardwareMap, DeviceConstants.turretEncoderDeviceName, false));
        encoder.setDirection(TurretConstants.TURRET_ENCODER_DIRECTION);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

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

    private final ElapsedTime timer = new ElapsedTime();

    public void update() {

        currTime = timer.milliseconds();
        dt = currTime - prevTime;

        double now = MathUtil.millisecondsToSeconds(currTime);
        if (voltageCompensator.shouldSample(now)) {
            voltageCompensator.addSample(batteryVoltageSensor.getVoltage(), now);
        }

        lastCurrentPosition = currentPosition;
        currentPosition = getCurrentPosition();

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

        if (dt != 0) errorSum.updateSum(dt, error);
        if (Math.signum(error) != Math.signum(prevError)) {
            errorSum.setSum(errorSum.getSum() * kISmash);
        }
        errorSum.setRawSum(MathUtil.clamp(errorSum.getSum(), (minI / ki) /*integrated error min*/, (maxI / ki) /*integrated error max*/));
        i = ki * errorSum.getSum();
        //i = MathUtil.clamp(ki * errorSum.getSum(), minI, maxI);

        //derivative
        d = dt > 0 && MathUtil.valueWithinRange(Math.abs(error), dActivation[0], dActivation[1]) ? kd * (error - prevError) / dt : 0;

        //feedforward
        f = kHold * Math.signum(error) * (1.0 - Math.exp(-Math.abs(error) / (holdDecay * ShooterConstants.TURRET_TICKS_PER_DEGREE)));

        power = voltageCompensator.toPower(p + i + d + f);

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

    public double getVelocity() {
        return encoder.getInternalVelocity();
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
