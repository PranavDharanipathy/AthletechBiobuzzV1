package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Drop-in {@link CRServo} (continuous-rotation servo) that counts and times every call. Also
 * implements {@link PwmControl} whenever the wrapped servo supports it.
 *
 * <pre>
 * CRServo intake = new MonitoredCRServo(hardwareMap, "intake", false);
 * </pre>
 */
public class MonitoredCRServo extends MonitoredDevice<CRServo> implements CRServo, PwmControl {

    private final CacheSlot powerSlot;
    private double lastPower = Double.NaN;
    private double lastPulseWidth = Double.NaN;
    private Direction lastDirection;

    public MonitoredCRServo(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, CRServo.class, name), name, cacheReads);
    }

    public MonitoredCRServo(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    public MonitoredCRServo(CRServo servo, String name, boolean cacheReads) {
        super(servo, name, "CRServo", cacheReads);
        powerSlot = tracker.newSlot();
    }

    @Override
    public ServoController getController() {
        return device.getController();
    }

    @Override
    public int getPortNumber() {
        return device.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        long t0 = tracker.start();
        device.setDirection(direction);
        tracker.write(HwOp.SET_DIRECTION, t0, direction == lastDirection);
        lastDirection = direction;
    }

    @Override
    public Direction getDirection() {
        long t0 = tracker.start();
        Direction v = device.getDirection();
        tracker.read(HwOp.GET_DIRECTION, t0);
        return v;
    }

    @Override
    public void setPower(double power) {
        long t0 = tracker.start();
        device.setPower(power);
        tracker.write(HwOp.SET_POWER, t0, power == lastPower, powerSlot);
        lastPower = power;
        lastPulseWidth = Double.NaN; // the pulse width now follows the power
    }

    @Override
    public double getPower() {
        if (tracker.cacheHit(HwOp.GET_POWER, powerSlot, CacheSlot.NO_KEY)) return powerSlot.d;
        long t0 = tracker.start();
        double v = device.getPower();
        powerSlot.d = v;
        tracker.read(HwOp.GET_POWER, t0, powerSlot, CacheSlot.NO_KEY);
        return v;
    }

    // ------------------------------------------------------------------ PwmControl

    public boolean supportsPwmControl() {
        return device instanceof PwmControl;
    }

    private PwmControl pwm() {
        if (device instanceof PwmControl) return (PwmControl) device;
        throw new UnsupportedOperationException(
                "CRServo '" + getName() + "' (" + device.getClass().getSimpleName()
                        + ") does not support PwmControl");
    }

    @Override
    public void setPwmRange(PwmRange range) {
        PwmControl p = pwm();
        long t0 = tracker.start();
        p.setPwmRange(range);
        tracker.write(HwOp.SET_PWM_RANGE, t0, false);
        lastPower = Double.NaN;
    }

    @Override
    public PwmRange getPwmRange() {
        PwmControl p = pwm();
        long t0 = tracker.start();
        PwmRange v = p.getPwmRange();
        tracker.read(HwOp.GET_PWM_RANGE, t0);
        return v;
    }

    @Override
    public void setPwmEnable() {
        PwmControl p = pwm();
        long t0 = tracker.start();
        p.setPwmEnable();
        tracker.write(HwOp.SET_PWM_ENABLE, t0, false);
    }

    @Override
    public void setPwmDisable() {
        PwmControl p = pwm();
        long t0 = tracker.start();
        p.setPwmDisable();
        tracker.write(HwOp.SET_PWM_DISABLE, t0, false);
    }

    @Override
    public boolean isPwmEnabled() {
        PwmControl p = pwm();
        long t0 = tracker.start();
        boolean v = p.isPwmEnabled();
        tracker.read(HwOp.IS_PWM_ENABLED, t0);
        return v;
    }

    /**
     * Commands the servo directly by pulse width in microseconds (typically 500-2500; the middle
     * of the range is stop), bypassing {@link #setPower(double)}. Clears the cached power, and
     * repeating the previous width counts as a redundant write.
     */
    @Override
    public void setPulseWidth(double usWidth) {
        PwmControl p = pwm();
        long t0 = tracker.start();
        p.setPulseWidth(usWidth);
        tracker.write(HwOp.SET_PULSE_WIDTH, t0, usWidth == lastPulseWidth, powerSlot);
        lastPulseWidth = usWidth;
        lastPower = Double.NaN; // moved without setPower: the next setPower is real
    }

    /** The commanded pulse width in microseconds. */
    @Override
    public double getPulseWidth() {
        PwmControl p = pwm();
        long t0 = tracker.start();
        double v = p.getPulseWidth();
        tracker.read(HwOp.GET_PULSE_WIDTH, t0);
        return v;
    }
}
