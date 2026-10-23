package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Drop-in {@link Servo} that counts and times every call. Also implements {@link PwmControl}
 * (what {@code ServoImplEx} adds) whenever the wrapped servo supports it.
 *
 * <pre>
 * Servo claw = new MonitoredServo(hardwareMap, "claw", false);
 * </pre>
 *
 * Note: {@link #getPosition()} and {@link #getPulseWidth()} return the last <i>commanded</i>
 * value - standard servos have no position feedback - so they are counted but never cached (the
 * SDK answers them locally).
 * {@link #getStats()} {@code .getRedundantWrites()} tells you how often you re-send the same
 * position.
 */
public class MonitoredServo extends MonitoredDevice<Servo> implements Servo, PwmControl {

    private double lastPosition = Double.NaN;
    private double lastPulseWidth = Double.NaN;
    private Direction lastDirection;

    public MonitoredServo(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, Servo.class, name), name, cacheReads);
    }

    public MonitoredServo(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    public MonitoredServo(Servo servo, String name, boolean cacheReads) {
        super(servo, name, "Servo", cacheReads);
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
    public void setPosition(double position) {
        long t0 = tracker.start();
        device.setPosition(position);
        tracker.write(HwOp.SET_POSITION, t0, position == lastPosition);
        lastPosition = position;
        lastPulseWidth = Double.NaN; // the pulse width now follows the position
    }

    @Override
    public double getPosition() {
        long t0 = tracker.start();
        double v = device.getPosition();
        tracker.read(HwOp.GET_POSITION, t0);
        return v;
    }

    @Override
    public void scaleRange(double min, double max) {
        long t0 = tracker.start();
        device.scaleRange(min, max);
        tracker.write(HwOp.SCALE_RANGE, t0, false);
        lastPosition = Double.NaN; // same number now means a different pulse width
    }

    // ------------------------------------------------------------------ PwmControl

    /** Whether the wrapped servo supports {@link PwmControl} (true for REV hub servos). */
    public boolean supportsPwmControl() {
        return device instanceof PwmControl;
    }

    private PwmControl pwm() {
        if (device instanceof PwmControl) return (PwmControl) device;
        throw new UnsupportedOperationException(
                "Servo '" + getName() + "' (" + device.getClass().getSimpleName()
                        + ") does not support PwmControl");
    }

    @Override
    public void setPwmRange(PwmRange range) {
        PwmControl p = pwm();
        long t0 = tracker.start();
        p.setPwmRange(range);
        tracker.write(HwOp.SET_PWM_RANGE, t0, false);
        lastPosition = Double.NaN;
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
     * Commands the servo directly by pulse width in microseconds (typically 500-2500), bypassing
     * {@link #setPosition(double)}. Repeating the previous width counts as a redundant write.
     */
    @Override
    public void setPulseWidth(double usWidth) {
        PwmControl p = pwm();
        long t0 = tracker.start();
        p.setPulseWidth(usWidth);
        tracker.write(HwOp.SET_PULSE_WIDTH, t0, usWidth == lastPulseWidth);
        lastPulseWidth = usWidth;
        lastPosition = Double.NaN; // moved without setPosition: the next setPosition is real
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
