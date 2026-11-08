package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

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
        tracker.v(direction);
        tracker.write(HwOp.SET_DIRECTION, t0, direction == lastDirection);
        lastDirection = direction;
    }

    @Override
    public Direction getDirection() {

        long t0 = tracker.start();
        Direction v = device.getDirection();
        tracker.v(v);
        tracker.read(HwOp.GET_DIRECTION, t0);
        return v;
    }

    @Override
    public void setPosition(double position) {

        long t0 = tracker.start();
        device.setPosition(position);
        tracker.v(position);
        tracker.write(HwOp.SET_POSITION, t0, position == lastPosition);
        lastPosition = position;
        lastPulseWidth = Double.NaN;
    }

    @Override
    public double getPosition() {

        long t0 = tracker.start();
        double v = device.getPosition();
        tracker.v(v);
        tracker.read(HwOp.GET_POSITION, t0);
        return v;
    }

    @Override
    public void scaleRange(double min, double max) {

        long t0 = tracker.start();
        device.scaleRange(min, max);
        tracker.v(min);
        tracker.write(HwOp.SCALE_RANGE, t0, false);
        lastPosition = Double.NaN;
    }

    public boolean supportsPwmControl() {
        return device instanceof PwmControl;
    }

    private PwmControl pwm() {

        if (device instanceof PwmControl) return (PwmControl) device;
        throw new UnsupportedOperationException("Servo '" + getName() + "' (" + device.getClass().getSimpleName() + ") does not support PwmControl");
    }

    @Override
    public void setPwmRange(PwmRange range) {

        PwmControl p = pwm();
        long t0 = tracker.start();
        p.setPwmRange(range);
        tracker.v(range);
        tracker.write(HwOp.SET_PWM_RANGE, t0, false);
        lastPosition = Double.NaN;
    }

    @Override
    public PwmRange getPwmRange() {

        PwmControl p = pwm();
        long t0 = tracker.start();
        PwmRange v = p.getPwmRange();
        tracker.v(v);
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
        tracker.v(v);
        tracker.read(HwOp.IS_PWM_ENABLED, t0);
        return v;
    }

    @Override
    public void setPulseWidth(double usWidth) {

        PwmControl p = pwm();
        long t0 = tracker.start();
        p.setPulseWidth(usWidth);
        tracker.v(usWidth);
        tracker.write(HwOp.SET_PULSE_WIDTH, t0, usWidth == lastPulseWidth);
        lastPulseWidth = usWidth;
        lastPosition = Double.NaN;
    }

    @Override
    public double getPulseWidth() {

        PwmControl p = pwm();
        long t0 = tracker.start();
        double v = p.getPulseWidth();
        tracker.v(v);
        tracker.read(HwOp.GET_PULSE_WIDTH, t0);
        return v;
    }
}
