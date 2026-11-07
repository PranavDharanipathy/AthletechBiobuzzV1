package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

public class MonitoredDcMotor extends MonitoredDevice<DcMotor> implements DcMotor {

    protected final CacheSlot powerSlot;
    protected final CacheSlot positionSlot;
    protected final CacheSlot busySlot;

    private double lastPower = Double.NaN;
    private Direction lastDirection;
    private RunMode lastMode;
    private ZeroPowerBehavior lastZeroPowerBehavior;
    private int lastTargetPosition;
    private boolean hasTargetPosition;

    private volatile double commandedPower;
    private volatile long commandedPowerNs = -1;
    protected volatile double observedCurrentAmps = Double.NaN;
    protected volatile long observedCurrentNs = -1;
    protected volatile double observedVelocity = Double.NaN;
    protected volatile long observedVelocityNs = -1;
    private volatile double windingResistanceOhms = Double.NaN;

    public MonitoredDcMotor(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, DcMotor.class, name), name, cacheReads);
    }

    public MonitoredDcMotor(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    public MonitoredDcMotor(DcMotor motor, String name, boolean cacheReads) {
        this(motor, name, "DcMotor", cacheReads);
    }

    protected MonitoredDcMotor(DcMotor motor, String name, String type, boolean cacheReads) {

        super(motor, name, type, cacheReads);
        powerSlot = tracker.newSlot();
        positionSlot = tracker.newSlot();
        busySlot = tracker.newSlot();
        HardwareMonitor.get().registerMotor(this);
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
        tracker.write(HwOp.SET_POWER, t0, power == lastPower, powerSlot, busySlot);
        lastPower = power;
        commandedPower = power;
        commandedPowerNs = t0;
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

    @Override
    public MotorConfigurationType getMotorType() {

        long t0 = tracker.start();
        MotorConfigurationType v = device.getMotorType();
        tracker.read(HwOp.GET_MOTOR_TYPE, t0);
        return v;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {

        long t0 = tracker.start();
        device.setMotorType(motorType);
        tracker.write(HwOp.SET_MOTOR_TYPE, t0, false);
    }

    @Override
    public DcMotorController getController() {
        return device.getController();
    }

    @Override
    public int getPortNumber() {
        return device.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

        long t0 = tracker.start();
        device.setZeroPowerBehavior(zeroPowerBehavior);
        tracker.writeKeepCache(HwOp.SET_ZERO_POWER_BEHAVIOR, t0, zeroPowerBehavior == lastZeroPowerBehavior);
        lastZeroPowerBehavior = zeroPowerBehavior;
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {

        long t0 = tracker.start();
        ZeroPowerBehavior v = device.getZeroPowerBehavior();
        tracker.read(HwOp.GET_ZERO_POWER_BEHAVIOR, t0);
        return v;
    }

    @Deprecated
    @Override
    @SuppressWarnings("deprecation")
    public void setPowerFloat() {

        long t0 = tracker.start();
        device.setPowerFloat();
        tracker.write(HwOp.SET_POWER_FLOAT, t0, false, powerSlot, busySlot);
        lastPower = Double.NaN;
        commandedPower = 0;
        commandedPowerNs = t0;
    }

    @Override
    public boolean getPowerFloat() {

        long t0 = tracker.start();
        boolean v = device.getPowerFloat();
        tracker.read(HwOp.GET_POWER_FLOAT, t0);
        return v;
    }

    @Override
    public void setTargetPosition(int position) {

        long t0 = tracker.start();
        device.setTargetPosition(position);
        tracker.write(HwOp.SET_TARGET_POSITION, t0, hasTargetPosition && position == lastTargetPosition, busySlot);
        lastTargetPosition = position;
        hasTargetPosition = true;
    }

    @Override
    public int getTargetPosition() {

        long t0 = tracker.start();
        int v = device.getTargetPosition();
        tracker.read(HwOp.GET_TARGET_POSITION, t0);
        return v;
    }

    @Override
    public boolean isBusy() {

        if (tracker.cacheHit(HwOp.IS_BUSY, busySlot, CacheSlot.NO_KEY)) return busySlot.b;
        long t0 = tracker.start();
        boolean v = device.isBusy();
        busySlot.b = v;
        tracker.read(HwOp.IS_BUSY, t0, busySlot, CacheSlot.NO_KEY);
        return v;
    }

    @Override
    public int getCurrentPosition() {

        if (tracker.cacheHit(HwOp.GET_CURRENT_POSITION, positionSlot, CacheSlot.NO_KEY)) {
            return positionSlot.i;
        }
        long t0 = tracker.start();
        int v = device.getCurrentPosition();
        positionSlot.i = v;
        tracker.read(HwOp.GET_CURRENT_POSITION, t0, positionSlot, CacheSlot.NO_KEY);
        return v;
    }

    @Override
    public void setMode(RunMode mode) {

        long t0 = tracker.start();
        device.setMode(mode);
        boolean redundant = mode == lastMode && mode != RunMode.STOP_AND_RESET_ENCODER;
        tracker.write(HwOp.SET_MODE, t0, redundant);
        lastMode = mode;
    }

    @Override
    public RunMode getMode() {

        long t0 = tracker.start();
        RunMode v = device.getMode();
        tracker.read(HwOp.GET_MODE, t0);
        return v;
    }

    protected void onVelocityCommand(long t0) {

        lastPower = Double.NaN;
        commandedPower = Double.NaN;
        commandedPowerNs = t0;
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public long getCommandedPowerNs() {
        return commandedPowerNs;
    }

    public double getObservedCurrentAmps() {
        return observedCurrentAmps;
    }

    public long getObservedCurrentNs() {
        return observedCurrentNs;
    }

    public double getObservedVelocity() {
        return observedVelocity;
    }

    public long getObservedVelocityNs() {
        return observedVelocityNs;
    }

    public boolean supportsElectricalSampling() {
        return device instanceof DcMotorEx;
    }

    public double sampleCurrentAmps() {

        if (!(device instanceof DcMotorEx)) return Double.NaN;
        long t0 = System.nanoTime();
        double amps = ((DcMotorEx) device).getCurrent(CurrentUnit.AMPS);
        long t1 = System.nanoTime();
        tracker.getStats().recordMonitorRead(t1 - t0);
        observedCurrentAmps = amps;
        observedCurrentNs = t1;
        return amps;
    }

    public double sampleVelocity() {

        if (!(device instanceof DcMotorEx)) return Double.NaN;
        long t0 = System.nanoTime();
        double v = ((DcMotorEx) device).getVelocity();
        long t1 = System.nanoTime();
        tracker.getStats().recordMonitorRead(t1 - t0);
        observedVelocity = v;
        observedVelocityNs = t1;
        return v;
    }

    public MotorConfigurationType peekMotorType() {
        return device.getMotorType();
    }

    public void setWindingResistance(double ohms) {
        windingResistanceOhms = ohms;
    }

    public double getWindingResistance() {
        return windingResistanceOhms;
    }
}
