package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Drop-in {@link DcMotorEx} that counts and times every call and can cache reads. Also usable
 * anywhere a {@code DcMotor} is expected.
 *
 * <pre>
 * DcMotorEx lift = new MonitoredDcMotorEx(hardwareMap, "lift", true);  // true = cache reads
 * </pre>
 *
 * Cached values (when caching is on): position, power, busy, velocity, current, over-current.
 * Current is cached in amps, so one hardware read serves both {@code AMPS} and
 * {@code MILLIAMPS}. Every value you read here is also handed to the
 * {@code HardwareMonitor}, so reading current yourself means the monitor never has to.
 */
public class MonitoredDcMotorEx extends MonitoredDcMotor implements DcMotorEx {

    private final DcMotorEx ex;
    private final CacheSlot velocitySlot;
    private final CacheSlot velocityUnitSlot;
    private final CacheSlot currentSlot;
    private final CacheSlot overCurrentSlot;

    private double lastVelocity = Double.NaN;
    private AngleUnit lastVelocityUnit;
    private int lastTolerance = Integer.MIN_VALUE;

    public MonitoredDcMotorEx(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookup(hardwareMap, DcMotorEx.class, name), name, cacheReads);
    }

    /** Same as {@code new MonitoredDcMotorEx(hardwareMap, name, false)} (no read caching). */
    public MonitoredDcMotorEx(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    public MonitoredDcMotorEx(DcMotorEx motor, String name, boolean cacheReads) {
        super(motor, name, "DcMotorEx", cacheReads);
        this.ex = motor;
        velocitySlot = tracker.newSlot();
        velocityUnitSlot = tracker.newSlot();
        currentSlot = tracker.newSlot();
        overCurrentSlot = tracker.newSlot();
    }

    @Override
    public DcMotorEx getDelegate() {
        return ex;
    }

    // ------------------------------------------------------------------ enable / disable

    @Override
    public void setMotorEnable() {
        long t0 = tracker.start();
        ex.setMotorEnable();
        tracker.write(HwOp.SET_MOTOR_ENABLE, t0, false, powerSlot, busySlot);
    }

    @Override
    public void setMotorDisable() {
        long t0 = tracker.start();
        ex.setMotorDisable();
        tracker.write(HwOp.SET_MOTOR_DISABLE, t0, false, powerSlot, busySlot);
    }

    @Override
    public boolean isMotorEnabled() {
        long t0 = tracker.start();
        boolean v = ex.isMotorEnabled();
        tracker.read(HwOp.IS_MOTOR_ENABLED, t0);
        return v;
    }

    // ------------------------------------------------------------------ velocity

    @Override
    public void setPower(double power) {
        super.setPower(power);
        // back in power mode: the next setVelocity is a real change even if the value matches
        lastVelocity = Double.NaN;
        lastVelocityUnit = null;
    }

    @Override
    public void setVelocity(double angularRate) {
        long t0 = tracker.start();
        ex.setVelocity(angularRate);
        boolean redundant = angularRate == lastVelocity && lastVelocityUnit == null;
        tracker.write(HwOp.SET_VELOCITY, t0, redundant, powerSlot, busySlot);
        lastVelocity = angularRate;
        lastVelocityUnit = null;
        onVelocityCommand(t0);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        long t0 = tracker.start();
        ex.setVelocity(angularRate, unit);
        boolean redundant = angularRate == lastVelocity && unit == lastVelocityUnit;
        tracker.write(HwOp.SET_VELOCITY, t0, redundant, powerSlot, busySlot);
        lastVelocity = angularRate;
        lastVelocityUnit = unit;
        onVelocityCommand(t0);
    }

    /** Velocity in encoder ticks per second. */
    @Override
    public double getVelocity() {
        if (tracker.cacheHit(HwOp.GET_VELOCITY, velocitySlot, CacheSlot.NO_KEY)) {
            return velocitySlot.d;
        }
        long t0 = tracker.start();
        double v = ex.getVelocity();
        velocitySlot.d = v;
        tracker.read(HwOp.GET_VELOCITY, t0, velocitySlot, CacheSlot.NO_KEY);
        observedVelocity = v;
        observedVelocityNs = System.nanoTime();
        return v;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        long key = unit == null ? CacheSlot.NO_KEY : unit.ordinal();
        if (tracker.cacheHit(HwOp.GET_VELOCITY, velocityUnitSlot, key)) return velocityUnitSlot.d;
        long t0 = tracker.start();
        double v = ex.getVelocity(unit);
        velocityUnitSlot.d = v;
        tracker.read(HwOp.GET_VELOCITY, t0, velocityUnitSlot, key);
        return v;
    }

    // ------------------------------------------------------------------ PIDF

    @Deprecated
    @Override
    @SuppressWarnings("deprecation")
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        long t0 = tracker.start();
        ex.setPIDCoefficients(mode, pidCoefficients);
        tracker.writeKeepCache(HwOp.SET_PIDF, t0, false);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) {
        long t0 = tracker.start();
        ex.setPIDFCoefficients(mode, pidfCoefficients);
        tracker.writeKeepCache(HwOp.SET_PIDF, t0, false);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        long t0 = tracker.start();
        ex.setVelocityPIDFCoefficients(p, i, d, f);
        tracker.writeKeepCache(HwOp.SET_PIDF, t0, false);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        long t0 = tracker.start();
        ex.setPositionPIDFCoefficients(p);
        tracker.writeKeepCache(HwOp.SET_PIDF, t0, false);
    }

    @Deprecated
    @Override
    @SuppressWarnings("deprecation")
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        long t0 = tracker.start();
        PIDCoefficients v = ex.getPIDCoefficients(mode);
        tracker.read(HwOp.GET_PIDF, t0);
        return v;
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        long t0 = tracker.start();
        PIDFCoefficients v = ex.getPIDFCoefficients(mode);
        tracker.read(HwOp.GET_PIDF, t0);
        return v;
    }

    // ------------------------------------------------------------------ tolerance

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        long t0 = tracker.start();
        ex.setTargetPositionTolerance(tolerance);
        tracker.write(HwOp.SET_TARGET_POSITION_TOLERANCE, t0, tolerance == lastTolerance, busySlot);
        lastTolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        long t0 = tracker.start();
        int v = ex.getTargetPositionTolerance();
        tracker.read(HwOp.GET_TARGET_POSITION_TOLERANCE, t0);
        return v;
    }

    // ------------------------------------------------------------------ current

    @Override
    public double getCurrent(CurrentUnit unit) {
        if (tracker.cacheHit(HwOp.GET_CURRENT, currentSlot, CacheSlot.NO_KEY)) {
            return fromAmps(currentSlot.d, unit);
        }
        long t0 = tracker.start();
        double amps = ex.getCurrent(CurrentUnit.AMPS);
        currentSlot.d = amps;
        tracker.read(HwOp.GET_CURRENT, t0, currentSlot, CacheSlot.NO_KEY);
        observedCurrentAmps = amps;
        observedCurrentNs = System.nanoTime();
        return fromAmps(amps, unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        long t0 = tracker.start();
        double v = ex.getCurrentAlert(unit);
        tracker.read(HwOp.GET_CURRENT_ALERT, t0);
        return v;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        long t0 = tracker.start();
        ex.setCurrentAlert(current, unit);
        tracker.write(HwOp.SET_CURRENT_ALERT, t0, false, overCurrentSlot);
    }

    @Override
    public boolean isOverCurrent() {
        if (tracker.cacheHit(HwOp.IS_OVER_CURRENT, overCurrentSlot, CacheSlot.NO_KEY)) {
            return overCurrentSlot.b;
        }
        long t0 = tracker.start();
        boolean v = ex.isOverCurrent();
        overCurrentSlot.b = v;
        tracker.read(HwOp.IS_OVER_CURRENT, t0, overCurrentSlot, CacheSlot.NO_KEY);
        return v;
    }

    private static double fromAmps(double amps, CurrentUnit unit) {
        return unit == CurrentUnit.MILLIAMPS ? amps * 1000.0 : amps;
    }
}
