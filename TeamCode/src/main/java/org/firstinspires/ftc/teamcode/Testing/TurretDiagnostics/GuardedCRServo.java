package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * The only path from any command source to a turret servo during diagnostics.
 *
 * <p>Both the baseline {@code TurretBase} (through its device-injection constructor) and the
 * open-loop runner write through this class. Until {@link #allow} it writes neutral (0) and
 * nothing else. After {@link #allow} it clamps every request to {@code [-limit, limit]}, and turns non-finite requests
 * into 0. It records both the requested and the applied value.
 *
 * <p>Direction passes straight through. {@code TurretBase} sets the configured
 * {@code TurretConstants} direction; the diagnostic never changes it.
 */
public final class GuardedCRServo implements CRServo {

    private final CRServo delegate;
    private boolean open;
    private double limit;
    private double requested;
    private double applied;
    private boolean saturated;
    private boolean everWritten;

    public GuardedCRServo(CRServo delegate) {
        if (delegate == null) throw new NullPointerException("delegate");
        this.delegate = delegate;
    }

    /** Allows commands up to {@code |limit|} until {@link #stop()}. */
    public void allow(double commandLimit) {
        if (!(commandLimit > 0) || commandLimit > 1) throw new IllegalArgumentException("limit " + commandLimit);
        limit = commandLimit;
        open = true;
    }

    /** Ends command authority and writes neutral immediately. Safe to call repeatedly. */
    public void stop() {
        open = false;
        requested = 0;
        writeApplied(0);
    }

    public boolean isOpen() { return open; }
    public double requested() { return requested; }
    public double applied() { return applied; }
    public boolean saturated() { return saturated; }
    public boolean everWritten() { return everWritten; }

    @Override
    public void setPower(double power) {
        requested = power;
        if (!open || Double.isNaN(power) || Double.isInfinite(power)) {
            saturated = false;
            writeApplied(0);
            return;
        }
        double clamped = Math.max(-limit, Math.min(limit, power));
        saturated = clamped != power;
        writeApplied(clamped);
    }

    private void writeApplied(double value) {
        applied = value;
        everWritten = true;
        delegate.setPower(value);
    }

    @Override
    public double getPower() {
        return applied;
    }

    @Override
    public void setDirection(Direction direction) {
        delegate.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return delegate.getDirection();
    }

    @Override
    public ServoController getController() {
        return delegate.getController();
    }

    @Override
    public int getPortNumber() {
        return delegate.getPortNumber();
    }

    @Override
    public Manufacturer getManufacturer() {
        return delegate.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return "Guarded " + delegate.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return delegate.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return delegate.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        delegate.resetDeviceConfigurationForOpMode();
    }

    /** HardwareDevice close: stops this guard. The wrapped device belongs to the SDK and stays open. */
    @Override
    public void close() {
        stop();
    }
}
