package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

/**
 * Base class for the drop-in {@code Monitored*} wrappers.
 *
 * <p>A wrapper implements the same SDK interface as the device it wraps, so it can be stored in
 * a {@code DcMotorEx}, {@code Servo}, {@code IMU}, ... variable and passed anywhere the plain
 * device is accepted. Every call is forwarded to the real device and recorded in
 * {@link #getStats()}; reads can optionally be served from a per-loop cache.
 *
 * @param <T> the SDK interface being wrapped
 */
public abstract class MonitoredDevice<T extends HardwareDevice> implements HardwareDevice {

    protected final T device;
    protected final DeviceTracker tracker;

    protected MonitoredDevice(T device, String name, String type, boolean cacheReads) {
        if (device == null) {
            throw new IllegalArgumentException(type + " '" + name + "' is null");
        }
        this.device = device;
        this.tracker = new DeviceTracker(name, type, device, cacheReads);
    }

    /** Looks a device up and lets the monitor see the hardware map (hubs, battery sensor). */
    protected static <D> D lookup(HardwareMap hardwareMap, Class<D> cls, String name) {
        if (hardwareMap == null) throw new IllegalArgumentException("hardwareMap is null");
        HardwareMonitor.get().attachHardwareMap(hardwareMap);
        return hardwareMap.get(cls, name);
    }

    // ------------------------------------------------------------------ monitoring API

    /** The real SDK device. Calls made on it directly are not counted. */
    public T getDelegate() {
        return device;
    }

    /** The name from the robot configuration. */
    public String getName() {
        return tracker.getName();
    }

    public DeviceStats getStats() {
        return tracker.getStats();
    }

    public DeviceTracker getTracker() {
        return tracker;
    }

    public boolean isCachingReads() {
        return tracker.isCachingReads();
    }

    /** Turns read caching on or off at runtime (the constructor sets the initial value). */
    public void setCachingReads(boolean cacheReads) {
        tracker.setCachingReads(cacheReads);
    }

    /** Forces the next read of every value to go to the hardware. */
    public void invalidateCache() {
        tracker.invalidateCache();
    }

    // ------------------------------------------------------------------ HardwareDevice
    // Metadata calls are forwarded without being counted: they never touch the bus.

    @Override
    public Manufacturer getManufacturer() {
        return device.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return device.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return device.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return device.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        device.resetDeviceConfigurationForOpMode();
        tracker.invalidateCache();
    }

    @Override
    public void close() {
        device.close();
    }

    @Override
    public String toString() {
        return tracker.toString();
    }
}
