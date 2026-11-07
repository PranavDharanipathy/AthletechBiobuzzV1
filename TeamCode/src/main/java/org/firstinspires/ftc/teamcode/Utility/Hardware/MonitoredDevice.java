package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

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

    protected static <D> D lookup(HardwareMap hardwareMap, Class<D> cls, String name) {

        if (hardwareMap == null) throw new IllegalArgumentException("hardwareMap is null");
        HardwareMonitor.get().attachHardwareMap(hardwareMap);
        return hardwareMap.get(cls, name);
    }

    public T getDelegate() {
        return device;
    }

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

    public void setCachingReads(boolean cacheReads) {
        tracker.setCachingReads(cacheReads);
    }

    public void invalidateCache() {
        tracker.invalidateCache();
    }

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
