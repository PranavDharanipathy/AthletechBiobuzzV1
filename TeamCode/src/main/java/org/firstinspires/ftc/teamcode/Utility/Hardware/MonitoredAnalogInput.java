package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

public class MonitoredAnalogInput extends AnalogInput {

    private final AnalogInput device;
    private final DeviceTracker tracker;
    private final CacheSlot voltageSlot;

    public MonitoredAnalogInput(HardwareMap hardwareMap, String name, boolean cacheReads) {
        this(lookupAnalog(hardwareMap, name), name, cacheReads);
    }

    public MonitoredAnalogInput(HardwareMap hardwareMap, String name) {
        this(hardwareMap, name, false);
    }

    public MonitoredAnalogInput(AnalogInput input, String name, boolean cacheReads) {

        super(null, -1);
        if (input == null) throw new IllegalArgumentException("AnalogInput '" + name + "' is null");
        this.device = input;
        this.tracker = new DeviceTracker(name, "AnalogInput", input, cacheReads);
        this.voltageSlot = tracker.newSlot();
    }

    private static AnalogInput lookupAnalog(HardwareMap hardwareMap, String name) {

        if (hardwareMap == null) throw new IllegalArgumentException("hardwareMap is null");
        HardwareMonitor.get().attachHardwareMap(hardwareMap);
        return hardwareMap.get(AnalogInput.class, name);
    }

    public AnalogInput getDelegate() {
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
    public double getVoltage() {

        if (tracker.cacheHit(HwOp.GET_VOLTAGE, voltageSlot, CacheSlot.NO_KEY)) return voltageSlot.d;
        long t0 = tracker.start();
        double v = device.getVoltage();
        voltageSlot.d = v;
        tracker.read(HwOp.GET_VOLTAGE, t0, voltageSlot, CacheSlot.NO_KEY);
        return v;
    }

    @Override
    public double getMaxVoltage() {

        long t0 = tracker.start();
        double v = device.getMaxVoltage();
        tracker.read(HwOp.GET_MAX_VOLTAGE, t0);
        return v;
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
