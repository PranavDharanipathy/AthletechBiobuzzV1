package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.util.ArrayList;

public final class DeviceTracker {

    private final String name;
    private final String type;
    private final String deviceClass;
    private final String connectionInfo;
    private final DeviceStats stats;
    private final ArrayList<CacheSlot> slots = new ArrayList<>(4);
    private volatile boolean cacheReads;
    private volatile int monitorId = -1;

    public DeviceTracker(String name, String type, HardwareDevice device, boolean cacheReads) {

        this.name = name;
        this.type = type;
        this.deviceClass = device == null ? "?" : device.getClass().getSimpleName();
        this.connectionInfo = safeConnectionInfo(device);
        this.stats = new DeviceStats(name, type);
        this.cacheReads = cacheReads;
        HardwareMonitor.get().register(this);
    }

    public String getName() {
        return name;
    }

    public String getType() {
        return type;
    }

    public String getDeviceClass() {
        return deviceClass;
    }

    public String getConnectionInfo() {
        return connectionInfo;
    }

    public DeviceStats getStats() {
        return stats;
    }

    public boolean isCachingReads() {
        return cacheReads;
    }

    public void setCachingReads(boolean cacheReads) {

        this.cacheReads = cacheReads;
        if (!cacheReads) invalidateCache();
    }

    public void invalidateCache() {

        synchronized (slots) {
            for (int k = 0; k < slots.size(); k++) slots.get(k).invalidate();
        }
    }

    public int getMonitorId() {
        return monitorId;
    }

    public void assignMonitorId(int id) {
        this.monitorId = id;
    }

    CacheSlot newSlot() {

        CacheSlot slot = new CacheSlot();
        synchronized (slots) {
            slots.add(slot);
        }
        return slot;
    }

    boolean cacheHit(HwOp op, CacheSlot slot, long key) {

        if (!cacheReads) return false;
        if (slot.cycle != HardwareMonitor.currentCycle() || slot.key != key) return false;
        long now = System.nanoTime();
        if (now - slot.timeNs > HardwareMonitor.cacheMaxAgeNs()) return false;
        stats.recordRead(op, now, now, true);
        HardwareMonitor.get().onIo(this, op, now, now, true);
        return true;
    }

    long start() {
        return System.nanoTime();
    }

    void read(HwOp op, long t0) {

        long t1 = System.nanoTime();
        stats.recordRead(op, t0, t1, false);
        HardwareMonitor.get().onIo(this, op, t0, t1, false);
    }

    void read(HwOp op, long t0, CacheSlot slot, long key) {

        long t1 = System.nanoTime();
        if (cacheReads) {
            slot.key = key;
            slot.timeNs = t1;
            slot.cycle = HardwareMonitor.currentCycle();
        }
        stats.recordRead(op, t0, t1, false);
        HardwareMonitor.get().onIo(this, op, t0, t1, false);
    }

    void write(HwOp op, long t0, boolean redundant) {

        long t1 = System.nanoTime();
        invalidateCache();
        record(op, t0, t1, redundant);
    }

    void writeKeepCache(HwOp op, long t0, boolean redundant) {
        record(op, t0, System.nanoTime(), redundant);
    }

    void write(HwOp op, long t0, boolean redundant, CacheSlot affected) {

        long t1 = System.nanoTime();
        affected.invalidate();
        record(op, t0, t1, redundant);
    }

    void write(HwOp op, long t0, boolean redundant, CacheSlot a, CacheSlot b) {

        long t1 = System.nanoTime();
        a.invalidate();
        b.invalidate();
        record(op, t0, t1, redundant);
    }

    private void record(HwOp op, long t0, long t1, boolean redundant) {

        stats.recordWrite(op, t0, t1, redundant);
        HardwareMonitor.get().onIo(this, op, t0, t1, redundant);
    }

    private static String safeConnectionInfo(HardwareDevice device) {

        if (device == null) return "";
        try {
            String info = device.getConnectionInfo();
            return info == null ? "" : info;
        } catch (RuntimeException e) {
            return "";
        }
    }

    @Override
    public String toString() {
        return type + " '" + name + "'";
    }
}
