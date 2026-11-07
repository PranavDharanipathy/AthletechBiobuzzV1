package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;
import org.firstinspires.ftc.teamcode.Utility.TemporalLog.TemporalLog;

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
    //temporal log: the value of the read/write that's about to be recorded (wrappers set it with v()), and the log's key per op
    private double pending = Double.NaN;
    private Class<?> pendingEnum;
    public int[] temporalKeys;
    public int temporalSession = -1;

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

    public void v(double x) {
        pending = x;
    }

    public void v(float x) {
        pending = x;
    }

    public void v(int x) {
        pending = x;
    }

    public void v(long x) {
        pending = x;
    }

    public void v(boolean x) {
        pending = x ? 1 : 0;
    }

    public void v(Enum<?> e) {

        pending = e == null ? Double.NaN : e.ordinal();
        pendingEnum = e == null ? null : e.getDeclaringClass();
    }

    public void v(Object o) {
        pending = Double.NaN;
    }

    /// HardwareMonitor's own sampling reads (current, velocity), which don't go through read()
    public void monitorRead(HwOp op, long t0, long t1, double value) {
        if (TemporalLog.active) TemporalLog.io(this, op, t0, t1, false, value, null);
    }

    private void logIo(HwOp op, long t0, long t1, boolean flag) {

        if (TemporalLog.active) TemporalLog.io(this, op, t0, t1, flag, pending, pendingEnum);
        pending = Double.NaN;
        pendingEnum = null;
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
        if (TemporalLog.active) TemporalLog.io(this, op, now, now, true, Double.NaN, null);
        return true;
    }

    long start() {
        return System.nanoTime();
    }

    void read(HwOp op, long t0) {

        long t1 = System.nanoTime();
        stats.recordRead(op, t0, t1, false);
        HardwareMonitor.get().onIo(this, op, t0, t1, false);
        logIo(op, t0, t1, false);
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
        logIo(op, t0, t1, false);
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
        logIo(op, t0, t1, redundant);
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
