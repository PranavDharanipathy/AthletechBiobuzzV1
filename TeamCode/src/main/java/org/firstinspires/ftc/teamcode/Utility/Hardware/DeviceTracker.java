package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.util.ArrayList;

/**
 * Instrumentation shared by every {@code Monitored*} wrapper: times each hardware call,
 * serves cached reads, records statistics and forwards events to the {@link HardwareMonitor}.
 *
 * <p>The pattern every wrapper method follows is:
 * <pre>
 * if (tracker.cacheHit(op, slot, key)) return slot.value;   // cached reads only
 * long t0 = tracker.start();
 * value = delegate.read();
 * slot.value = value;
 * tracker.read(op, t0, slot, key);
 * </pre>
 * No allocation happens on that path.
 */
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

    // ------------------------------------------------------------------ public info

    /** The name the device has in the robot configuration. */
    public String getName() {
        return name;
    }

    /** Wrapper type, e.g. {@code DcMotorEx}. */
    public String getType() {
        return type;
    }

    /** Class of the real SDK device being wrapped, e.g. {@code DcMotorImplEx}. */
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

    /** Forces the next read of every value to go to the hardware. */
    public void invalidateCache() {
        synchronized (slots) {
            for (int k = 0; k < slots.size(); k++) slots.get(k).invalidate();
        }
    }

    /** Id used in the monitor's event log. Assigned by {@link HardwareMonitor}. */
    public int getMonitorId() {
        return monitorId;
    }

    /** Internal: called by {@link HardwareMonitor} when the device joins a session. */
    public void assignMonitorId(int id) {
        this.monitorId = id;
    }

    // ------------------------------------------------------------------ wrapper helpers

    CacheSlot newSlot() {
        CacheSlot slot = new CacheSlot();
        synchronized (slots) {
            slots.add(slot);
        }
        return slot;
    }

    /**
     * Returns true (and records a cache hit) if {@code slot} holds a value for {@code key} that
     * is still fresh. The caller then returns the value stored in the slot.
     */
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

    /** Records an uncached read that started at {@code t0}. */
    void read(HwOp op, long t0) {
        long t1 = System.nanoTime();
        stats.recordRead(op, t0, t1, false);
        HardwareMonitor.get().onIo(this, op, t0, t1, false);
    }

    /** Records a read and, if caching is on, marks {@code slot} (already filled) as fresh. */
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

    /**
     * Records a write and invalidates <b>every</b> cached read of this device - the safe default
     * for writes that can change anything (direction, run mode, motor type, ...).
     */
    void write(HwOp op, long t0, boolean redundant) {
        long t1 = System.nanoTime();
        invalidateCache();
        record(op, t0, t1, redundant);
    }

    /** Records a write that cannot change any cached reading (e.g. PIDF coefficients). */
    void writeKeepCache(HwOp op, long t0, boolean redundant) {
        record(op, t0, System.nanoTime(), redundant);
    }

    /** Records a write that can only change the reading held in {@code affected}. */
    void write(HwOp op, long t0, boolean redundant, CacheSlot affected) {
        long t1 = System.nanoTime();
        affected.invalidate();
        record(op, t0, t1, redundant);
    }

    /** Records a write that can only change the readings held in {@code a} and {@code b}. */
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
