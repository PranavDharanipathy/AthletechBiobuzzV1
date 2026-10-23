package org.firstinspires.ftc.teamcode.Utility.Hardware;

/**
 * One cached read result. Holds primitives directly so cache hits never allocate.
 *
 * <p>A slot is valid while it was filled during the current {@code HardwareMonitor} cycle, its
 * {@link #key} matches the request (e.g. the unit of {@code getDistance(unit)}), and it is
 * younger than the configured max age. Any write to the device invalidates all its slots.
 */
final class CacheSlot {
    /** Key for reads that take no arguments. */
    static final long NO_KEY = -1;

    long cycle = Long.MIN_VALUE;
    long timeNs;
    long key = NO_KEY;

    double d;
    int i;
    boolean b;
    float f;
    Object o;

    void invalidate() {
        cycle = Long.MIN_VALUE;
    }
}
