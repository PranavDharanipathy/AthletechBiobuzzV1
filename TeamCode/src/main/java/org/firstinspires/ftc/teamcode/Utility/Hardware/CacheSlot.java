package org.firstinspires.ftc.teamcode.Utility.Hardware;

final class CacheSlot {
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
