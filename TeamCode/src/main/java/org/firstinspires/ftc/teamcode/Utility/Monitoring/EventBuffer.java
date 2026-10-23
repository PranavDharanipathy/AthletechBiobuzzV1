package org.firstinspires.ftc.teamcode.Utility.Monitoring;

/**
 * Double-buffered queue of hardware I/O events, stored as primitive arrays so logging an event
 * from the OpMode thread never allocates. The writer thread periodically swaps in an empty
 * buffer and writes the full one to disk. If the writer falls behind and the buffer fills,
 * new events are dropped and counted rather than blocking the robot loop.
 */
final class EventBuffer {

    /** One batch of events, owned by the writer thread between swaps. */
    static final class Batch {
        long[] timeNs;
        int[] deviceId;
        short[] op;
        int[] latencyNs;
        boolean[] flag;
        int size;

        Batch(int capacity) {
            timeNs = new long[capacity];
            deviceId = new int[capacity];
            op = new short[capacity];
            latencyNs = new int[capacity];
            flag = new boolean[capacity];
        }
    }

    private final int capacity;
    private Batch active;
    private long dropped;
    private long accepted;

    EventBuffer(int capacity) {
        this.capacity = capacity;
        this.active = new Batch(capacity);
    }

    int capacity() {
        return capacity;
    }

    /** @param timeNs nanoseconds since the session started */
    synchronized void add(long timeNs, int deviceId, int op, int latencyNs, boolean flag) {
        Batch b = active;
        if (b.size >= capacity) {
            dropped++;
            return;
        }
        int i = b.size++;
        b.timeNs[i] = timeNs;
        b.deviceId[i] = deviceId;
        b.op[i] = (short) op;
        b.latencyNs[i] = latencyNs;
        b.flag[i] = flag;
        accepted++;
    }

    /**
     * Hands the filled batch to the caller and takes {@code empty} (whose size is reset) as the
     * new active buffer. Returns the filled batch.
     */
    synchronized Batch swap(Batch empty) {
        Batch full = active;
        empty.size = 0;
        active = empty;
        return full;
    }

    synchronized long dropped() {
        return dropped;
    }

    synchronized long accepted() {
        return accepted;
    }
}
