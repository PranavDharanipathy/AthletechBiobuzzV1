package org.firstinspires.ftc.teamcode.Utility.Hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Thread-safe I/O statistics for one hardware device.
 *
 * <p>Every read and write made through a {@code Monitored*} wrapper lands here: how many,
 * how often (recent and lifetime frequency), how long the call blocked (latency), when it
 * happened ({@link System#nanoTime()} timestamps), how many reads were served from the read
 * cache, and how many writes re-sent a value the device already had.
 *
 * <p>Use {@link #snapshot()} for a consistent, immutable copy you can print or log.
 */
public final class DeviceStats {

    /** How many of the most recent events are kept in memory with their timestamps. */
    public static final int RECENT_EVENT_CAPACITY = 256;
    /** Window used for "recent" frequencies. */
    public static final long RECENT_WINDOW_NS = 1_000_000_000L;

    private static final int RATE_RING_CAPACITY = 1024;

    private final String name;
    private final String type;
    private final long createdNs = System.nanoTime();
    private final long createdWallMs = System.currentTimeMillis();

    private final OpCounter[] ops = new OpCounter[HwOp.count()];

    private long reads;
    private long writes;
    private long cacheHits;
    private long redundantWrites;
    private long readIoNs;
    private long writeIoNs;
    private long maxReadNs;
    private long maxWriteNs;
    private long firstReadNs = -1;
    private long lastReadNs = -1;
    private long firstWriteNs = -1;
    private long lastWriteNs = -1;
    private long monitorReads;
    private long monitorReadNs;

    private final TimeRing readTimes = new TimeRing(RATE_RING_CAPACITY);
    private final TimeRing writeTimes = new TimeRing(RATE_RING_CAPACITY);

    private final long[] evTime = new long[RECENT_EVENT_CAPACITY];
    private final int[] evLatencyNs = new int[RECENT_EVENT_CAPACITY];
    private final short[] evOp = new short[RECENT_EVENT_CAPACITY];
    private final boolean[] evFlag = new boolean[RECENT_EVENT_CAPACITY];
    private int evHead;
    private int evSize;

    public DeviceStats(String name, String type) {
        this.name = name;
        this.type = type;
    }

    public String getName() {
        return name;
    }

    public String getType() {
        return type;
    }

    public long getCreatedNs() {
        return createdNs;
    }

    public long getCreatedWallMs() {
        return createdWallMs;
    }

    // ------------------------------------------------------------------ recording

    /**
     * Records a read. {@code cached} reads were answered from the wrapper's read cache and did
     * not touch the hardware, so they add no I/O time.
     */
    synchronized void recordRead(HwOp op, long startNs, long endNs, boolean cached) {
        long latency = cached ? 0 : Math.max(0, endNs - startNs);
        reads++;
        if (cached) {
            cacheHits++;
        } else {
            readIoNs += latency;
            if (latency > maxReadNs) maxReadNs = latency;
        }
        if (firstReadNs < 0) firstReadNs = startNs;
        lastReadNs = startNs;
        readTimes.add(startNs);
        counter(op).add(startNs, latency, cached);
        pushEvent(startNs, op, latency, cached);
    }

    /**
     * Records a write. {@code redundant} writes sent the same value as the previous write of
     * the same kind - a sign that write caching would save bus time.
     */
    synchronized void recordWrite(HwOp op, long startNs, long endNs, boolean redundant) {
        long latency = Math.max(0, endNs - startNs);
        writes++;
        writeIoNs += latency;
        if (latency > maxWriteNs) maxWriteNs = latency;
        if (redundant) redundantWrites++;
        if (firstWriteNs < 0) firstWriteNs = startNs;
        lastWriteNs = startNs;
        writeTimes.add(startNs);
        counter(op).add(startNs, latency, redundant);
        pushEvent(startNs, op, latency, redundant);
    }

    /** Records a read the HardwareMonitor made for its own sampling (not your code's read). */
    public synchronized void recordMonitorRead(long latencyNs) {
        monitorReads++;
        monitorReadNs += Math.max(0, latencyNs);
    }

    /** Zeroes every counter (the device itself is untouched). */
    public synchronized void reset() {
        for (int i = 0; i < ops.length; i++) ops[i] = null;
        reads = writes = cacheHits = redundantWrites = 0;
        readIoNs = writeIoNs = maxReadNs = maxWriteNs = 0;
        firstReadNs = lastReadNs = firstWriteNs = lastWriteNs = -1;
        monitorReads = monitorReadNs = 0;
        readTimes.clear();
        writeTimes.clear();
        evHead = evSize = 0;
    }

    // ------------------------------------------------------------------ quick getters

    public synchronized long getReads() {
        return reads;
    }

    public synchronized long getWrites() {
        return writes;
    }

    public synchronized long getCacheHits() {
        return cacheHits;
    }

    /** Reads that actually went to the hardware (total reads minus cache hits). */
    public synchronized long getHardwareReads() {
        return reads - cacheHits;
    }

    public synchronized long getRedundantWrites() {
        return redundantWrites;
    }

    /** Reads per second over the last second. */
    public synchronized double getRecentReadHz() {
        return readTimes.rateHz(System.nanoTime(), RECENT_WINDOW_NS);
    }

    /** Writes per second over the last second. */
    public synchronized double getRecentWriteHz() {
        return writeTimes.rateHz(System.nanoTime(), RECENT_WINDOW_NS);
    }

    /** Total time spent blocked in this device's hardware calls, in milliseconds. */
    public synchronized double getTotalIoMs() {
        return (readIoNs + writeIoNs) / 1e6;
    }

    public synchronized Snapshot snapshot() {
        long now = System.nanoTime();
        List<OpSnapshot> opList = new ArrayList<>();
        for (int i = 0; i < ops.length; i++) {
            OpCounter c = ops[i];
            if (c != null && c.count > 0) opList.add(c.snapshot(HwOp.fromOrdinal(i)));
        }
        Collections.sort(opList, (a, b) -> Long.compare(b.count, a.count));

        int n = evSize;
        long[] t = new long[n];
        int[] lat = new int[n];
        HwOp[] op = new HwOp[n];
        boolean[] flag = new boolean[n];
        int start = (evHead - n + RECENT_EVENT_CAPACITY) % RECENT_EVENT_CAPACITY;
        for (int k = 0; k < n; k++) {
            int idx = (start + k) % RECENT_EVENT_CAPACITY;
            t[k] = evTime[idx];
            lat[k] = evLatencyNs[idx];
            op[k] = HwOp.fromOrdinal(evOp[idx]);
            flag[k] = evFlag[idx];
        }

        long hwReads = reads - cacheHits;
        return new Snapshot(
                name, type, now, createdNs, createdWallMs,
                reads, writes, cacheHits, hwReads, redundantWrites,
                readTimes.rateHz(now, RECENT_WINDOW_NS),
                writeTimes.rateHz(now, RECENT_WINDOW_NS),
                spanHz(reads, firstReadNs, lastReadNs),
                spanHz(writes, firstWriteNs, lastWriteNs),
                hwReads > 0 ? readIoNs / 1e3 / hwReads : 0,
                maxReadNs / 1e3,
                writes > 0 ? writeIoNs / 1e3 / writes : 0,
                maxWriteNs / 1e3,
                (readIoNs + writeIoNs) / 1e6,
                firstReadNs, lastReadNs, firstWriteNs, lastWriteNs,
                monitorReads, monitorReadNs / 1e6,
                Collections.unmodifiableList(opList),
                t, lat, op, flag);
    }

    // ------------------------------------------------------------------ internals

    private OpCounter counter(HwOp op) {
        OpCounter c = ops[op.ordinal()];
        if (c == null) {
            c = new OpCounter();
            ops[op.ordinal()] = c;
        }
        return c;
    }

    private void pushEvent(long t, HwOp op, long latencyNs, boolean flag) {
        evTime[evHead] = t;
        evLatencyNs[evHead] = (int) Math.min(Integer.MAX_VALUE, latencyNs);
        evOp[evHead] = (short) op.ordinal();
        evFlag[evHead] = flag;
        evHead = (evHead + 1) % RECENT_EVENT_CAPACITY;
        if (evSize < RECENT_EVENT_CAPACITY) evSize++;
    }

    /** Average frequency across the span between the first and last event. */
    static double spanHz(long count, long firstNs, long lastNs) {
        if (count < 2 || lastNs <= firstNs) return 0;
        return (count - 1) / ((lastNs - firstNs) / 1e9);
    }

    private static final class OpCounter {
        long count;
        long flagged;
        long ioNs;
        long minNs = Long.MAX_VALUE;
        long maxNs;
        long firstNs = -1;
        long lastNs = -1;

        void add(long t, long latencyNs, boolean flag) {
            count++;
            if (flag) flagged++;
            ioNs += latencyNs;
            if (!flag || latencyNs > 0) {
                if (latencyNs < minNs) minNs = latencyNs;
                if (latencyNs > maxNs) maxNs = latencyNs;
            }
            if (firstNs < 0) firstNs = t;
            lastNs = t;
        }

        OpSnapshot snapshot(HwOp op) {
            long timed = op.isRead() ? count - flagged : count;
            return new OpSnapshot(op, count, flagged,
                    timed > 0 ? ioNs / 1e3 / timed : 0,
                    minNs == Long.MAX_VALUE ? 0 : minNs / 1e3,
                    maxNs / 1e3,
                    spanHz(count, firstNs, lastNs),
                    firstNs, lastNs);
        }
    }

    /** Fixed-size ring of timestamps, used for "events per second over the last N ns". */
    private static final class TimeRing {
        private final long[] t;
        private int head;
        private int size;

        TimeRing(int capacity) {
            t = new long[capacity];
        }

        void add(long ns) {
            t[head] = ns;
            head = (head + 1) % t.length;
            if (size < t.length) size++;
        }

        void clear() {
            head = size = 0;
        }

        double rateHz(long nowNs, long windowNs) {
            if (size == 0) return 0;
            long cutoff = nowNs - windowNs;
            int counted = 0;
            long oldest = 0;
            for (int k = 1; k <= size; k++) {
                long v = t[(head - k + t.length) % t.length];
                if (v < cutoff) break;
                counted++;
                oldest = v;
            }
            if (counted == size && size == t.length) {
                // the whole ring fits in the window: extrapolate from the span it covers
                long newest = t[(head - 1 + t.length) % t.length];
                return newest > oldest ? (counted - 1) / ((newest - oldest) / 1e9) : 0;
            }
            return counted / (windowNs / 1e9);
        }
    }

    // ------------------------------------------------------------------ snapshots

    /** Immutable statistics for one operation (e.g. {@code getCurrentPosition}). */
    public static final class OpSnapshot {
        public final HwOp op;
        public final long count;
        /** Cache hits for reads, redundant (same-value) writes for writes. */
        public final long flagged;
        public final double avgLatencyUs;
        public final double minLatencyUs;
        public final double maxLatencyUs;
        public final double avgHz;
        public final long firstNs;
        public final long lastNs;

        OpSnapshot(HwOp op, long count, long flagged, double avgLatencyUs, double minLatencyUs,
                   double maxLatencyUs, double avgHz, long firstNs, long lastNs) {
            this.op = op;
            this.count = count;
            this.flagged = flagged;
            this.avgLatencyUs = avgLatencyUs;
            this.minLatencyUs = minLatencyUs;
            this.maxLatencyUs = maxLatencyUs;
            this.avgHz = avgHz;
            this.firstNs = firstNs;
            this.lastNs = lastNs;
        }
    }

    /** Immutable statistics for one device. Times are {@link System#nanoTime()} values. */
    public static final class Snapshot {
        public final String name;
        public final String type;
        public final long takenNs;
        public final long createdNs;
        public final long createdWallMs;
        public final long reads;
        public final long writes;
        public final long cacheHits;
        public final long hardwareReads;
        public final long redundantWrites;
        public final double recentReadHz;
        public final double recentWriteHz;
        public final double avgReadHz;
        public final double avgWriteHz;
        /** Mean latency of reads that hit the hardware (cache hits excluded). */
        public final double avgReadLatencyUs;
        public final double maxReadLatencyUs;
        public final double avgWriteLatencyUs;
        public final double maxWriteLatencyUs;
        public final double totalIoMs;
        public final long firstReadNs;
        public final long lastReadNs;
        public final long firstWriteNs;
        public final long lastWriteNs;
        public final long monitorReads;
        public final double monitorIoMs;
        public final List<OpSnapshot> ops;
        public final long[] recentTimeNs;
        public final int[] recentLatencyNs;
        public final HwOp[] recentOp;
        /** Cache hit (reads) or redundant write (writes) for each recent event. */
        public final boolean[] recentFlag;

        Snapshot(String name, String type, long takenNs, long createdNs, long createdWallMs,
                 long reads, long writes, long cacheHits, long hardwareReads, long redundantWrites,
                 double recentReadHz, double recentWriteHz, double avgReadHz, double avgWriteHz,
                 double avgReadLatencyUs, double maxReadLatencyUs,
                 double avgWriteLatencyUs, double maxWriteLatencyUs, double totalIoMs,
                 long firstReadNs, long lastReadNs, long firstWriteNs, long lastWriteNs,
                 long monitorReads, double monitorIoMs, List<OpSnapshot> ops,
                 long[] recentTimeNs, int[] recentLatencyNs, HwOp[] recentOp, boolean[] recentFlag) {
            this.name = name;
            this.type = type;
            this.takenNs = takenNs;
            this.createdNs = createdNs;
            this.createdWallMs = createdWallMs;
            this.reads = reads;
            this.writes = writes;
            this.cacheHits = cacheHits;
            this.hardwareReads = hardwareReads;
            this.redundantWrites = redundantWrites;
            this.recentReadHz = recentReadHz;
            this.recentWriteHz = recentWriteHz;
            this.avgReadHz = avgReadHz;
            this.avgWriteHz = avgWriteHz;
            this.avgReadLatencyUs = avgReadLatencyUs;
            this.maxReadLatencyUs = maxReadLatencyUs;
            this.avgWriteLatencyUs = avgWriteLatencyUs;
            this.maxWriteLatencyUs = maxWriteLatencyUs;
            this.totalIoMs = totalIoMs;
            this.firstReadNs = firstReadNs;
            this.lastReadNs = lastReadNs;
            this.firstWriteNs = firstWriteNs;
            this.lastWriteNs = lastWriteNs;
            this.monitorReads = monitorReads;
            this.monitorIoMs = monitorIoMs;
            this.ops = ops;
            this.recentTimeNs = recentTimeNs;
            this.recentLatencyNs = recentLatencyNs;
            this.recentOp = recentOp;
            this.recentFlag = recentFlag;
        }

        /** Percentage of reads answered by the read cache. */
        public double cacheHitPercent() {
            return reads > 0 ? 100.0 * cacheHits / reads : 0;
        }
    }
}
