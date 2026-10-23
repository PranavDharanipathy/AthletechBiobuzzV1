package org.firstinspires.ftc.teamcode.Utility.Monitoring;

/**
 * Fixed-bucket histogram for percentiles over an unbounded number of samples (e.g. every loop
 * time of a long practice session) in constant memory. Accuracy is one bucket width.
 * Thread-safe.
 */
public final class Histogram {
    private final double bucketWidth;
    private final long[] buckets;
    private long overflow;
    private double overflowMax;
    private long total;

    /** Covers {@code [0, bucketWidth * bucketCount)}; larger values land in an overflow bucket. */
    public Histogram(double bucketWidth, int bucketCount) {
        this.bucketWidth = bucketWidth;
        this.buckets = new long[bucketCount];
    }

    public synchronized void add(double value) {
        if (Double.isNaN(value) || value < 0) return;
        total++;
        int idx = (int) (value / bucketWidth);
        if (idx >= buckets.length) {
            overflow++;
            if (value > overflowMax) overflowMax = value;
        } else {
            buckets[idx]++;
        }
    }

    public synchronized long count() {
        return total;
    }

    /** Value below which {@code p} percent of samples fall, e.g. {@code percentile(95)}. */
    public synchronized double percentile(double p) {
        if (total == 0) return Double.NaN;
        long target = (long) Math.ceil(total * Math.min(100, Math.max(0, p)) / 100.0);
        if (target < 1) target = 1;
        long seen = 0;
        for (int i = 0; i < buckets.length; i++) {
            seen += buckets[i];
            if (seen >= target) return (i + 0.5) * bucketWidth;
        }
        return overflowMax;
    }

    public synchronized void reset() {
        java.util.Arrays.fill(buckets, 0);
        overflow = 0;
        overflowMax = 0;
        total = 0;
    }
}
