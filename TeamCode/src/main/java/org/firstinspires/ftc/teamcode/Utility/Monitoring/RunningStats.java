package org.firstinspires.ftc.teamcode.Utility.Monitoring;

/**
 * Streaming count / mean / standard deviation / min / max (Welford's algorithm). Constant
 * memory no matter how many samples are added. Thread-safe.
 */
public final class RunningStats {
    private long count;
    private double mean;
    private double m2;
    private double min = Double.POSITIVE_INFINITY;
    private double max = Double.NEGATIVE_INFINITY;
    private double last = Double.NaN;

    /** Adds a sample. NaN and infinite values are ignored. */
    public synchronized void add(double x) {
        if (Double.isNaN(x) || Double.isInfinite(x)) return;
        count++;
        double delta = x - mean;
        mean += delta / count;
        m2 += delta * (x - mean);
        if (x < min) min = x;
        if (x > max) max = x;
        last = x;
    }

    public synchronized long count() {
        return count;
    }

    public synchronized double mean() {
        return count > 0 ? mean : Double.NaN;
    }

    public synchronized double stddev() {
        return count > 1 ? Math.sqrt(m2 / (count - 1)) : Double.NaN;
    }

    public synchronized double min() {
        return count > 0 ? min : Double.NaN;
    }

    public synchronized double max() {
        return count > 0 ? max : Double.NaN;
    }

    public synchronized double last() {
        return last;
    }

    public synchronized double sum() {
        return count > 0 ? mean * count : 0;
    }

    public synchronized void reset() {
        count = 0;
        mean = m2 = 0;
        min = Double.POSITIVE_INFINITY;
        max = Double.NEGATIVE_INFINITY;
        last = Double.NaN;
    }
}
