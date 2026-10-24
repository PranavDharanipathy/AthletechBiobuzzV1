package org.athletech.sim.undeterminables;

import org.athletech.sim.utility.Rng;

/**
 * How long each control loop takes and when, inside a loop, sensors are read and motors are
 * commanded. FTC loops run: hardware reads (bulk read, I2C sensors) -> your code -> hardware
 * writes. So a loop k that starts at t_k
 * <pre>
 *   reads sensors at   t_k + read time                       (sense)
 *   commands motors at t_k + read + logic + half the writes  (act; writes are spread out)
 *   ends at            t_k + read + logic + writes            (period)
 * </pre>
 * Each hardware call's time is drawn from a lognormal around its average (coefficient of variation
 * {@code ioCv}); fractional counts (e.g. 1.5 reads per loop) are randomized per loop. Your code
 * takes {@code logicMs} plus random jitter, and occasionally a spike (garbage collection, USB
 * hiccup). Loops never take less than 0.2 ms.
 */
public final class LoopTimingModel {

    /** One loop: when, relative to its start, sensors were read and motors commanded. */
    public static final class Loop {
        public final double periodMs, senseMs, actMs;

        public Loop(double periodMs, double senseMs, double actMs) {
            this.periodMs = periodMs;
            this.senseMs = senseMs;
            this.actMs = actMs;
        }
    }

    /** What one loop is made of. */
    public static final class Params {
        public final double logicMs;
        public final double motorReadMs, motorReads;
        public final double motorWriteMs, motorWrites;
        public final double servoWriteMs, servoWrites;
        public final double sensorReadMs, sensorReads;
        public final double otherReadMs, otherReads;
        public final double ioCv;
        public final Distribution jitterMs;
        public final double spikeProb;
        public final Distribution spikeMs;

        public Params(double logicMs, double motorReadMs, double motorReads, double motorWriteMs, double motorWrites,
                      double servoWriteMs, double servoWrites, double sensorReadMs, double sensorReads,
                      double otherReadMs, double otherReads, double ioCv, Distribution jitterMs, double spikeProb, Distribution spikeMs) {
            this.logicMs = logicMs;
            this.motorReadMs = motorReadMs;
            this.motorReads = motorReads;
            this.motorWriteMs = motorWriteMs;
            this.motorWrites = motorWrites;
            this.servoWriteMs = servoWriteMs;
            this.servoWrites = servoWrites;
            this.sensorReadMs = sensorReadMs;
            this.sensorReads = sensorReads;
            this.otherReadMs = otherReadMs;
            this.otherReads = otherReads;
            this.ioCv = ioCv;
            this.jitterMs = jitterMs;
            this.spikeProb = spikeProb;
            this.spikeMs = spikeMs;
        }

        public double readMs() {
            return motorReadMs * motorReads + sensorReadMs * sensorReads + otherReadMs * otherReads;
        }

        public double writeMs() {
            return motorWriteMs * motorWrites + servoWriteMs * servoWrites;
        }

        /** Average loop time, ms (jitter is zero-mean; spikes add spikeProb * their mean). */
        public double meanMs() {
            double spike = spikeProb > 0 && spikeMs.isKnown() ? spikeProb * Math.max(0, spikeMs.mean()) : 0;
            double jitterMean = jitterMs.isKnown() && Double.isFinite(jitterMs.mean()) ? jitterMs.mean() : 0;
            return logicMs + jitterMean + readMs() + writeMs() + spike;
        }

        public Params withSensorReads(double n) {
            return new Params(logicMs, motorReadMs, motorReads, motorWriteMs, motorWrites, servoWriteMs, servoWrites,
                    sensorReadMs, n, otherReadMs, otherReads, ioCv, jitterMs, spikeProb, spikeMs);
        }

        public Params withMotorWrites(double n) {
            return new Params(logicMs, motorReadMs, motorReads, motorWriteMs, n, servoWriteMs, servoWrites,
                    sensorReadMs, sensorReads, otherReadMs, otherReads, ioCv, jitterMs, spikeProb, spikeMs);
        }

        /** Rows {component, count per loop, ms each, ms per loop} for reports. */
        public double[][] breakdown() {
            return new double[][]{
                    {Double.NaN, logicMs, logicMs},
                    {motorReads, motorReadMs, motorReads * motorReadMs},
                    {sensorReads, sensorReadMs, sensorReads * sensorReadMs},
                    {otherReads, otherReadMs, otherReads * otherReadMs},
                    {motorWrites, motorWriteMs, motorWrites * motorWriteMs},
                    {servoWrites, servoWriteMs, servoWrites * servoWriteMs}};
        }

        public static final String[] BREAKDOWN_NAMES = {"your code (logic)", "motor reads", "I2C sensor reads", "other reads", "motor writes", "servo writes"};
    }

    private final Params p;
    private final double meanMs, cvMs, readFrac, writeFrac;
    private final boolean synthetic;
    private final Distribution periods;

    private LoopTimingModel(Params p, double meanMs, double cvMs, double readFrac, double writeFrac, boolean synthetic, Distribution periods) {
        this.p = p;
        this.meanMs = meanMs;
        this.cvMs = cvMs;
        this.readFrac = readFrac;
        this.writeFrac = writeFrac;
        this.synthetic = synthetic;
        this.periods = periods;
    }

    /** A loop built from its parts (logic + reads + writes + randomness). */
    public static LoopTimingModel of(Params p) {
        return new LoopTimingModel(p, p.meanMs(), Double.NaN, Double.NaN, Double.NaN, false, null);
    }

    /**
     * A loop with a given average and randomness (std), for "what if the loop were X ms" sweeps.
     * Periods are lognormal; reads take {@code readFrac} of each loop and writes {@code writeFrac}.
     */
    public static LoopTimingModel synthetic(double meanMs, double stdMs, double readFrac, double writeFrac) {
        return new LoopTimingModel(null, meanMs, stdMs, clamp01(readFrac), clamp01(writeFrac), true, null);
    }

    /**
     * Loop times drawn from a measured distribution (e.g. HardwareMonitor's empirical samples) when
     * the loop's parts are not known; reads/writes take fixed shares of each loop.
     */
    public static LoopTimingModel fromDistribution(Distribution d, double readFrac, double writeFrac) {
        double mean = Double.isFinite(d.mean()) ? d.mean() : 10;
        return new LoopTimingModel(null, mean, Double.NaN, clamp01(readFrac), clamp01(writeFrac), true, d);
    }

    private static double clamp01(double v) {
        return Double.isFinite(v) ? Math.max(0, Math.min(0.95, v)) : 0.3;
    }

    public Params params() { return p; }

    public double meanMs() { return meanMs; }

    /** Share of the loop spent reading (before the code runs), from the parts. */
    public double readFraction() {
        if (synthetic) return readFrac;
        double m = p.meanMs();
        return m > 0 ? p.readMs() / m : 0;
    }

    public double writeFraction() {
        if (synthetic) return writeFrac;
        double m = p.meanMs();
        return m > 0 ? p.writeMs() / m : 0;
    }

    public Loop next(Rng rng) {
        if (synthetic) {
            double period = meanMs;
            if (periods != null) {
                period = periods.sample(rng);
            } else if (cvMs > 0) {
                double s2 = Math.log(1 + (cvMs * cvMs) / (meanMs * meanMs));
                period = Math.exp(Math.log(meanMs) - s2 / 2 + Math.sqrt(s2) * rng.gaussian());
            }
            period = Math.max(0.2, period);
            double read = period * readFrac, write = period * writeFrac;
            return new Loop(period, read, period - write / 2);
        }
        double read = ops(rng, p.motorReadMs, p.motorReads) + ops(rng, p.sensorReadMs, p.sensorReads) + ops(rng, p.otherReadMs, p.otherReads);
        double write = ops(rng, p.motorWriteMs, p.motorWrites) + ops(rng, p.servoWriteMs, p.servoWrites);
        double logic = p.logicMs + (p.jitterMs.isKnown() ? p.jitterMs.sample(rng) : 0);
        if (p.spikeProb > 0 && rng.uniform() < p.spikeProb && p.spikeMs.isKnown()) logic += Math.max(0, p.spikeMs.sample(rng));
        logic = Math.max(0.05, logic);
        double period = Math.max(0.2, read + logic + write);
        return new Loop(period, read, read + logic + write / 2);
    }

    /** Total time of {@code count} calls averaging {@code eachMs} (fractional counts randomized). */
    private double ops(Rng rng, double eachMs, double count) {
        if (!(eachMs > 0) || !(count > 0)) return 0;
        int n = (int) Math.floor(count);
        if (rng.uniform() < count - n) n++;
        if (n == 0) return 0;
        double cv = Double.isFinite(p.ioCv) ? p.ioCv : 0;
        if (cv <= 0) return n * eachMs;
        double s2 = Math.log(1 + cv * cv), mu = Math.log(eachMs) - s2 / 2, s = Math.sqrt(s2);
        double t = 0;
        for (int i = 0; i < n; i++) t += Math.exp(mu + s * rng.gaussian());
        return t;
    }

    /** {@code n} loop times (ms), e.g. for a histogram or percentiles. */
    public double[] samplePeriods(Rng rng, int n) {
        double[] out = new double[n];
        for (int i = 0; i < n; i++) out[i] = next(rng).periodMs;
        return out;
    }
}
