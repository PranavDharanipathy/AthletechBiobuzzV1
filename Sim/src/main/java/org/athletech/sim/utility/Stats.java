package org.athletech.sim.utility;

import java.util.Arrays;

/** Small statistics helpers (no external dependencies). */
public final class Stats {
    private Stats() {
    }

    public static double mean(double[] v) {
        if (v.length == 0) return Double.NaN;
        double s = 0;
        for (double x : v) s += x;
        return s / v.length;
    }

    /** Sample standard deviation (n - 1). */
    public static double std(double[] v) {
        if (v.length < 2) return Double.NaN;
        double m = mean(v), s = 0;
        for (double x : v) s += (x - m) * (x - m);
        return Math.sqrt(s / (v.length - 1));
    }

    public static double rms(double[] v) {
        if (v.length == 0) return Double.NaN;
        double s = 0;
        for (double x : v) s += x * x;
        return Math.sqrt(s / v.length);
    }

    /** Percentile (0..100) with linear interpolation. */
    public static double percentile(double[] v, double p) {
        if (v.length == 0) return Double.NaN;
        double[] s = v.clone();
        Arrays.sort(s);
        double pos = (p / 100.0) * (s.length - 1);
        int lo = (int) Math.floor(pos), hi = (int) Math.ceil(pos);
        return s[lo] + (pos - lo) * (s[hi] - s[lo]);
    }

    /** 95% Wilson score interval for k successes in n trials: {low, high}. */
    public static double[] wilson(long k, long n) {
        if (n == 0) return new double[]{0, 1};
        double z = 1.959964, p = (double) k / n, z2 = z * z;
        double denom = 1 + z2 / n;
        double centre = (p + z2 / (2 * n)) / denom;
        double half = z * Math.sqrt(p * (1 - p) / n + z2 / (4.0 * n * n)) / denom;
        return new double[]{Math.max(0, centre - half), Math.min(1, centre + half)};
    }

    /** Standard error of a proportion, used as the "y_std" of probability curves. */
    public static double proportionStd(long k, long n) {
        if (n == 0) return Double.NaN;
        double p = (double) k / n;
        return Math.sqrt(Math.max(p * (1 - p), 0.25 / n) / n);
    }

    /** Standard normal CDF (Abramowitz-Stegun 26.2.17, error < 7.5e-8). */
    public static double normalCdf(double z) {
        if (Double.isNaN(z)) return Double.NaN;
        if (z == Double.POSITIVE_INFINITY) return 1;
        if (z == Double.NEGATIVE_INFINITY) return 0;
        double t = 1 / (1 + 0.2316419 * Math.abs(z));
        double poly = t * (0.319381530 + t * (-0.356563782 + t * (1.781477937 + t * (-1.821255978 + t * 1.330274429))));
        double tail = Math.exp(-z * z / 2) / Math.sqrt(2 * Math.PI) * poly;
        return z >= 0 ? 1 - tail : tail;
    }

    /** Ordinary least squares y = intercept + slope * x. */
    public static final class Fit {
        private final double slope;
        private final double intercept;
        private final double slopeStdErr;
        private final double interceptStdErr;
        private final double r2;
        private final int n;

        public Fit(double slope, double intercept, double slopeStdErr, double interceptStdErr, double r2, int n) {
            this.slope = slope;
            this.intercept = intercept;
            this.slopeStdErr = slopeStdErr;
            this.interceptStdErr = interceptStdErr;
            this.r2 = r2;
            this.n = n;
        }

        public double slope() { return slope; }
        public double intercept() { return intercept; }
        public double slopeStdErr() { return slopeStdErr; }
        public double interceptStdErr() { return interceptStdErr; }
        public double r2() { return r2; }
        public int n() { return n; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Fit)) return false;
            Fit o = (Fit) other;
            return Double.compare(slope, o.slope) == 0
                    && Double.compare(intercept, o.intercept) == 0
                    && Double.compare(slopeStdErr, o.slopeStdErr) == 0
                    && Double.compare(interceptStdErr, o.interceptStdErr) == 0
                    && Double.compare(r2, o.r2) == 0
                    && n == o.n;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(slope, intercept, slopeStdErr, interceptStdErr, r2, n);
        }

        @Override
        public String toString() {
            return "Fit[slope=" + slope + ", intercept=" + intercept + ", slopeStdErr=" + slopeStdErr + ", interceptStdErr=" + interceptStdErr + ", r2=" + r2 + ", n=" + n + "]";
        }
    }

    public static Fit linearFit(double[] x, double[] y) {
        int n = x.length;
        if (n < 3 || y.length != n) throw new IllegalArgumentException("need >= 3 points");
        double mx = mean(x), my = mean(y), sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < n; i++) {
            sxx += (x[i] - mx) * (x[i] - mx);
            sxy += (x[i] - mx) * (y[i] - my);
            syy += (y[i] - my) * (y[i] - my);
        }
        if (sxx == 0) throw new IllegalArgumentException("x has no spread");
        double slope = sxy / sxx, intercept = my - slope * mx;
        double sse = 0;
        for (int i = 0; i < n; i++) {
            double e = y[i] - (intercept + slope * x[i]);
            sse += e * e;
        }
        double s2 = sse / (n - 2);
        double slopeSe = Math.sqrt(s2 / sxx);
        double interceptSe = Math.sqrt(s2 * (1.0 / n + mx * mx / sxx));
        double r2 = syy == 0 ? 1 : 1 - sse / syy;
        return new Fit(slope, intercept, slopeSe, interceptSe, r2, n);
    }

    /** Streaming mean / variance / min / max (Welford). */
    public static final class Running {
        private long n;
        private double mean, m2, min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;

        public void add(double x) {
            if (Double.isNaN(x)) return;
            n++;
            double d = x - mean;
            mean += d / n;
            m2 += d * (x - mean);
            min = Math.min(min, x);
            max = Math.max(max, x);
        }

        public long n() { return n; }
        public double mean() { return n == 0 ? Double.NaN : mean; }
        public double std() { return n < 2 ? Double.NaN : Math.sqrt(m2 / (n - 1)); }
        public double min() { return n == 0 ? Double.NaN : min; }
        public double max() { return n == 0 ? Double.NaN : max; }
    }
}
