package org.athletech.sim.undeterminables;

import org.athletech.sim.utility.Rng;

/**
 * A random error or variation ("undeterminable"): how much a quantity scatters from shot to shot,
 * loop to loop or match to match. Read from the profile's {@code distributions} table.
 */
public final class Distribution {
    private final String name;
    private final Kind kind;
    private final double mean;
    private final double std;
    private final double min;
    private final double max;
    private final double mode;
    private final double[] samples;
    private final String unit;

    public Distribution(String name, Kind kind, double mean, double std, double min, double max, double mode, double[] samples, String unit) {
        this.name = name;
        this.kind = kind;
        this.mean = mean;
        this.std = std;
        this.min = min;
        this.max = max;
        this.mode = mode;
        this.samples = samples;
        this.unit = unit;
    }

    public String name() { return name; }
    public Kind kind() { return kind; }
    public double mean() { return mean; }
    public double std() { return std; }
    public double min() { return min; }
    public double max() { return max; }
    public double mode() { return mode; }
    public double[] samples() { return samples; }
    public String unit() { return unit; }

    @Override
    public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof Distribution)) return false;
        Distribution o = (Distribution) other;
        return java.util.Objects.equals(name, o.name)
                && java.util.Objects.equals(kind, o.kind)
                && Double.compare(mean, o.mean) == 0
                && Double.compare(std, o.std) == 0
                && Double.compare(min, o.min) == 0
                && Double.compare(max, o.max) == 0
                && Double.compare(mode, o.mode) == 0
                && java.util.Objects.equals(samples, o.samples)
                && java.util.Objects.equals(unit, o.unit);
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(name, kind, mean, std, min, max, mode, samples, unit);
    }

    @Override
    public String toString() {
        return "Distribution[name=" + name + ", kind=" + kind + ", mean=" + mean + ", std=" + std + ", min=" + min + ", max=" + max + ", mode=" + mode + ", samples=" + samples + ", unit=" + unit + "]";
    }

    public enum Kind {
        /** Not measured yet: always returns 0 (no error). */
        NONE,
        CONSTANT, NORMAL, UNIFORM, TRIANGULAR, LOGNORMAL,
        /** Draws from recorded samples. */
        EMPIRICAL;

        public static Kind parse(String s) {
            if (s == null || s.trim().isEmpty()) return NONE;
            return Kind.valueOf(s.trim().toUpperCase(java.util.Locale.ROOT));
        }
    }

    public static Distribution none(String name, String unit) {
        return new Distribution(name, Kind.NONE, 0, 0, Double.NaN, Double.NaN, Double.NaN, new double[0], unit);
    }

    public static Distribution normal(String name, double mean, double std, String unit) {
        return new Distribution(name, Kind.NORMAL, mean, std, Double.NaN, Double.NaN, Double.NaN, new double[0], unit);
    }

    public boolean isKnown() {
        return kind != Kind.NONE;
    }

    private static double or(double v, double fallback) {
        return Double.isNaN(v) ? fallback : v;
    }

    public double sample(Rng rng) {
        double m = or(mean, 0), s = or(std, 0);
        double v;
        switch (kind) {
            case CONSTANT:
                v = m;
                break;
            case NORMAL:
                v = rng.gaussian(m, s);
                break;
            case UNIFORM:
                v = rng.uniform(or(min, m - s * Math.sqrt(3)), or(max, m + s * Math.sqrt(3)));
                break;
            case TRIANGULAR:
                v = triangular(rng, or(min, m - s * Math.sqrt(6)), or(max, m + s * Math.sqrt(6)), or(mode, m));
                break;
            case LOGNORMAL: {
                // parameterised by the mean and std of the value itself (must be > 0)
                double variance = s * s, mm = Math.max(1e-12, m);
                double sigma2 = Math.log(1 + variance / (mm * mm));
                double mu = Math.log(mm) - sigma2 / 2;
                v = Math.exp(mu + Math.sqrt(sigma2) * rng.gaussian());
                break;
            }
            case EMPIRICAL:
                v = samples.length == 0 ? m : samples[rng.nextInt(samples.length)];
                break;
            default: // NONE
                v = 0;
        }
        if (kind != Kind.UNIFORM && kind != Kind.TRIANGULAR) {
            if (!Double.isNaN(min)) v = Math.max(min, v);
            if (!Double.isNaN(max)) v = Math.min(max, v);
        }
        return v;
    }

    private static double triangular(Rng rng, double a, double b, double c) {
        if (!(b > a)) return a;
        c = Math.max(a, Math.min(b, c));
        double u = rng.uniform(), f = (c - a) / (b - a);
        return u < f ? a + Math.sqrt(u * (b - a) * (c - a)) : b - Math.sqrt((1 - u) * (b - a) * (b - c));
    }

    private static String f(double v) {
        return org.athletech.sim.utility.Csv.fmt(Math.abs(v) < 1e-12 ? 0 : v);
    }

    /** One-line description, e.g. "normal(mean 0, std 35 rpm)". */
    public String describe() {
        switch (kind) {
            case NONE: return "not set (treated as zero error)";
            case CONSTANT: return "constant " + f(mean) + " " + unit;
            case EMPIRICAL: return "empirical (" + samples.length + " samples) " + unit;
            default: return kind.name().toLowerCase(java.util.Locale.ROOT) + "(mean " + f(mean) + ", std " + f(std) + " " + unit + ")";
        }
    }
}
