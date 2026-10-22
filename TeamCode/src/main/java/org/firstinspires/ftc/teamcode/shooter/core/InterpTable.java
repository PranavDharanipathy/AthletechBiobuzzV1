package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Piecewise-linear 1D lookup with strictly increasing x. Clamps outside the range.
 * Lookups are O(log n) and allocation-free.
 */
public final class InterpTable {

    private final double[] xs;
    private final double[] ys;

    public InterpTable(double[] xs, double[] ys) {
        if (xs.length != ys.length || xs.length < 2) {
            throw new IllegalArgumentException("need >= 2 points of equal length");
        }
        for (int i = 1; i < xs.length; i++) {
            if (!(xs[i] > xs[i - 1])) throw new IllegalArgumentException("x must be strictly increasing");
        }
        this.xs = xs.clone();
        this.ys = ys.clone();
    }

    /** Builds a table from points whose x may be decreasing (e.g. a reversed servo); sorts by x. */
    public static InterpTable fromUnsorted(double[] xs, double[] ys) {
        int n = xs.length;
        Integer[] idx = new Integer[n];
        for (int i = 0; i < n; i++) idx[i] = i;
        java.util.Arrays.sort(idx, (a, b) -> Double.compare(xs[a], xs[b]));
        double[] sx = new double[n], sy = new double[n];
        for (int i = 0; i < n; i++) {
            sx[i] = xs[idx[i]];
            sy[i] = ys[idx[i]];
        }
        return new InterpTable(sx, sy);
    }

    /** Returns the inverse table (y → x). Requires y to be strictly monotonic. */
    public InterpTable inverse() {
        return fromUnsorted(ys, xs);
    }

    public double get(double x) {
        int n = xs.length;
        if (!(x > xs[0])) return ys[0]; // also catches NaN → first point
        if (x >= xs[n - 1]) return ys[n - 1];
        int lo = 0, hi = n - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) >>> 1;
            if (xs[mid] <= x) lo = mid; else hi = mid;
        }
        double t = (x - xs[lo]) / (xs[hi] - xs[lo]);
        return ys[lo] + (ys[hi] - ys[lo]) * t;
    }

    public double minX() { return xs[0]; }
    public double maxX() { return xs[xs.length - 1]; }
    public int size() { return xs.length; }
}
