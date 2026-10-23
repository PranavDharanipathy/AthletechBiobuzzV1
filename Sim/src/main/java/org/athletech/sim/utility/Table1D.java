package org.athletech.sim.utility;

import java.util.Arrays;

/** Piecewise-linear lookup table y(x). Outside the x range the end values are held (clamped). */
public final class Table1D {
    private final double[] xs;
    private final double[] ys;

    public Table1D(double[] xs, double[] ys) {
        if (xs.length != ys.length || xs.length == 0) throw new IllegalArgumentException("need matching, non-empty x/y");
        Integer[] order = new Integer[xs.length];
        for (int i = 0; i < order.length; i++) order[i] = i;
        Arrays.sort(order, (a, b) -> Double.compare(xs[a], xs[b]));
        this.xs = new double[xs.length];
        this.ys = new double[xs.length];
        for (int i = 0; i < order.length; i++) {
            this.xs[i] = xs[order[i]];
            this.ys[i] = ys[order[i]];
        }
    }

    public double at(double x) {
        if (x <= xs[0]) return ys[0];
        int n = xs.length;
        if (x >= xs[n - 1]) return ys[n - 1];
        int i = Arrays.binarySearch(xs, x);
        if (i >= 0) return ys[i];
        int hi = -i - 1, lo = hi - 1;
        double t = (x - xs[lo]) / (xs[hi] - xs[lo]);
        return ys[lo] + t * (ys[hi] - ys[lo]);
    }

    public double minX() { return xs[0]; }
    public double maxX() { return xs[xs.length - 1]; }
    public int size() { return xs.length; }
}
