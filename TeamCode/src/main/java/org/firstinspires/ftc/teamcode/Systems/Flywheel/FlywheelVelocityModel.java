package org.firstinspires.ftc.teamcode.Systems.Flywheel;

import java.util.List;

public class FlywheelVelocityModel {

    public static final int MAX_DEGREE = 3;

    private double[] xs = new double[0], ys = new double[0];
    private int requested = -1, degree;
    private final double[] coeffs = new double[MAX_DEGREE + 1];
    private double mean, scale = 1, minX, maxX, rms = Double.NaN;
    private boolean valid;
    public int fits;

    public double velocity(List<Double> distances, List<Double> velocities, int degree, double extrapolate, double distance) {

        if (changed(distances, velocities, degree)) fit(distances, velocities, degree);
        if (!valid || distance != distance) return Double.NaN;
        return evaluate(Math.max(minX - extrapolate, Math.min(maxX + extrapolate, distance)));
    }

    public double evaluate(double x) {

        double u = (x - mean) / scale, y = 0;
        for (int k = degree; k >= 0; k--) y = y * u + coeffs[k];
        return y;
    }

    private boolean changed(List<Double> distances, List<Double> velocities, int degree) {

        int n = Math.min(distances.size(), velocities.size());
        if (degree != requested || n != xs.length) return true;
        for (int i = 0; i < n; i++) {
            //bit compare so NaN doesn't look changed
            if (Double.doubleToLongBits(distances.get(i)) != Double.doubleToLongBits(xs[i]) || Double.doubleToLongBits(velocities.get(i)) != Double.doubleToLongBits(ys[i])) return true;
        }
        return false;
    }

    public void fit(List<Double> distances, List<Double> velocities, int degree) {

        int n = Math.min(distances.size(), velocities.size());
        xs = new double[n];
        ys = new double[n];
        requested = degree;
        fits++;
        int used = 0;
        double sum = 0;
        minX = Double.POSITIVE_INFINITY;
        maxX = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < n; i++) {
            xs[i] = distances.get(i);
            ys[i] = velocities.get(i);
            if (!finite(xs[i]) || !finite(ys[i])) continue;
            used++;
            sum += xs[i];
            minX = Math.min(minX, xs[i]);
            maxX = Math.max(maxX, xs[i]);
        }
        valid = false;
        if (used == 0) return;
        this.degree = Math.max(0, Math.min(Math.min(degree, MAX_DEGREE), used - 1));
        mean = sum / used;
        scale = Math.max(1e-9, (maxX - minX) / 2);
        int m = this.degree + 1;
        double[][] a = new double[m][m + 1];
        for (int i = 0; i < n; i++) {
            if (!finite(xs[i]) || !finite(ys[i])) continue;
            double u = (xs[i] - mean) / scale;
            double[] p = new double[2 * m - 1];
            p[0] = 1;
            for (int k = 1; k < p.length; k++) p[k] = p[k - 1] * u;
            for (int r = 0; r < m; r++) {
                for (int c = 0; c < m; c++) a[r][c] += p[r + c];
                a[r][m] += p[r] * ys[i];
            }
        }
        if (!solve(a, m)) return;
        for (int k = 0; k < m; k++) coeffs[k] = a[k][m];
        double e2 = 0;
        for (int i = 0; i < n; i++) {
            if (!finite(xs[i]) || !finite(ys[i])) continue;
            double e = ys[i] - evaluate(xs[i]);
            e2 += e * e;
        }
        rms = Math.sqrt(e2 / used);
        valid = true;
    }

    private static boolean solve(double[][] a, int m) {

        for (int col = 0; col < m; col++) {
            int pivot = col;
            for (int r = col + 1; r < m; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
            }
            if (Math.abs(a[pivot][col]) < 1e-12) return false;
            double[] t = a[col];
            a[col] = a[pivot];
            a[pivot] = t;
            for (int r = 0; r < m; r++) {
                if (r == col) continue;
                double f = a[r][col] / a[col][col];
                for (int c = col; c <= m; c++) a[r][c] -= f * a[col][c];
            }
        }
        for (int r = 0; r < m; r++) a[r][m] /= a[r][r];
        return true;
    }

    private static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    public double distanceFor(double velocity, double extrapolate) {

        if (!valid || velocity != velocity) return Double.NaN;
        double lo = minX - extrapolate, hi = maxX + extrapolate, flo = evaluate(lo), fhi = evaluate(hi);
        boolean up = fhi >= flo;
        double low = Math.min(flo, fhi), high = Math.max(flo, fhi);
        if (velocity < low - 1 || velocity > high + 1) return Double.NaN;
        for (int k = 0; k < 50; k++) {
            double mid = 0.5 * (lo + hi);
            if ((evaluate(mid) < velocity) == up) lo = mid;
            else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    public boolean isValid() {
        return valid;
    }

    public int degree() {
        return degree;
    }

    public double rms() {
        return rms;
    }

    public double minDistance() {
        return minX;
    }

    public double maxDistance() {
        return maxX;
    }

    public String describe() {

        if (!valid) return "no fit (need distance/velocity points)";
        StringBuilder sb = new StringBuilder();
        for (int k = 0; k <= degree; k++) {
            if (k > 0) sb.append(coeffs[k] < 0 ? " - " : " + ");
            sb.append(String.format(java.util.Locale.US, "%.1f", k > 0 ? Math.abs(coeffs[k]) : coeffs[k]));
            if (k == 1) sb.append(" u");
            if (k > 1) sb.append(" u^").append(k);
        }
        return sb.append(String.format(java.util.Locale.US, ", u = (d - %.1f) / %.1f, rms %.1f", mean, scale, rms)).toString();
    }
}
