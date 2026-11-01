package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Offline fit of u = kS·sgn(v) + kV·v + kA·a from logged (time, position, input) samples.
 *
 * <p>Velocity and acceleration come from <b>centered</b> finite differences over ±k samples. A centered
 * window has zero phase lag, unlike any causal filter or observer, and that removes the bias a lagging
 * acceleration estimate puts into kS and kA during ramps. This is only possible offline: the characterization
 * OpMode stores the run and fits at the end.
 */
public final class SysIdFitter {

    public static final class Result {
        public double kS, kV, kA, rSquared;
        public int samples;
        public boolean ok;

        @Override
        public String toString() {
            return ok ? String.format(java.util.Locale.US, "kS=%.4f kV=%.5f kA=%.5f R²=%.4f (n=%d)", kS, kV, kA, rSquared, samples)
                    : "fit failed (not enough excitation)";
        }
    }

    private SysIdFitter() {}

    /**
     * @param t          sample times (s)
     * @param pos        positions (rad)
     * @param input      applied input (volts for the flywheel, power for servos), held over the sample
     * @param n          number of valid samples
     * @param halfWindow k (e.g. 3–6 samples)
     * @param minSpeed   samples slower than this are skipped (static-friction region is not modeled)
     */
    public static Result fit(double[] t, double[] pos, double[] input, int n, int halfWindow, double minSpeed) {
        int k = Math.max(1, halfWindow);
        double[] v = new double[n];
        for (int i = k; i < n - k; i++) {
            double dt = t[i + k] - t[i - k];
            v[i] = dt > 0 ? (pos[i + k] - pos[i - k]) / dt : Double.NaN;
        }
        LeastSquares ls = new LeastSquares(3);
        double[] x = new double[3];
        for (int i = 2 * k; i < n - 2 * k; i++) {
            double dt = t[i + k] - t[i - k];
            if (!(dt > 0)) continue;
            double a = (v[i + k] - v[i - k]) / dt;
            if (!MathUtil.isFinite(v[i]) || !MathUtil.isFinite(a) || Math.abs(v[i]) < minSpeed) continue;
            x[0] = Math.signum(v[i]);
            x[1] = v[i];
            x[2] = a;
            ls.add(x, input[i]);
        }
        Result r = new Result();
        double[] beta = ls.solve();
        r.samples = (int) ls.count();
        if (beta == null || r.samples < 20) return r;
        r.kS = beta[0];
        r.kV = beta[1];
        r.kA = beta[2];
        r.rSquared = ls.rSquared(beta);
        r.ok = r.kV > 0 && r.kA > 0;
        return r;
    }
}
