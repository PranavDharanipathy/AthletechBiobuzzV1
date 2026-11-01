package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Streaming ordinary least squares y ≈ Σ βk·xk for a few parameters (system identification).
 * It accumulates the normal equations, so no samples are stored. Solves by Gaussian elimination with partial
 * pivoting. Reports R² so you can judge whether the model fits.
 */
public final class LeastSquares {

    private final int p;
    private final double[][] ata;
    private final double[] aty;
    private double sumY, sumYY;
    private long count;

    public LeastSquares(int parameters) {
        p = parameters;
        ata = new double[p][p];
        aty = new double[p];
    }

    /** Adds one sample; {@code x.length} must equal the parameter count. */
    public void add(double[] x, double y) {
        for (double v : x) if (!MathUtil.isFinite(v)) return;
        if (!MathUtil.isFinite(y)) return;
        for (int i = 0; i < p; i++) {
            aty[i] += x[i] * y;
            for (int j = 0; j < p; j++) ata[i][j] += x[i] * x[j];
        }
        sumY += y;
        sumYY += y * y;
        count++;
    }

    public long count() { return count; }

    /** @return β, or null if the system is singular (not enough excitation). */
    public double[] solve() {
        double[][] m = new double[p][p + 1];
        for (int i = 0; i < p; i++) {
            System.arraycopy(ata[i], 0, m[i], 0, p);
            m[i][p] = aty[i];
        }
        for (int col = 0; col < p; col++) {
            int piv = col;
            for (int r = col + 1; r < p; r++) if (Math.abs(m[r][col]) > Math.abs(m[piv][col])) piv = r;
            if (Math.abs(m[piv][col]) < 1e-12) return null;
            double[] tmp = m[col]; m[col] = m[piv]; m[piv] = tmp;
            for (int r = 0; r < p; r++) {
                if (r == col) continue;
                double f = m[r][col] / m[col][col];
                for (int c = col; c <= p; c++) m[r][c] -= f * m[col][c];
            }
        }
        double[] beta = new double[p];
        for (int i = 0; i < p; i++) beta[i] = m[i][p] / m[i][i];
        return beta;
    }

    /** Coefficient of determination for solution β (computed from the accumulated sums). */
    public double rSquared(double[] beta) {
        if (beta == null || count < 2) return Double.NaN;
        // SSE = yᵀy − 2βᵀAᵀy + βᵀAᵀAβ
        double btaty = 0, btatab = 0;
        for (int i = 0; i < p; i++) {
            btaty += beta[i] * aty[i];
            for (int j = 0; j < p; j++) btatab += beta[i] * ata[i][j] * beta[j];
        }
        double sse = sumYY - 2 * btaty + btatab;
        double mean = sumY / count;
        double sst = sumYY - count * mean * mean;
        return sst > 0 ? 1 - sse / sst : Double.NaN;
    }
}
