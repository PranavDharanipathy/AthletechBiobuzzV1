package org.athletech.sim.undeterminables;

public final class TileChargeGrid {
    private final int n;
    private final double half, cell;
    private final double[] q;

    public TileChargeGrid(double fieldSizeM, int cells) {

        this.n = cells;
        this.half = fieldSizeM / 2;
        this.cell = fieldSizeM / cells;
        this.q = new double[cells * cells];
    }

    public TileChargeGrid copy() {

        TileChargeGrid g = new TileChargeGrid(2 * half, n);
        System.arraycopy(q, 0, g.q, 0, q.length);
        return g;
    }

    private int index(double x, double y) {

        int i = (int) Math.floor((x + half) / cell), j = (int) Math.floor((y + half) / cell);
        i = Math.max(0, Math.min(n - 1, i));
        j = Math.max(0, Math.min(n - 1, j));
        return j * n + i;
    }

    public void deposit(double x, double y, double dq) {
        q[index(x, y)] += dq;
    }

    public double cellSize() { return cell; }

    public void diffuse(double d, double seconds, int maxSteps) {

        if (!(d > 0) || !(seconds > 0)) return;
        double lambdaTotal = d * seconds / (cell * cell);
        int steps = (int) Math.ceil(lambdaTotal / 0.2);
        if (steps > maxSteps) {
            double mean = total() / q.length;
            java.util.Arrays.fill(q, mean);
            return;
        }
        double lam = lambdaTotal / steps;
        double[] next = new double[q.length];
        for (int s = 0; s < steps; s++) {
            for (int j = 0; j < n; j++) {
                for (int i = 0; i < n; i++) {
                    int k = j * n + i;
                    double flow = 0;
                    if (i > 0) flow += q[k - 1] - q[k];
                    if (i < n - 1) flow += q[k + 1] - q[k];
                    if (j > 0) flow += q[k - n] - q[k];
                    if (j < n - 1) flow += q[k + n] - q[k];
                    next[k] = q[k] + lam * flow;
                }
            }
            System.arraycopy(next, 0, q, 0, q.length);
        }
    }

    public double maxAbsCell() {

        double m = 0;
        for (double v : q) m = Math.max(m, Math.abs(v));
        return m;
    }

    public void decay(double factor) {
        for (int k = 0; k < q.length; k++) q[k] *= factor;
    }

    public double underFootprint(double x, double y, double h, double length, double width) {

        double c = Math.cos(h), s = Math.sin(h);
        double ex = 0.5 * length * Math.abs(c) + 0.5 * width * Math.abs(s);
        double ey = 0.5 * length * Math.abs(s) + 0.5 * width * Math.abs(c);
        int i0 = Math.max(0, (int) Math.floor((x - ex + half) / cell)), i1 = Math.min(n - 1, (int) Math.floor((x + ex + half) / cell));
        int j0 = Math.max(0, (int) Math.floor((y - ey + half) / cell)), j1 = Math.min(n - 1, (int) Math.floor((y + ey + half) / cell));
        double sum = 0;
        for (int j = j0; j <= j1; j++) {
            double cy = -half + (j + 0.5) * cell - y;
            for (int i = i0; i <= i1; i++) {
                double cx = -half + (i + 0.5) * cell - x;
                double along = c * cx + s * cy, across = -s * cx + c * cy;
                if (Math.abs(along) <= 0.5 * length && Math.abs(across) <= 0.5 * width) sum += q[j * n + i];
            }
        }
        return sum;
    }

    public double total() {

        double t = 0;
        for (double v : q) t += v;
        return t;
    }

    public double absTotal() {

        double t = 0;
        for (double v : q) t += Math.abs(v);
        return t;
    }
}
