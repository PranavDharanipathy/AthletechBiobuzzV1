package org.athletech.sim.undeterminables;

/**
 * Charge left on the foam tiles' surface, on a grid over the field (origin at the field center).
 * Foam is an insulator, so charge stays near where the wheels left it: it leaks slowly through the
 * foam to the floor ({@link #decay}) and creeps sideways over the surface ({@link #diffuse}).
 *
 * <p>Sideways creep: a charged sheet over a grounded floor with surface resistivity rho_s (ohm per
 * square) and capacitance per area c = eps0 * eps_r / thickness spreads like heat, with diffusion
 * coefficient D = 1 / (rho_s * c). Solved with conservative five-point steps (no flow through the
 * field walls), so the total charge is exactly kept.
 */
public final class TileChargeGrid {
    private final int n;
    private final double half, cell;
    private final double[] q;

    /** @param fieldSizeM inside of the perimeter; @param cells grid cells per side */
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

    /** Cell holding field point (x, y); points outside the walls go to the nearest edge cell. */
    private int index(double x, double y) {
        int i = (int) Math.floor((x + half) / cell), j = (int) Math.floor((y + half) / cell);
        i = Math.max(0, Math.min(n - 1, i));
        j = Math.max(0, Math.min(n - 1, j));
        return j * n + i;
    }

    /** Adds charge {@code dq} (C) at field point (x, y). */
    public void deposit(double x, double y, double dq) {
        q[index(x, y)] += dq;
    }

    public double cellSize() { return cell; }

    /**
     * Lets surface charge creep sideways for {@code seconds} with diffusion coefficient {@code d}
     * (m^2/s). Explicit steps sized for stability (d * dt / cell^2 <= 0.2); if more than
     * {@code maxSteps} would be needed the surface is so conductive over this time that the charge
     * is simply spread evenly.
     */
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

    /** Largest charge (C) in any one cell. */
    public double maxAbsCell() {
        double m = 0;
        for (double v : q) m = Math.max(m, Math.abs(v));
        return m;
    }

    /** Multiplies every cell by {@code factor} (leakage over some time). */
    public void decay(double factor) {
        for (int k = 0; k < q.length; k++) q[k] *= factor;
    }

    /** Charge (C) in cells whose centers lie under a robot footprint centered at (x, y), heading h. */
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
