package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Precomputed shot solutions (design D-7). Built once at init from {@link Ballistics}. Lookups are
 * allocation-free interpolation.
 *
 * <ul>
 *   <li><b>Nominal table</b> over horizontal distance d: the launch angle, exit speed and time of flight chosen
 *       to minimize the predicted vertical miss variance, (σv·∂z/∂v)² + (σθ·∂z/∂θ)², subject to hood range,
 *       maximum exit speed and a minimum descent (entry) angle. Small penalties keep the hood away from its
 *       limits (room for speed compensation) and the solution smooth across distance.</li>
 *   <li><b>Speed-compensation table</b> θ(d, v): the angle that still hits the target at a different exit speed,
 *       on the same trajectory branch as the nominal solution. This is what lets the hood correct for a
 *       flywheel speed dip instead of waiting for recovery.</li>
 * </ul>
 */
public final class ShotTable {

    public static final class Config {
        public double minDistanceM = 0.60;
        public double maxDistanceM = 4.40;
        public double distanceStepM = 0.10;
        public double angleStepDeg = 0.5;
        /** Minimum descent angle at the target opening (deg below horizontal) ❓ (opening tilted ~30°). */
        public double minEntryAngleDeg = 25;
        /**
         * Speed reserve: the nominal exit speed must be at least vMin(d)/(1 − reserve), where vMin is the
         * minimum-energy speed (the peak of height-vs-angle). At the peak, ANY speed dip makes the target
         * unreachable at every angle, so the speed-adaptive hood could not compensate. With a 10% reserve a 10%
         * dip is still reachable by re-aiming the hood. Sweep (2026-10-21): with energy banking the per-shot dip is
         * ≈ 1.6%, so 6% covers dips plus spin-up tails. Larger reserves force steeper, slower arcs.
         */
        public double speedReserve = 0.06;
        /**
         * Expected 1σ errors used to pick the most robust arc. The hood compensates using the MEASURED speed,
         * so the relevant speed error is measurement + exit-efficiency scatter (≈1%), not regulation error.
         */
        public double speedSigmaFraction = 0.01;
        public double angleSigmaDeg = 0.6;
        /** Penalties (dimensionless weights). */
        public double tofWeight = 0.0004;          // m² per s² (prefers faster shots for moving robots)
        public double smoothWeight = 0.004;        // m² per rad² change between neighbouring distances
        public double hoodEdgeMarginDeg = 3.0;     // keep this far from hood limits when possible
        public double hoodEdgeWeight = 0.002;      // m² per (rad inside margin)²
        /**
         * Speed grid for the compensation table, as fractions of the nominal speed. Kept fine (≈1%) so that
         * interpolation next to the reserve edge (where the coarser neighbour is unreachable) loses little range.
         */
        public double compMinFraction = 0.88;
        public double compMaxFraction = 1.12;
        public int compSpeedPoints = 25;
    }

    /** Mutable result (reuse one instance per caller). */
    public static final class Solution {
        public boolean valid;
        public double distance;      // m
        public double launchAngle;   // rad
        public double exitSpeed;     // m/s
        public double tof;           // s
    }

    private final Config cfg;
    private final double dz;           // target height above the exit point (m)
    private final double floorDrop;    // exit height above the floor (m)

    private final int n;
    private final double[] dist, angle, speed, tof;
    private final boolean[] valid;
    private final boolean[] increasingBranch;
    // compensation grid: angleComp[i][j] for dist[i], speed = speed[i] * frac[j]
    private final double[] frac;
    private final double[][] angleComp;
    private final double hoodMin, hoodMax;
    private final double minValidD, maxValidD;

    /**
     * @param ballistics physics model
     * @param dzM        target height minus exit height (m)
     * @param exitHeightM exit height above the floor (m)
     * @param hoodMinRad hood launch-angle range
     * @param hoodMaxRad hood launch-angle range
     * @param maxExitSpeed maximum achievable exit speed (m/s)
     */
    public ShotTable(Ballistics ballistics, Config cfg, double dzM, double exitHeightM,
                     double hoodMinRad, double hoodMaxRad, double maxExitSpeed) {
        this.cfg = cfg;
        this.dz = dzM;
        this.floorDrop = exitHeightM;
        this.hoodMin = hoodMinRad;
        this.hoodMax = hoodMaxRad;
        n = (int) Math.floor((cfg.maxDistanceM - cfg.minDistanceM) / cfg.distanceStepM + 1e-9) + 1;
        dist = new double[n];
        angle = new double[n];
        speed = new double[n];
        tof = new double[n];
        valid = new boolean[n];
        increasingBranch = new boolean[n];
        frac = new double[cfg.compSpeedPoints];
        for (int j = 0; j < frac.length; j++) {
            frac[j] = MathUtil.lerp(cfg.compMinFraction, cfg.compMaxFraction, j / (double) (frac.length - 1));
        }
        angleComp = new double[n][frac.length];

        double sigTh = Math.toRadians(cfg.angleSigmaDeg);
        double minEntry = Math.toRadians(cfg.minEntryAngleDeg);
        double margin = Math.toRadians(cfg.hoodEdgeMarginDeg);
        double step = Math.toRadians(cfg.angleStepDeg);
        Ballistics.Crossing c = new Ballistics.Crossing();
        double prevAngle = Double.NaN;
        double vMin = 0.5;
        int angleCount = (int) Math.floor((hoodMax - hoodMin) / step + 1e-9) + 1;
        double[] vOfTheta = new double[angleCount];

        for (int i = 0; i < n; i++) {
            double d = cfg.minDistanceM + i * cfg.distanceStepM;
            dist[i] = d;
            // pass 1: required speed for every hood angle → minimum-energy speed at this distance
            double vMinEnergy = Double.POSITIVE_INFINITY;
            for (int a = 0; a < angleCount; a++) {
                vOfTheta[a] = ballistics.solveSpeed(hoodMin + a * step, d, dz, vMin, maxExitSpeed, floorDrop);
                if (!Double.isNaN(vOfTheta[a])) vMinEnergy = Math.min(vMinEnergy, vOfTheta[a]);
            }
            double vRequired = vMinEnergy / (1.0 - cfg.speedReserve);
            // pass 2: pick the most robust candidate that keeps the speed reserve
            double bestCost = Double.POSITIVE_INFINITY;
            for (int a = 0; a < angleCount; a++) {
                double th = hoodMin + a * step;
                double v = vOfTheta[a];
                if (Double.isNaN(v) || v < vRequired) continue;
                ballistics.fly(v, th, d, floorDrop, c);
                if (!c.reached || c.descentAngle() < minEntry) continue;
                double flight = c.tof;
                double ev = 0.002 * v, et = Math.toRadians(0.1);
                double dzdv = (ballistics.heightAt(v + ev, th, d, floorDrop)
                        - ballistics.heightAt(v - ev, th, d, floorDrop)) / (2 * ev);
                double dzdt = (ballistics.heightAt(v, th + et, d, floorDrop)
                        - ballistics.heightAt(v, th - et, d, floorDrop)) / (2 * et);
                double sv = cfg.speedSigmaFraction * v * dzdv;
                double st = sigTh * dzdt;
                double cost = sv * sv + st * st + cfg.tofWeight * flight * flight;
                double edge = Math.max(0, margin - Math.min(th - hoodMin, hoodMax - th));
                cost += cfg.hoodEdgeWeight * edge * edge;
                if (!Double.isNaN(prevAngle)) cost += cfg.smoothWeight * (th - prevAngle) * (th - prevAngle);
                if (cost < bestCost) {
                    bestCost = cost;
                    angle[i] = th;
                    speed[i] = v;
                    tof[i] = flight;
                    increasingBranch[i] = dzdt > 0;
                    valid[i] = true;
                }
            }
            if (valid[i]) prevAngle = angle[i];
        }

        // Compensation grid on the nominal branch.
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < frac.length; j++) angleComp[i][j] = Double.NaN;
            if (!valid[i]) continue;
            for (int j = 0; j < frac.length; j++) {
                double v = speed[i] * frac[j];
                double peak = ballistics.peakAngle(v, dist[i], hoodMin, hoodMax, floorDrop);
                double lo = increasingBranch[i] ? hoodMin : peak;
                double hi = increasingBranch[i] ? peak : hoodMax;
                double th = ballistics.solveAngle(v, dist[i], dz, lo, hi, increasingBranch[i], floorDrop);
                if (!Double.isNaN(th)) {
                    ballistics.fly(v, th, dist[i], floorDrop, c);
                    if (!c.reached || c.descentAngle() < minEntry) th = Double.NaN;
                }
                angleComp[i][j] = th;
            }
        }

        double lo = Double.NaN, hi = Double.NaN;
        for (int i = 0; i < n; i++) {
            if (valid[i]) {
                if (Double.isNaN(lo)) lo = dist[i];
                hi = dist[i];
            }
        }
        minValidD = lo;
        maxValidD = hi;
    }

    // ------------------------------------------------------------------ caching (build once, load in ms)

    private static final int FORMAT_VERSION = 1;

    /** Hash of every input that affects the table. A cached table is used only if this matches exactly. */
    public static long signature(Ballistics.Params p, Config c, double dzM, double exitHeightM,
                                 double hoodMinRad, double hoodMaxRad, double maxExitSpeed) {
        double[] v = {FORMAT_VERSION, p.massKg, p.diameterM, p.dragCoefficient, p.liftCoefficient, p.airDensity,
                p.gravity, p.integrationStep, p.maxFlightTime, c.minDistanceM, c.maxDistanceM, c.distanceStepM,
                c.angleStepDeg, c.minEntryAngleDeg, c.speedReserve, c.speedSigmaFraction, c.angleSigmaDeg,
                c.tofWeight, c.smoothWeight, c.hoodEdgeMarginDeg, c.hoodEdgeWeight, c.compMinFraction,
                c.compMaxFraction, c.compSpeedPoints, dzM, exitHeightM, hoodMinRad, hoodMaxRad, maxExitSpeed};
        long s = 1125899906842597L;
        for (double x : v) s = 31 * s + Double.doubleToLongBits(x);
        return s;
    }

    public void write(java.io.DataOutputStream out, long signature) throws java.io.IOException {
        out.writeInt(FORMAT_VERSION);
        out.writeLong(signature);
        out.writeDouble(dz);
        out.writeDouble(floorDrop);
        out.writeDouble(hoodMin);
        out.writeDouble(hoodMax);
        out.writeInt(n);
        out.writeInt(frac.length);
        for (int i = 0; i < n; i++) {
            out.writeDouble(dist[i]);
            out.writeDouble(angle[i]);
            out.writeDouble(speed[i]);
            out.writeDouble(tof[i]);
            out.writeBoolean(valid[i]);
            out.writeBoolean(increasingBranch[i]);
            for (int j = 0; j < frac.length; j++) out.writeDouble(angleComp[i][j]);
        }
        for (double f : frac) out.writeDouble(f);
    }

    /** @return the cached table, or null if the file is from different inputs or a different format */
    public static ShotTable read(java.io.DataInputStream in, Config cfg, long expectedSignature) throws java.io.IOException {
        if (in.readInt() != FORMAT_VERSION || in.readLong() != expectedSignature) return null;
        double dz = in.readDouble(), floor = in.readDouble(), hMin = in.readDouble(), hMax = in.readDouble();
        int n = in.readInt(), m = in.readInt();
        if (n <= 1 || n > 10000 || m <= 1 || m > 1000) return null;
        double[] dist = new double[n], angle = new double[n], speed = new double[n], tof = new double[n];
        boolean[] valid = new boolean[n], inc = new boolean[n];
        double[][] comp = new double[n][m];
        double[] frac = new double[m];
        for (int i = 0; i < n; i++) {
            dist[i] = in.readDouble();
            angle[i] = in.readDouble();
            speed[i] = in.readDouble();
            tof[i] = in.readDouble();
            valid[i] = in.readBoolean();
            inc[i] = in.readBoolean();
            for (int j = 0; j < m; j++) comp[i][j] = in.readDouble();
        }
        for (int j = 0; j < m; j++) frac[j] = in.readDouble();
        return new ShotTable(cfg, dz, floor, hMin, hMax, dist, angle, speed, tof, valid, inc, frac, comp);
    }

    private ShotTable(Config cfg, double dz, double floorDrop, double hoodMin, double hoodMax, double[] dist,
                      double[] angle, double[] speed, double[] tof, boolean[] valid, boolean[] inc, double[] frac,
                      double[][] comp) {
        this.cfg = cfg;
        this.dz = dz;
        this.floorDrop = floorDrop;
        this.hoodMin = hoodMin;
        this.hoodMax = hoodMax;
        this.n = dist.length;
        this.dist = dist;
        this.angle = angle;
        this.speed = speed;
        this.tof = tof;
        this.valid = valid;
        this.increasingBranch = inc;
        this.frac = frac;
        this.angleComp = comp;
        double lo = Double.NaN, hi = Double.NaN;
        for (int i = 0; i < n; i++) {
            if (valid[i]) {
                if (Double.isNaN(lo)) lo = dist[i];
                hi = dist[i];
            }
        }
        minValidD = lo;
        maxValidD = hi;
    }

    public boolean hasAnySolution() { return !Double.isNaN(minValidD); }
    public double minValidDistance() { return minValidD; }
    public double maxValidDistance() { return maxValidD; }

    /** Nominal solution at distance d (m). Invalid outside the solvable range. */
    public Solution nominal(double d, Solution out) {
        out.distance = d;
        out.valid = false;
        if (!MathUtil.isFinite(d) || Double.isNaN(minValidD) || d < minValidD || d > maxValidD) return out;
        double fi = (d - cfg.minDistanceM) / cfg.distanceStepM;
        int i = (int) Math.floor(fi);
        if (i >= n - 1) i = n - 2;
        if (i < 0) i = 0;
        double t = MathUtil.clamp(fi - i, 0, 1);
        if (!valid[i] || !valid[i + 1]) {
            int k = t < 0.5 ? i : i + 1;
            if (!valid[k]) return out;
            out.launchAngle = angle[k];
            out.exitSpeed = speed[k];
            out.tof = tof[k];
        } else {
            out.launchAngle = MathUtil.lerp(angle[i], angle[i + 1], t);
            out.exitSpeed = MathUtil.lerp(speed[i], speed[i + 1], t);
            out.tof = MathUtil.lerp(tof[i], tof[i + 1], t);
        }
        out.valid = true;
        return out;
    }

    /**
     * Launch angle that hits the target at distance d with the actual exit speed v (speed-adaptive hood).
     *
     * @return angle (rad) or NaN if v is outside the compensable range at this distance
     */
    public double compensatedAngle(double d, double v) {
        if (!MathUtil.isFinite(d) || !MathUtil.isFinite(v) || Double.isNaN(minValidD)
                || d < minValidD || d > maxValidD) return Double.NaN;
        double fi = (d - cfg.minDistanceM) / cfg.distanceStepM;
        int i = (int) Math.floor(fi);
        if (i >= n - 1) i = n - 2;
        if (i < 0) i = 0;
        double t = MathUtil.clamp(fi - i, 0, 1);
        double a0 = angleAtSpeed(i, v);
        double a1 = angleAtSpeed(i + 1, v);
        if (Double.isNaN(a0)) return t > 0.5 ? a1 : Double.NaN;
        if (Double.isNaN(a1)) return t < 0.5 ? a0 : Double.NaN;
        return MathUtil.lerp(a0, a1, t);
    }

    private double angleAtSpeed(int i, double v) {
        if (!valid[i]) return Double.NaN;
        double f = v / speed[i];
        if (f < frac[0] || f > frac[frac.length - 1]) return Double.NaN;
        double fj = (f - frac[0]) / (frac[1] - frac[0]);
        int j = (int) Math.floor(fj);
        if (j >= frac.length - 1) j = frac.length - 2;
        double t = fj - j;
        double a0 = angleComp[i][j], a1 = angleComp[i][j + 1];
        if (Double.isNaN(a0) || Double.isNaN(a1)) return Double.NaN;
        return MathUtil.lerp(a0, a1, t);
    }

    public double targetHeightAboveExit() { return dz; }
    public int size() { return n; }
    public double distanceAt(int i) { return dist[i]; }
    public boolean validAt(int i) { return valid[i]; }
    public double angleAt(int i) { return angle[i]; }
    public double speedAt(int i) { return speed[i]; }
    public double tofAt(int i) { return tof[i]; }
}
