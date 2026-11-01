package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Ball-flight physics in the vertical plane of the shot (design D-7):
 * gravity + quadratic drag + Magnus lift from backspin, integrated with RK4.
 *
 * <p>Frame: x = horizontal distance from the exit point toward the target, z = up, both in metres, origin at
 * the ball exit point. Used at init to build lookup tables ({@link ShotTable}), never in the control loop.
 * Not thread-safe (reuses internal buffers to avoid allocations).
 */
public final class Ballistics {

    public static final class Params {
        public double massKg = 0.050;        // POLLEN mass ❓ (weigh one)
        public double diameterM = 0.071;     // 2.8 in (manual §9.8)
        public double dragCoefficient = 0.50; // sphere, Re ~ 4e4 ❓
        public double liftCoefficient = 0.12; // Magnus from backspin ❓ (fit from shots)
        public double airDensity = 1.20;     // kg/m³ (sea level, 20 °C)
        public double gravity = 9.80665;
        public double integrationStep = 0.008; // s (RK4 error ∝ h⁴: ≈2e-4 m at 8 ms, verified by test)
        public double maxFlightTime = 3.0;   // s
    }

    /** Result of one trajectory evaluation (reused, not allocated per call). */
    public static final class Crossing {
        public boolean reached;   // true if the ball reached the target distance
        public double z;          // height at the target distance (m, relative to the exit point)
        public double tof;        // time of flight to the target distance (s)
        public double vx, vz;     // velocity at the target distance (m/s)
        /** Descent angle below horizontal at the target (rad; positive = falling). */
        public double descentAngle() { return Math.atan2(-vz, vx); }
    }

    private final Params p;
    private final double k; // ½ρA/m
    private final Crossing scratch = new Crossing();

    public Ballistics(Params p) {
        this.p = p;
        double area = Math.PI * 0.25 * p.diameterM * p.diameterM;
        this.k = 0.5 * p.airDensity * area / p.massKg;
    }

    public Params params() { return p; }

    // state derivative buffers
    private double ax, az;

    private void accel(double vx, double vz) {
        double s = Math.sqrt(vx * vx + vz * vz);
        double kd = k * p.dragCoefficient * s;
        double kl = k * p.liftCoefficient * s;
        ax = -kd * vx - kl * vz;
        az = -p.gravity - kd * vz + kl * vx;
    }

    /**
     * Flies the ball from the exit point with speed v at elevation theta and reports the state when it
     * crosses horizontal distance dx. Stops (reached = false) if the ball falls more than floorDrop below the
     * exit point or the flight time limit passes.
     */
    public Crossing fly(double v, double theta, double dx, double floorDrop, Crossing out) {
        double x = 0, z = 0, vx = v * Math.cos(theta), vz = v * Math.sin(theta), t = 0;
        double h = p.integrationStep;
        out.reached = false;
        while (t < p.maxFlightTime) {
            // RK4 on (x, z, vx, vz)
            accel(vx, vz);
            double k1x = vx, k1z = vz, k1vx = ax, k1vz = az;
            accel(vx + 0.5 * h * k1vx, vz + 0.5 * h * k1vz);
            double k2x = vx + 0.5 * h * k1vx, k2z = vz + 0.5 * h * k1vz, k2vx = ax, k2vz = az;
            accel(vx + 0.5 * h * k2vx, vz + 0.5 * h * k2vz);
            double k3x = vx + 0.5 * h * k2vx, k3z = vz + 0.5 * h * k2vz, k3vx = ax, k3vz = az;
            accel(vx + h * k3vx, vz + h * k3vz);
            double k4x = vx + h * k3vx, k4z = vz + h * k3vz, k4vx = ax, k4vz = az;

            double nx = x + h / 6 * (k1x + 2 * k2x + 2 * k3x + k4x);
            double nz = z + h / 6 * (k1z + 2 * k2z + 2 * k3z + k4z);
            double nvx = vx + h / 6 * (k1vx + 2 * k2vx + 2 * k3vx + k4vx);
            double nvz = vz + h / 6 * (k1vz + 2 * k2vz + 2 * k3vz + k4vz);

            if (nx >= dx) {
                double f = (dx - x) / (nx - x);
                out.reached = true;
                out.z = z + f * (nz - z);
                out.vx = vx + f * (nvx - vx);
                out.vz = vz + f * (nvz - vz);
                out.tof = t + f * h;
                return out;
            }
            if (nvx <= 0 || nz < -floorDrop) return out; // stalled or hit the floor
            x = nx; z = nz; vx = nvx; vz = nvz; t += h;
        }
        return out;
    }

    /** Height at dx (m), or −∞ if the ball never gets there. */
    public double heightAt(double v, double theta, double dx, double floorDrop) {
        fly(v, theta, dx, floorDrop, scratch);
        return scratch.reached ? scratch.z : Double.NEGATIVE_INFINITY;
    }

    /**
     * Exit speed that makes the ball pass through (dx, dz) at elevation theta. Height at dx increases
     * monotonically with speed for a fixed angle, so bisection is guaranteed to converge.
     *
     * @return speed in m/s, or NaN if not achievable within [vMin, vMax]
     */
    public double solveSpeed(double theta, double dx, double dz, double vMin, double vMax, double floorDrop) {
        if (heightAt(vMax, theta, dx, floorDrop) < dz) return Double.NaN;
        if (heightAt(vMin, theta, dx, floorDrop) > dz) return Double.NaN; // would fly over even at vMin
        double lo = vMin, hi = vMax;
        for (int i = 0; i < 22; i++) {
            double mid = 0.5 * (lo + hi);
            if (heightAt(mid, theta, dx, floorDrop) < dz) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    /**
     * Angle in [thLo, thHi] that hits (dx, dz) at speed v, on the branch where height is monotonic in angle
     * (increasing if {@code increasing}). The caller supplies a bracket on one side of the height peak.
     *
     * @return angle in rad, or NaN if no solution in the bracket
     */
    public double solveAngle(double v, double dx, double dz, double thLo, double thHi, boolean increasing,
                             double floorDrop) {
        double zLo = heightAt(v, thLo, dx, floorDrop) - dz;
        double zHi = heightAt(v, thHi, dx, floorDrop) - dz;
        if (increasing ? (zLo > 0 || zHi < 0) : (zLo < 0 || zHi > 0)) return Double.NaN;
        double lo = thLo, hi = thHi;
        for (int i = 0; i < 22; i++) {
            double mid = 0.5 * (lo + hi);
            double zm = heightAt(v, mid, dx, floorDrop) - dz;
            boolean below = zm < 0;
            if (below == increasing) lo = mid; else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    /** Angle in [thLo, thHi] maximizing the height at dx for speed v (golden-section search; unimodal). */
    public double peakAngle(double v, double dx, double thLo, double thHi, double floorDrop) {
        final double g = 0.6180339887498949;
        double a = thLo, b = thHi;
        double c = b - g * (b - a), d = a + g * (b - a);
        double fc = heightAt(v, c, dx, floorDrop), fd = heightAt(v, d, dx, floorDrop);
        for (int i = 0; i < 30; i++) {
            if (fc > fd) { b = d; d = c; fd = fc; c = b - g * (b - a); fc = heightAt(v, c, dx, floorDrop); }
            else { a = c; c = d; fc = fd; d = a + g * (b - a); fd = heightAt(v, d, dx, floorDrop); }
        }
        return 0.5 * (a + b);
    }
}
