package org.firstinspires.ftc.teamcode.shooter.core;

/**
 * Turns robot state + target into turret / flywheel / hood set-points, including shooting on the move and
 * latency compensation (design D-8). Allocation-free; reuse one {@link Aim} instance.
 *
 * <pre>
 *  1. predict the robot pose ahead by the system latency (constant velocity and yaw rate)
 *  2. turret axis = pose + R(h)·offset ; launcher velocity = v + ω × offset
 *  3. virtual target = target − v_launcher · tof(d)   (fixed-point iteration, tof from the shot table)
 *  4. turret angle = bearing(virtual target) − heading (+ lateral exit-offset parallax correction)
 *     turret rate  = bearing rate − yaw rate  (velocity feedforward for the turret)
 *  5. lateral tolerance = atan((opening half-width − ball radius) / distance) × safety
 * </pre>
 */
public final class AimSolver {

    public static final class Config {
        /** Turret axis position in the robot frame (in): forward, left. CAD: 2.82 in behind centre. */
        public double turretForward = -2.82;
        public double turretLeft = 0.0;
        /** Ball exit point relative to the turret axis along the shot direction (in) ❓. */
        public double exitForward = 0.0;
        /** Ball exit point lateral offset (left of the shot line, in) ❓ CAD suggests |e| < 1 in. */
        public double exitLateral = 0.0;
        /** Ball exit height above the floor (in) ❓. CAD: axle 9.6 in + wheel radius + ball radius ≈ 12.4. */
        public double exitHeight = 12.4;
        /** Pose age + compute + actuation latency (s) ❓ measured. */
        public double latency = 0.06;
        public int iterations = 3;
        public double ballRadius = 1.4;           // in
        public double toleranceSafety = 0.6;      // fraction of the geometric tolerance we allow
    }

    /** Aim output. */
    public static final class Aim {
        public boolean valid;
        public double turretAngle;      // rad, robot frame
        public double turretRate;       // rad/s, robot frame
        public double distanceM;        // horizontal exit→(virtual) target distance, m
        public double tolerance;        // allowed |turret error| (rad)
        public final ShotTable.Solution shot = new ShotTable.Solution();
    }

    private final Config cfg;

    public AimSolver(Config cfg) {
        this.cfg = cfg;
    }

    public Config config() { return cfg; }

    public Aim solve(RobotState s, HiveTarget target, ShotTable table, double now, Aim out) {
        out.valid = false;
        if (!s.isFinite()) return out;

        // 1) latency-compensated pose (state age + fixed latency)
        double lat = cfg.latency + Math.max(0, now - s.timestamp);
        double h = s.heading + s.omega * lat;
        double px = s.x + s.vx * lat;
        double py = s.y + s.vy * lat;

        // 2) turret axis and launcher velocity in the field frame
        double c = Math.cos(h), sn = Math.sin(h);
        double rx = c * cfg.turretForward - sn * cfg.turretLeft;
        double ry = sn * cfg.turretForward + c * cfg.turretLeft;
        double tx = px + rx, ty = py + ry;
        double lvx = s.vx - s.omega * ry;
        double lvy = s.vy + s.omega * rx;

        // 3) virtual-target iteration
        double gx = target.x(), gy = target.y();
        double tof = 0;
        double dx = gx - tx, dy = gy - ty;
        double dExitM = Double.NaN;
        for (int i = 0; i <= cfg.iterations; i++) {
            dx = gx - lvx * tof - tx;
            dy = gy - lvy * tof - ty;
            double dIn = Math.hypot(dx, dy) - cfg.exitForward;
            dExitM = dIn * MathUtil.IN_TO_M;
            table.nominal(dExitM, out.shot);
            if (!out.shot.valid) return out;
            tof = out.shot.tof;
        }

        // 4) turret angle and rate
        double d2 = dx * dx + dy * dy;
        double d = Math.sqrt(d2);
        if (d < 1e-6) return out;
        double bearing = Math.atan2(dy, dx);
        double parallax = -Math.asin(MathUtil.clamp(cfg.exitLateral / d, -1, 1));
        out.turretAngle = MathUtil.wrapPi(bearing + parallax - h);
        double bearingRate = (dx * (-lvy) - dy * (-lvx)) / d2;
        out.turretRate = bearingRate - s.omega;
        out.distanceM = dExitM;

        // 5) tolerance from the opening geometry
        double half = Math.max(0.1, target.halfWidth() - cfg.ballRadius);
        out.tolerance = Math.atan(half / d) * cfg.toleranceSafety;
        out.valid = true;
        return out;
    }
}
