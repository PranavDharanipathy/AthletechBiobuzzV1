package org.athletech.sim.predict;

import org.athletech.sim.constants.Physics;
import org.athletech.sim.field.Aperture;
import org.athletech.sim.physics.Projectile;
import org.athletech.sim.robot.Launcher;
import org.athletech.sim.utility.Stats;
import org.athletech.sim.utility.Vec2;
import org.athletech.sim.utility.Vec3;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import java.util.stream.IntStream;

/**
 * Finds the hood angle and flywheel speed for a shot. For every hood angle in range it finds the
 * band of exit speeds that score and, at the middle of that band, the band of launch angles that
 * score. Each candidate is scored by its chance of scoring when speed and angle have normal errors
 * of the given sizes.
 *
 * <p>{@link #solve} picks the best candidate at one distance. {@link #solveTable} builds a whole
 * shooting table: it picks hood angles across all distances together, penalising hood jumps, so
 * neighbouring distances stay on the same arc. Where the arc must switch (the high arc runs out of
 * reach, say) it adds a pair of points 1 mm apart at the switch, so linear interpolation of the
 * table never blends a high-arc shot with a low-arc one.
 */
public final class AimSolver {
    private AimSolver() {
    }

    /** Default error sizes when the profile has no distributions yet. */
    public static final double DEFAULT_SPEED_SIGMA_FRAC = 0.02, DEFAULT_ANGLE_SIGMA_DEG = 1.0;
    /** Hood changes larger than this between table neighbours count as an arc switch. */
    public static final double ARC_JUMP_DEG = 6;
    private static final int SPEED_STEPS = 160;

    /** Where the robot is relative to the target. */
    public static final class Spot {
        private final double robotX;
        private final double robotY;
        private final double robotHeading;
        private final Vec3 launchPoint;
        private final double yawToTarget;

        public Spot(double robotX, double robotY, double robotHeading, Vec3 launchPoint, double yawToTarget) {
            this.robotX = robotX;
            this.robotY = robotY;
            this.robotHeading = robotHeading;
            this.launchPoint = launchPoint;
            this.yawToTarget = yawToTarget;
        }

        public double robotX() { return robotX; }
        public double robotY() { return robotY; }
        public double robotHeading() { return robotHeading; }
        public Vec3 launchPoint() { return launchPoint; }
        public double yawToTarget() { return yawToTarget; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Spot)) return false;
            Spot o = (Spot) other;
            return Double.compare(robotX, o.robotX) == 0
                    && Double.compare(robotY, o.robotY) == 0
                    && Double.compare(robotHeading, o.robotHeading) == 0
                    && java.util.Objects.equals(launchPoint, o.launchPoint)
                    && Double.compare(yawToTarget, o.yawToTarget) == 0;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(robotX, robotY, robotHeading, launchPoint, yawToTarget);
        }

        @Override
        public String toString() {
            return "Spot[robotX=" + robotX + ", robotY=" + robotY + ", robotHeading=" + robotHeading + ", launchPoint=" + launchPoint + ", yawToTarget=" + yawToTarget + "]";
        }
    }

    public static final class Solution {
        private final boolean reachable;
        private final double distanceM;
        private final double hoodDeg;
        private final double exitSpeed;
        private final double rpm;
        private final double speedLow;
        private final double speedHigh;
        private final double angleLowDeg;
        private final double angleHighDeg;
        private final double score;
        private final Projectile.Result nominal;

        public Solution(boolean reachable, double distanceM, double hoodDeg, double exitSpeed, double rpm, double speedLow, double speedHigh, double angleLowDeg, double angleHighDeg, double score, Projectile.Result nominal) {
            this.reachable = reachable;
            this.distanceM = distanceM;
            this.hoodDeg = hoodDeg;
            this.exitSpeed = exitSpeed;
            this.rpm = rpm;
            this.speedLow = speedLow;
            this.speedHigh = speedHigh;
            this.angleLowDeg = angleLowDeg;
            this.angleHighDeg = angleHighDeg;
            this.score = score;
            this.nominal = nominal;
        }

        public boolean reachable() { return reachable; }
        public double distanceM() { return distanceM; }
        public double hoodDeg() { return hoodDeg; }
        public double exitSpeed() { return exitSpeed; }
        public double rpm() { return rpm; }
        public double speedLow() { return speedLow; }
        public double speedHigh() { return speedHigh; }
        public double angleLowDeg() { return angleLowDeg; }
        public double angleHighDeg() { return angleHighDeg; }
        public double score() { return score; }
        public Projectile.Result nominal() { return nominal; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Solution)) return false;
            Solution o = (Solution) other;
            return reachable == o.reachable
                    && Double.compare(distanceM, o.distanceM) == 0
                    && Double.compare(hoodDeg, o.hoodDeg) == 0
                    && Double.compare(exitSpeed, o.exitSpeed) == 0
                    && Double.compare(rpm, o.rpm) == 0
                    && Double.compare(speedLow, o.speedLow) == 0
                    && Double.compare(speedHigh, o.speedHigh) == 0
                    && Double.compare(angleLowDeg, o.angleLowDeg) == 0
                    && Double.compare(angleHighDeg, o.angleHighDeg) == 0
                    && Double.compare(score, o.score) == 0
                    && java.util.Objects.equals(nominal, o.nominal);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(reachable, distanceM, hoodDeg, exitSpeed, rpm, speedLow, speedHigh, angleLowDeg, angleHighDeg, score, nominal);
        }

        @Override
        public String toString() {
            return "Solution[reachable=" + reachable + ", distanceM=" + distanceM + ", hoodDeg=" + hoodDeg + ", exitSpeed=" + exitSpeed + ", rpm=" + rpm + ", speedLow=" + speedLow + ", speedHigh=" + speedHigh + ", angleLowDeg=" + angleLowDeg + ", angleHighDeg=" + angleHighDeg + ", score=" + score + ", nominal=" + nominal + "]";
        }

        /** Fractional speed tolerance of the chosen shot (band width / exit speed). */
        public double tolerance() {
            return reachable ? (speedHigh - speedLow) / exitSpeed : 0;
        }

        public static Solution unreachable(double d) {
            return new Solution(false, d, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, 0, null);
        }
    
    }

    /** One hood angle's best shot at one distance. */
    static final class Candidate {
        private final double hood;
        private final double vLo;
        private final double vHi;
        private final double aLo;
        private final double aHi;
        private final double score;

        Candidate(double hood, double vLo, double vHi, double aLo, double aHi, double score) {
            this.hood = hood;
            this.vLo = vLo;
            this.vHi = vHi;
            this.aLo = aLo;
            this.aHi = aHi;
            this.score = score;
        }

        public double hood() { return hood; }
        public double vLo() { return vLo; }
        public double vHi() { return vHi; }
        public double aLo() { return aLo; }
        public double aHi() { return aHi; }
        public double score() { return score; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Candidate)) return false;
            Candidate o = (Candidate) other;
            return Double.compare(hood, o.hood) == 0
                    && Double.compare(vLo, o.vLo) == 0
                    && Double.compare(vHi, o.vHi) == 0
                    && Double.compare(aLo, o.aLo) == 0
                    && Double.compare(aHi, o.aHi) == 0
                    && Double.compare(score, o.score) == 0;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(hood, vLo, vHi, aLo, aHi, score);
        }

        @Override
        public String toString() {
            return "Candidate[hood=" + hood + ", vLo=" + vLo + ", vHi=" + vHi + ", aLo=" + aLo + ", aHi=" + aHi + ", score=" + score + "]";
        }

        double v() { return 0.5 * (vLo + vHi); }
    
    }

    /** A shooting table: one solution per distance plus extra points where the arc switches. */
    public static final class Table {
        private final Solution[] solutions;
        private final List<Solution> switchPoints;

        public Table(Solution[] solutions, List<Solution> switchPoints) {
            this.solutions = solutions;
            this.switchPoints = switchPoints;
        }

        public Solution[] solutions() { return solutions; }
        public List<Solution> switchPoints() { return switchPoints; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Table)) return false;
            Table o = (Table) other;
            return java.util.Objects.equals(solutions, o.solutions)
                    && java.util.Objects.equals(switchPoints, o.switchPoints);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(solutions, switchPoints);
        }

        @Override
        public String toString() {
            return "Table[solutions=" + solutions + ", switchPoints=" + switchPoints + "]";
        }

        /** Reachable solutions and switch points, sorted by distance (what goes into the lookup curves). */
        public List<Solution> lookup() {
            List<Solution> out = new ArrayList<>(switchPoints);
            for (Solution s : solutions) if (s.reachable()) out.add(s);
            out.sort((a, b) -> Double.compare(a.distanceM(), b.distanceM()));
            return out;
        }
    
    }

    /**
     * Robot position for a shot from {@code distanceM} (horizontal, to the opening center) at
     * {@code bearingRad} from the direction the opening faces, robot facing the target.
     */
    public static Spot spot(Launcher.Params lp, Aperture target, double distanceM, double bearingRad) {
        Vec3 n = target.normal();
        double facing = Math.hypot(n.x(), n.y()) > 1e-6 ? Math.atan2(n.y(), n.x())
                : Math.atan2(-target.heightAxis().y(), -target.heightAxis().x());
        double dir = facing + bearingRad;
        double x = target.center().x() + distanceM * Math.cos(dir), y = target.center().y() + distanceM * Math.sin(dir);
        double heading = Math.atan2(target.center().y() - y, target.center().x() - x);
        Vec2 off = new Vec2(lp.offsetForwardM(), lp.offsetLeftM()).rotate(heading);
        Vec3 launch = new Vec3(x + off.x(), y + off.y(), lp.exitHeightM());
        double yaw = Math.atan2(target.center().y() - launch.y(), target.center().x() - launch.x());
        return new Spot(x, y, heading, launch, yaw);
    }

    public static Solution solve(Launcher launcher, String ball, Aperture target, double distanceM, double bearingRad) {
        return solve(launcher, ball, target, distanceM, bearingRad, v -> DEFAULT_SPEED_SIGMA_FRAC * v, DEFAULT_ANGLE_SIGMA_DEG);
    }

    /**
     * Best shot at one distance.
     *
     * @param speedSigma    1-sigma exit speed error (m/s) for a shot at exit speed v (m/s)
     * @param angleSigmaDeg 1-sigma launch angle error, degrees
     */
    public static Solution solve(Launcher launcher, String ball, Aperture target, double distanceM, double bearingRad,
                                 DoubleUnaryOperator speedSigma, double angleSigmaDeg) {
        Launcher.Params lp = launcher.params();
        Candidate[] cs = candidates(launcher, ball, target, distanceM, bearingRad, speedSigma, angleSigmaDeg,
                lp.hoodMinDeg(), Math.max(lp.hoodMinDeg(), lp.hoodMaxDeg()));
        Candidate best = null;
        for (Candidate c : cs) if (c != null && (best == null || c.score() > best.score())) best = c;
        return toSolution(launcher, ball, target, distanceM, bearingRad, best);
    }

    /** Shooting table over {@code distances} (ascending), arc-consistent; see the class comment. */
    public static Table solveTable(Launcher launcher, String ball, Aperture target, double[] distances, double bearingRad,
                                   DoubleUnaryOperator speedSigma, double angleSigmaDeg) {
        Launcher.Params lp = launcher.params();
        double hMin = lp.hoodMinDeg(), hMax = Math.max(lp.hoodMinDeg(), lp.hoodMaxDeg());
        Candidate[][] all = new Candidate[distances.length][];
        IntStream.range(0, distances.length).parallel().forEach(i ->
                all[i] = candidates(launcher, ball, target, distances[i], bearingRad, speedSigma, angleSigmaDeg, hMin, hMax));
        int h = all.length == 0 ? 0 : all[0].length;
        int[] pick = new int[distances.length];
        java.util.Arrays.fill(pick, -1);
        // dynamic programming over runs of reachable distances: maximise sum(log score) - hood-jump penalty
        int i = 0;
        while (i < distances.length) {
            if (!anyCandidate(all[i])) {
                i++;
                continue;
            }
            int start = i;
            while (i < distances.length && anyCandidate(all[i])) i++;
            int end = i; // exclusive
            double[][] best = new double[end - start][h];
            int[][] from = new int[end - start][h];
            for (int k = 0; k < h; k++) best[0][k] = all[start][k] == null ? Double.NEGATIVE_INFINITY : Math.log(all[start][k].score());
            for (int d = 1; d < end - start; d++) {
                for (int k = 0; k < h; k++) {
                    best[d][k] = Double.NEGATIVE_INFINITY;
                    Candidate c = all[start + d][k];
                    if (c == null) continue;
                    for (int j = 0; j < h; j++) {
                        if (best[d - 1][j] == Double.NEGATIVE_INFINITY) continue;
                        double jump = Math.abs(all[start + d - 1][j].hood() - c.hood());
                        double v = best[d - 1][j] - 0.01 * jump - 0.4 * Math.max(0, jump - ARC_JUMP_DEG) + Math.log(c.score());
                        if (v > best[d][k]) {
                            best[d][k] = v;
                            from[d][k] = j;
                        }
                    }
                }
            }
            int last = end - start - 1, k = 0;
            for (int j = 1; j < h; j++) if (best[last][j] > best[last][k]) k = j;
            for (int d = last; d >= 0; d--) {
                pick[start + d] = k;
                k = from[d][k];
            }
        }
        Solution[] sols = new Solution[distances.length];
        for (int d = 0; d < distances.length; d++) {
            sols[d] = toSolution(launcher, ball, target, distances[d], bearingRad, pick[d] < 0 ? null : all[d][pick[d]]);
        }
        // arc switches: solve both arcs at the midpoint and put them 1 mm apart
        List<Solution> switches = new ArrayList<>();
        for (int d = 1; d < distances.length; d++) {
            Solution a = sols[d - 1], b = sols[d];
            if (!a.reachable() || !b.reachable() || Math.abs(a.hoodDeg() - b.hoodDeg()) <= ARC_JUMP_DEG) continue;
            double mid = 0.5 * (a.distanceM() + b.distanceM());
            Solution left = bestNear(launcher, ball, target, mid, bearingRad, speedSigma, angleSigmaDeg, a.hoodDeg(), hMin, hMax);
            Solution right = bestNear(launcher, ball, target, mid, bearingRad, speedSigma, angleSigmaDeg, b.hoodDeg(), hMin, hMax);
            if (left.reachable() && right.reachable()) {
                switches.add(at(left, mid - 0.0005));
                switches.add(at(right, mid + 0.0005));
            }
        }
        return new Table(sols, switches);
    }

    private static boolean anyCandidate(Candidate[] cs) {
        for (Candidate c : cs) if (c != null) return true;
        return false;
    }

    private static Solution bestNear(Launcher l, String ball, Aperture target, double d, double bearing, DoubleUnaryOperator sv,
                                     double sa, double hood, double hMin, double hMax) {
        Candidate[] cs = candidates(l, ball, target, d, bearing, sv, sa, Math.max(hMin, hood - ARC_JUMP_DEG / 2), Math.min(hMax, hood + ARC_JUMP_DEG / 2));
        Candidate best = null;
        for (Candidate c : cs) if (c != null && (best == null || c.score() > best.score())) best = c;
        return toSolution(l, ball, target, d, bearing, best);
    }

    private static Solution at(Solution s, double d) {
        return new Solution(s.reachable(), d, s.hoodDeg(), s.exitSpeed(), s.rpm(), s.speedLow(), s.speedHigh(),
                s.angleLowDeg(), s.angleHighDeg(), s.score(), s.nominal());
    }

    private static Solution toSolution(Launcher launcher, String ball, Aperture target, double d, double bearing, Candidate c) {
        if (c == null) return Solution.unreachable(d);
        double v = c.v();
        double rpm = launcher.rpmForExitSpeed(ball, v);
        Projectile.Result nominal = fly(launcher.params().ball(ball).ball(), spot(launcher.params(), target, d, bearing), Physics.rad(c.hood()), v, target);
        return new Solution(Double.isFinite(rpm), d, c.hood(), v, rpm, c.vLo(), c.vHi(), c.aLo(), c.aHi(), c.score(), nominal);
    }

    /** Scored candidates for each 1-degree hood step in [hoodLo, hoodHi]; null where nothing scores. */
    static Candidate[] candidates(Launcher launcher, String ball, Aperture target, double distanceM, double bearingRad,
                                  DoubleUnaryOperator speedSigma, double angleSigmaDeg, double hoodLo, double hoodHi) {
        Launcher.Params lp = launcher.params();
        Projectile.Ball b = lp.ball(ball).ball();
        Spot s = spot(lp, target, distanceM, bearingRad);
        double vMax = launcher.maxExitSpeed(ball);
        int hoodSteps = Math.max(1, (int) Math.ceil(hoodHi - hoodLo));
        Candidate[] out = new Candidate[hoodSteps + 1];
        if (!(vMax > 0.5)) return out;
        double sa = angleSigmaDeg > 0 ? angleSigmaDeg : DEFAULT_ANGLE_SIGMA_DEG;
        double step = vMax / SPEED_STEPS;
        for (int hi = 0; hi <= hoodSteps; hi++) {
            double hood = hoodLo + (hoodHi - hoodLo) * hi / hoodSteps;
            double el = Physics.rad(hood);
            double[] band = widestBand(k -> fly(b, s, el, step * k, target).made(), SPEED_STEPS);
            if (band == null) continue;
            double lo = edge(v -> fly(b, s, el, v, target).made(), band[0] * step - step, band[0] * step);
            // never extrapolate past the fastest exit speed the launcher has
            double hiV = band[1] >= SPEED_STEPS ? vMax : edge(v -> fly(b, s, el, v, target).made(), band[1] * step + step, band[1] * step);
            double v = 0.5 * (lo + hiV);
            double aHi = angleReach(b, s, target, v, hood, +1), aLo = angleReach(b, s, target, v, hood, -1);
            double sigV = speedSigma.applyAsDouble(v);
            if (!(sigV > 0)) sigV = DEFAULT_SPEED_SIGMA_FRAC * v;
            double pv = 2 * Stats.normalCdf((hiV - lo) / 2 / sigV) - 1;
            double pa = Stats.normalCdf(aHi / sa) - Stats.normalCdf(-aLo / sa);
            double score = pv * pa;
            if (score > 1e-9) out[hi] = new Candidate(hood, lo, hiV, aLo, aHi, score);
        }
        return out;
    }

    /** Index range [first, last] of the longest run of "made" over k = 1..steps, or null. */
    private static double[] widestBand(java.util.function.IntPredicate made, int steps) {
        int bestLo = -1, bestHi = -1, cur = -1;
        for (int k = 1; k <= steps + 1; k++) {
            boolean m = k <= steps && made.test(k);
            if (m && cur < 0) cur = k;
            if (!m && cur >= 0) {
                if (bestLo < 0 || (k - 1) - cur > bestHi - bestLo) {
                    bestLo = cur;
                    bestHi = k - 1;
                }
                cur = -1;
            }
        }
        return bestLo < 0 ? null : new double[]{bestLo, bestHi};
    }

    /** Bisection between a missing value and a scoring value: returns the boundary. */
    private static double edge(java.util.function.DoublePredicate made, double miss, double make) {
        if (miss <= 0) return make;
        for (int i = 0; i < 25; i++) {
            double m = 0.5 * (miss + make);
            if (made.test(m)) make = m;
            else miss = m;
        }
        return make;
    }

    /** How many degrees the launch angle can move in {@code dir} (+1 up, -1 down) and still score. */
    private static double angleReach(Projectile.Ball b, Spot s, Aperture target, double v, double hood, int dir) {
        double step = 0.25, d = 0;
        while (d < 15 && fly(b, s, Physics.rad(hood + dir * (d + step)), v, target).made()) d += step;
        double lo = d, hi = d + step;
        for (int i = 0; i < 12; i++) {
            double m = 0.5 * (lo + hi);
            if (fly(b, s, Physics.rad(hood + dir * m), v, target).made()) lo = m;
            else hi = m;
        }
        return lo;
    }

    static Projectile.Result fly(Projectile.Ball b, Spot s, double elevationRad, double speed, Aperture target) {
        Vec3 v = Launcher.launchVelocity(s.yawToTarget(), elevationRad, speed, Vec2.ZERO);
        return Projectile.fly(b, s.launchPoint(), v, target);
    }
}
