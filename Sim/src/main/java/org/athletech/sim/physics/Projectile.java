package org.athletech.sim.physics;

import org.athletech.sim.constants.Physics;
import org.athletech.sim.field.Aperture;
import org.athletech.sim.utility.Vec2;
import org.athletech.sim.utility.Vec3;

public final class Projectile {
    private Projectile() {
    }

    public static final class Ball {
        private final String name;
        private final double massKg;
        private final double diameterM;
        private final double dragCoeff;
        private final double liftCoeff;

        public Ball(String name, double massKg, double diameterM, double dragCoeff, double liftCoeff) {

            if (!Double.isFinite(massKg) || massKg <= 0 || !Double.isFinite(diameterM) || diameterM <= 0 || !Double.isFinite(dragCoeff) || dragCoeff < 0 || !Double.isFinite(liftCoeff)) {
                throw new IllegalArgumentException("ball needs finite positive mass/diameter, nonnegative drag and finite lift");
            }
            this.name = name;
            this.massKg = massKg;
            this.diameterM = diameterM;
            this.dragCoeff = dragCoeff;
            this.liftCoeff = liftCoeff;
        }

        public String name() { return name; }
        public double massKg() { return massKg; }
        public double diameterM() { return diameterM; }
        public double dragCoeff() { return dragCoeff; }
        public double liftCoeff() { return liftCoeff; }

        @Override
        public boolean equals(Object other) {

            if (this == other) return true;
            if (!(other instanceof Ball)) return false;
            Ball o = (Ball) other;
            return java.util.Objects.equals(name, o.name) && Double.compare(massKg, o.massKg) == 0 && Double.compare(diameterM, o.diameterM) == 0 && Double.compare(dragCoeff, o.dragCoeff) == 0 && Double.compare(liftCoeff, o.liftCoeff) == 0;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, massKg, diameterM, dragCoeff, liftCoeff);
        }

        @Override
        public String toString() {
            return "Ball[name=" + name + ", massKg=" + massKg + ", diameterM=" + diameterM + ", dragCoeff=" + dragCoeff + ", liftCoeff=" + liftCoeff + "]";
        }

        public double radius() { return diameterM / 2; }
        public double area() { return Math.PI * diameterM * diameterM / 4; }
    
    }

    public enum Outcome { MAKE, SHORT, LONG, LEFT, RIGHT, TIMEOUT }

    public static final class Result {
        private final Outcome outcome;
        private final double timeS;
        private final Vec3 endPoint;
        private final double along;
        private final double lateral;
        private final double apexZ;
        private final double entrySpeed;
        private final double entryAngleDeg;

        public Result(Outcome outcome, double timeS, Vec3 endPoint, double along, double lateral, double apexZ, double entrySpeed, double entryAngleDeg) {

            this.outcome = outcome;
            this.timeS = timeS;
            this.endPoint = endPoint;
            this.along = along;
            this.lateral = lateral;
            this.apexZ = apexZ;
            this.entrySpeed = entrySpeed;
            this.entryAngleDeg = entryAngleDeg;
        }

        public Outcome outcome() { return outcome; }
        public double timeS() { return timeS; }
        public Vec3 endPoint() { return endPoint; }
        public double along() { return along; }
        public double lateral() { return lateral; }
        public double apexZ() { return apexZ; }
        public double entrySpeed() { return entrySpeed; }
        public double entryAngleDeg() { return entryAngleDeg; }

        @Override
        public boolean equals(Object other) {

            if (this == other) return true;
            if (!(other instanceof Result)) return false;
            Result o = (Result) other;
            return java.util.Objects.equals(outcome, o.outcome) && Double.compare(timeS, o.timeS) == 0 && java.util.Objects.equals(endPoint, o.endPoint) && Double.compare(along, o.along) == 0 && Double.compare(lateral, o.lateral) == 0 && Double.compare(apexZ, o.apexZ) == 0 && Double.compare(entrySpeed, o.entrySpeed) == 0 && Double.compare(entryAngleDeg, o.entryAngleDeg) == 0;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(outcome, timeS, endPoint, along, lateral, apexZ, entrySpeed, entryAngleDeg);
        }

        @Override
        public String toString() {
            return "Result[outcome=" + outcome + ", timeS=" + timeS + ", endPoint=" + endPoint + ", along=" + along + ", lateral=" + lateral + ", apexZ=" + apexZ + ", entrySpeed=" + entrySpeed + ", entryAngleDeg=" + entryAngleDeg + "]";
        }

        public boolean made() { return outcome == Outcome.MAKE; }
    
    }

    public static Result fly(Ball ball, Vec3 p0, Vec3 v0, Aperture target) {
        return fly(ball, p0, v0, target, Physics.AIR_DENSITY, 0.002, 4.0);
    }

    public static Result fly(Ball ball, Vec3 p0, Vec3 v0, Aperture target, double airDensity, double dt, double maxT) {
        return fly(ball, p0, v0, target, airDensity, dt, maxT, null);
    }

    public static Result fly(Ball ball, Vec3 p0, Vec3 v0, Aperture target, double airDensity, double dt, double maxT, java.util.List<Vec3> trace) {

        if (!Double.isFinite(dt) || dt <= 0 || !Double.isFinite(maxT) || maxT < 0 || !Double.isFinite(airDensity) || airDensity < 0 || !finite(p0) || !finite(v0)) {
            throw new IllegalArgumentException("flight needs finite vectors, positive dt, nonnegative horizon and air density");
        }
        if (trace != null) trace.add(p0);
        double kd = 0.5 * airDensity * ball.dragCoeff() * ball.area() / ball.massKg();
        double kl = 0.5 * airDensity * ball.liftCoeff() * ball.area() / ball.massKg();
        double[] s = {p0.x(), p0.y(), p0.z(), v0.x(), v0.y(), v0.z()};
        double[] k1 = new double[6], k2 = new double[6], k3 = new double[6], k4 = new double[6], work = new double[6];
        double apex = p0.z();
        double prevDist = target.signedDistance(p0);
        double horizontal = Math.hypot(v0.x(), v0.y());
        double travelX = horizontal > 1e-9 ? v0.x() / horizontal : 1;
        double travelY = horizontal > 1e-9 ? v0.y() / horizontal : 0;
        Vec3 center = target.center(), normal = target.normal();
        double t = 0;
        while (t < maxT) {
            double prevX = s[0], prevY = s[1], prevZ = s[2];
            double step = Math.min(dt, maxT - t);
            if (t + step <= t) throw new IllegalArgumentException("timestep cannot advance flight time");
            rk4(s, step, kd, kl, k1, k2, k3, k4, work);
            t += step;
            apex = Math.max(apex, s[2]);
            if (trace != null) trace.add(new Vec3(s[0], s[1], s[2]));
            horizontal = Math.hypot(s[3], s[4]);
            if (horizontal > 1e-9) { travelX = s[3] / horizontal; travelY = s[4] / horizontal; }
            double dist = (s[0] - center.x()) * normal.x() + (s[1] - center.y()) * normal.y() + (s[2] - center.z()) * normal.z();
            double normalVelocity = s[3] * normal.x() + s[4] * normal.y() + s[5] * normal.z();
            if (prevDist <= 0 && dist > 0 && normalVelocity > 0) {
                double f = -prevDist / (dist - prevDist);
                Vec3 out = new Vec3(prevX + f * (s[0] - prevX), prevY + f * (s[1] - prevY), prevZ + f * (s[2] - prevZ));
                if (target.overlaps(out, ball.radius())) {
                    Vec2 travel = new Vec2(travelX, travelY);
                    Vec3 v = new Vec3(s[3], s[4], s[5]);
                    Vec2 rel = out.minus(target.center()).xy();
                    return new Result(Outcome.SHORT, t - step + f * step, out, rel.dot(travel), travel.cross(rel), apex, v.norm(), Double.NaN);
                }
            }
            if (prevDist > 0 && dist <= 0 && normalVelocity < 0) {
                double f = prevDist / (prevDist - dist);
                Vec3 hit = new Vec3(prevX + f * (s[0] - prevX), prevY + f * (s[1] - prevY), prevZ + f * (s[2] - prevZ));
                return classify(ball, target, hit, new Vec3(s[3], s[4], s[5]), new Vec2(travelX, travelY), t - step + f * step, apex);
            }
            prevDist = dist;
            if (s[2] <= ball.radius()) {
                return hitFloor(target, new Vec3(s[0], s[1], s[2]), new Vec3(s[3], s[4], s[5]), new Vec2(travelX, travelY), t, apex);
            }
        }
        Vec3 p = new Vec3(s[0], s[1], s[2]);
        return new Result(Outcome.TIMEOUT, t, p, Double.NaN, Double.NaN, apex, Double.NaN, Double.NaN);
    }

    private static Result classify(Ball ball, Aperture target, Vec3 hit, Vec3 v, Vec2 travel, double t, double apex) {

        Vec2 rel = hit.minus(target.center()).xy();
        double along = rel.dot(travel);
        double lateral = travel.cross(rel);
        double speed = v.norm();
        double entry = Math.toDegrees(Math.asin(Math.min(1, -v.dot(target.normal()) / Math.max(1e-9, speed))));
        if (target.fits(hit, ball.radius())) return new Result(Outcome.MAKE, t, hit, along, lateral, apex, speed, entry);
        double[] l = target.local(hit);
        Vec3 tr = new Vec3(travel.x(), travel.y(), 0);
        double tu = tr.dot(target.widthAxis()), th = tr.dot(target.heightAxis()), tn = Math.hypot(tu, th);
        double longCoord = tn > 0.3 ? (l[0] * tu + l[1] * th) / tn : l[1];
        Outcome o;
        if (Math.abs(lateral) > Math.abs(longCoord)) o = lateral > 0 ? Outcome.LEFT : Outcome.RIGHT;
        else o = longCoord > 0 ? Outcome.LONG : Outcome.SHORT;
        return new Result(o, t, hit, along, lateral, apex, speed, entry);
    }

    private static Result hitFloor(Aperture target, Vec3 p, Vec3 v, Vec2 travel, double t, double apex) {

        Vec2 rel = p.minus(target.center()).xy();
        double along = rel.dot(travel);
        double lateral = travel.cross(rel);
        return new Result(along > 0 ? Outcome.LONG : Outcome.SHORT, t, p, along, lateral, apex, v.norm(), Double.NaN);
    }

    private static boolean finite(Vec3 v) {
        return v != null && Double.isFinite(v.x()) && Double.isFinite(v.y()) && Double.isFinite(v.z());
    }

    private static void rk4(double[] s, double h, double kd, double kl, double[] k1, double[] k2, double[] k3, double[] k4, double[] work) {

        deriv(s, kd, kl, k1);
        add(s, k1, h / 2, work);
        deriv(work, kd, kl, k2);
        add(s, k2, h / 2, work);
        deriv(work, kd, kl, k3);
        add(s, k3, h, work);
        deriv(work, kd, kl, k4);
        for (int i = 0; i < 6; i++) s[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }

    private static void add(double[] s, double[] k, double h, double[] o) {
        for (int i = 0; i < 6; i++) o[i] = s[i] + h * k[i];
    }

    private static void deriv(double[] s, double kd, double kl, double[] out) {

        double vx = s[3], vy = s[4], vz = s[5];
        double sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double ax = -kd * sp * vx, ay = -kd * sp * vy, az = -Physics.G - kd * sp * vz;
        if (kl != 0 && sp > 1e-9) {
            double ux = vx / sp, uy = vy / sp, uz = vz / sp;
            double lx = -uz * ux, ly = -uz * uy, lz = 1 - uz * uz;
            double ln = Math.sqrt(lx * lx + ly * ly + lz * lz);
            if (ln > 1e-9) {
                double f = kl * sp * sp / ln;
                ax += f * lx;
                ay += f * ly;
                az += f * lz;
            }
        }
        out[0] = vx; out[1] = vy; out[2] = vz;
        out[3] = ax; out[4] = ay; out[5] = az;
    }
}
