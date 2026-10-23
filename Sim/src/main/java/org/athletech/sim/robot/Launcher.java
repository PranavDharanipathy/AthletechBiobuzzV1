package org.athletech.sim.robot;

import org.athletech.sim.constants.Physics;
import org.athletech.sim.physics.Projectile;
import org.athletech.sim.utility.Vec2;
import org.athletech.sim.utility.Vec3;

import java.util.Map;

/**
 * The whole launcher: flywheel, hood (launch elevation), turret (yaw) and where on the robot the
 * ball leaves. Ball-specific behavior (how fast each game element comes off the wheel and how much
 * energy it takes) lives in {@link BallFit}.
 */
public final class Launcher {

    /** How one game element leaves this launcher. */
    public static final class BallFit {
        private final Projectile.Ball ball;
        private final double exitSpeedRatio;
        private final double energyLossFactor;

        public BallFit(Projectile.Ball ball, double exitSpeedRatio, double energyLossFactor) {
            this.ball = ball;
            this.exitSpeedRatio = exitSpeedRatio;
            this.energyLossFactor = energyLossFactor;
        }

        public Projectile.Ball ball() { return ball; }
        public double exitSpeedRatio() { return exitSpeedRatio; }
        public double energyLossFactor() { return energyLossFactor; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof BallFit)) return false;
            BallFit o = (BallFit) other;
            return java.util.Objects.equals(ball, o.ball)
                    && Double.compare(exitSpeedRatio, o.exitSpeedRatio) == 0
                    && Double.compare(energyLossFactor, o.energyLossFactor) == 0;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(ball, exitSpeedRatio, energyLossFactor);
        }

        @Override
        public String toString() {
            return "BallFit[ball=" + ball + ", exitSpeedRatio=" + exitSpeedRatio + ", energyLossFactor=" + energyLossFactor + "]";
        }
    }

    public static final class Params {
        private final Flywheel.Params flywheel;
        private final double exitHeightM;
        private final double offsetForwardM;
        private final double offsetLeftM;
        private final double hoodMinDeg;
        private final double hoodMaxDeg;
        private final double maxRpm;
        private final double readyToleranceRpm;
        private final double feedIntervalS;
        private final boolean waitForReady;
        private final boolean hasTurret;
        private final Map<String, BallFit> balls;

        public Params(Flywheel.Params flywheel, double exitHeightM, double offsetForwardM, double offsetLeftM, double hoodMinDeg, double hoodMaxDeg, double maxRpm, double readyToleranceRpm, double feedIntervalS, boolean waitForReady, boolean hasTurret, Map<String, BallFit> balls) {
            this.flywheel = flywheel;
            this.exitHeightM = exitHeightM;
            this.offsetForwardM = offsetForwardM;
            this.offsetLeftM = offsetLeftM;
            this.hoodMinDeg = hoodMinDeg;
            this.hoodMaxDeg = hoodMaxDeg;
            this.maxRpm = maxRpm;
            this.readyToleranceRpm = readyToleranceRpm;
            this.feedIntervalS = feedIntervalS;
            this.waitForReady = waitForReady;
            this.hasTurret = hasTurret;
            this.balls = balls;
        }

        public Flywheel.Params flywheel() { return flywheel; }
        public double exitHeightM() { return exitHeightM; }
        public double offsetForwardM() { return offsetForwardM; }
        public double offsetLeftM() { return offsetLeftM; }
        public double hoodMinDeg() { return hoodMinDeg; }
        public double hoodMaxDeg() { return hoodMaxDeg; }
        public double maxRpm() { return maxRpm; }
        public double readyToleranceRpm() { return readyToleranceRpm; }
        public double feedIntervalS() { return feedIntervalS; }
        public boolean waitForReady() { return waitForReady; }
        public boolean hasTurret() { return hasTurret; }
        public Map<String, BallFit> balls() { return balls; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Params)) return false;
            Params o = (Params) other;
            return java.util.Objects.equals(flywheel, o.flywheel)
                    && Double.compare(exitHeightM, o.exitHeightM) == 0
                    && Double.compare(offsetForwardM, o.offsetForwardM) == 0
                    && Double.compare(offsetLeftM, o.offsetLeftM) == 0
                    && Double.compare(hoodMinDeg, o.hoodMinDeg) == 0
                    && Double.compare(hoodMaxDeg, o.hoodMaxDeg) == 0
                    && Double.compare(maxRpm, o.maxRpm) == 0
                    && Double.compare(readyToleranceRpm, o.readyToleranceRpm) == 0
                    && Double.compare(feedIntervalS, o.feedIntervalS) == 0
                    && waitForReady == o.waitForReady
                    && hasTurret == o.hasTurret
                    && java.util.Objects.equals(balls, o.balls);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(flywheel, exitHeightM, offsetForwardM, offsetLeftM, hoodMinDeg, hoodMaxDeg, maxRpm, readyToleranceRpm, feedIntervalS, waitForReady, hasTurret, balls);
        }

        @Override
        public String toString() {
            return "Params[flywheel=" + flywheel + ", exitHeightM=" + exitHeightM + ", offsetForwardM=" + offsetForwardM + ", offsetLeftM=" + offsetLeftM + ", hoodMinDeg=" + hoodMinDeg + ", hoodMaxDeg=" + hoodMaxDeg + ", maxRpm=" + maxRpm + ", readyToleranceRpm=" + readyToleranceRpm + ", feedIntervalS=" + feedIntervalS + ", waitForReady=" + waitForReady + ", hasTurret=" + hasTurret + ", balls=" + balls + "]";
        }

        public BallFit ball(String name) {
            BallFit b = balls.get(name);
            if (b == null) throw new IllegalArgumentException("no launcher data for ball type '" + name + "'");
            return b;
        }
    
    }

    private final Params p;

    public Launcher(Params p) {
        this.p = p;
    }

    public Params params() { return p; }

    public Flywheel newFlywheel() {
        return new Flywheel(p.flywheel());
    }

    /** Exit speed of one ball fired from a wheel spinning at {@code rpm} (wheel not otherwise disturbed). */
    public double exitSpeed(String ball, double rpm) {
        BallFit b = p.ball(ball);
        return Flywheel.shotFrom(Physics.rpmToRadPerSec(rpm), p.flywheel().radiusM(), p.flywheel().moiKgm2(),
                b.ball().massKg(), b.exitSpeedRatio(), b.energyLossFactor()).exitSpeed();
    }

    /** Wheel rpm (before the shot) that gives exit speed {@code v}, or NaN if out of reach. */
    public double rpmForExitSpeed(String ball, double v) {
        double hi = Math.max(p.maxRpm(), 1);
        if (exitSpeed(ball, hi) < v) return Double.NaN;
        double lo = 0;
        for (int i = 0; i < 60; i++) {
            double mid = 0.5 * (lo + hi);
            if (exitSpeed(ball, mid) < v) lo = mid;
            else hi = mid;
        }
        return 0.5 * (lo + hi);
    }

    /** Fastest a ball can leave at {@code maxRpm}, m/s. */
    public double maxExitSpeed(String ball) {
        return exitSpeed(ball, p.maxRpm());
    }

    /** Field position where the ball leaves, for a robot at (x, y) facing {@code headingRad}. */
    public Vec3 launchPoint(double x, double y, double headingRad) {
        Vec2 off = new Vec2(p.offsetForwardM(), p.offsetLeftM()).rotate(headingRad);
        return new Vec3(x + off.x(), y + off.y(), p.exitHeightM());
    }

    /** Ball velocity at release: launcher direction plus the robot's own velocity. */
    public static Vec3 launchVelocity(double yawRad, double elevationRad, double speed, Vec2 robotVelocity) {
        double horiz = speed * Math.cos(elevationRad);
        return new Vec3(horiz * Math.cos(yawRad) + robotVelocity.x(), horiz * Math.sin(yawRad) + robotVelocity.y(),
                speed * Math.sin(elevationRad));
    }
}
