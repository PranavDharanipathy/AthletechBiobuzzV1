package org.athletech.sim.predict;

import org.athletech.sim.constants.Physics;
import org.athletech.sim.field.Aperture;
import org.athletech.sim.physics.Projectile;
import org.athletech.sim.robot.Launcher;
import org.athletech.sim.undeterminables.Distribution;
import org.athletech.sim.utility.Rng;
import org.athletech.sim.utility.Stats;
import org.athletech.sim.utility.Vec2;
import org.athletech.sim.utility.Vec3;

import java.util.EnumMap;
import java.util.Map;

/**
 * Monte Carlo make probability: fires many simulated shots, each with random errors drawn from
 * the profile's distributions (flywheel speed, hood, ball-to-ball variation, turret aim, and the
 * robot not being exactly where it thinks it is), and counts how many score.
 */
public final class ShotPredictor {
    private ShotPredictor() {
    }

    /** The random errors applied to every simulated shot. */
    public static final class Errors {
        private final Distribution rpm;
        private final Distribution hoodDeg;
        private final Distribution exitSpeedFrac;
        private final Distribution exitAngleDeg;
        private final Distribution aimDeg;
        private final Distribution poseAxisM;
        private final Distribution headingDeg;
        private final Distribution loopRpm;

        public Errors(Distribution rpm, Distribution hoodDeg, Distribution exitSpeedFrac, Distribution exitAngleDeg, Distribution aimDeg, Distribution poseAxisM, Distribution headingDeg) {
            this(rpm, hoodDeg, exitSpeedFrac, exitAngleDeg, aimDeg, poseAxisM, headingDeg, Distribution.none("loop rpm", "rpm"));
        }

        /** @param loopRpm extra flywheel speed error from loop timing (control.flywheel_rpm_error_from_loop) */
        public Errors(Distribution rpm, Distribution hoodDeg, Distribution exitSpeedFrac, Distribution exitAngleDeg, Distribution aimDeg, Distribution poseAxisM,
                      Distribution headingDeg, Distribution loopRpm) {
            this.rpm = rpm;
            this.hoodDeg = hoodDeg;
            this.exitSpeedFrac = exitSpeedFrac;
            this.exitAngleDeg = exitAngleDeg;
            this.aimDeg = aimDeg;
            this.poseAxisM = poseAxisM;
            this.headingDeg = headingDeg;
            this.loopRpm = loopRpm;
        }

        public Distribution rpm() { return rpm; }
        public Distribution hoodDeg() { return hoodDeg; }
        public Distribution exitSpeedFrac() { return exitSpeedFrac; }
        public Distribution exitAngleDeg() { return exitAngleDeg; }
        public Distribution aimDeg() { return aimDeg; }
        public Distribution poseAxisM() { return poseAxisM; }
        public Distribution headingDeg() { return headingDeg; }
        public Distribution loopRpm() { return loopRpm; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Errors)) return false;
            Errors o = (Errors) other;
            return java.util.Objects.equals(rpm, o.rpm)
                    && java.util.Objects.equals(hoodDeg, o.hoodDeg)
                    && java.util.Objects.equals(exitSpeedFrac, o.exitSpeedFrac)
                    && java.util.Objects.equals(exitAngleDeg, o.exitAngleDeg)
                    && java.util.Objects.equals(aimDeg, o.aimDeg)
                    && java.util.Objects.equals(poseAxisM, o.poseAxisM)
                    && java.util.Objects.equals(headingDeg, o.headingDeg)
                    && java.util.Objects.equals(loopRpm, o.loopRpm);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(rpm, hoodDeg, exitSpeedFrac, exitAngleDeg, aimDeg, poseAxisM, headingDeg, loopRpm);
        }

        @Override
        public String toString() {
            return "Errors[rpm=" + rpm + ", hoodDeg=" + hoodDeg + ", exitSpeedFrac=" + exitSpeedFrac + ", exitAngleDeg=" + exitAngleDeg + ", aimDeg=" + aimDeg + ", poseAxisM=" + poseAxisM + ", headingDeg=" + headingDeg + ", loopRpm=" + loopRpm + "]";
        }

        /** Combined 1-sigma launch angle error (hood + ball-to-ball), degrees. */
        public double angleSigmaDeg() {
            return Math.hypot(sd(hoodDeg), sd(exitAngleDeg));
        }

        /** Combined 1-sigma exit speed error (m/s) at exit speed v: ball-to-ball scatter plus flywheel rpm error. */
        public java.util.function.DoubleUnaryOperator speedSigma(Launcher launcher, String ball) {
            return v -> {
                double rpm = launcher.rpmForExitSpeed(ball, v);
                double rpmFrac = rpm > 0 && Double.isFinite(rpm) ? Math.hypot(sd(this.rpm), sd(loopRpm)) / rpm : 0;
                return v * Math.hypot(sd(exitSpeedFrac), rpmFrac);
            };
        }

        /** Standard deviation of a distribution (0 when unknown). */
        static double sd(Distribution d) {
            switch (d.kind()) {
                case NONE:
                case CONSTANT:
                    return 0;
                case UNIFORM:
                    if (Double.isFinite(d.min()) && Double.isFinite(d.max())) return (d.max() - d.min()) / Math.sqrt(12);
                    break;
                case TRIANGULAR:
                    if (Double.isFinite(d.min()) && Double.isFinite(d.max())) {
                        double a = d.min(), b = d.max(), c = Double.isFinite(d.mode()) ? d.mode() : (Double.isFinite(d.mean()) ? d.mean() : (a + b) / 2);
                        return Math.sqrt(Math.max(0, (a * a + b * b + c * c - a * b - a * c - b * c) / 18));
                    }
                    break;
                case EMPIRICAL:
                    if (d.samples().length > 1) return Stats.std(d.samples());
                    break;
                default:
                    break;
            }
            return Double.isFinite(d.std()) ? Math.abs(d.std()) : 0;
        }
    
    }

    public static final class Prediction {
        private final double probability;
        private final double stdErr;
        private final double low95;
        private final double high95;
        private final int trials;
        private final Map<Projectile.Outcome, Integer> outcomes;

        public Prediction(double probability, double stdErr, double low95, double high95, int trials, Map<Projectile.Outcome, Integer> outcomes) {
            this.probability = probability;
            this.stdErr = stdErr;
            this.low95 = low95;
            this.high95 = high95;
            this.trials = trials;
            this.outcomes = outcomes;
        }

        public double probability() { return probability; }
        public double stdErr() { return stdErr; }
        public double low95() { return low95; }
        public double high95() { return high95; }
        public int trials() { return trials; }
        public Map<Projectile.Outcome, Integer> outcomes() { return outcomes; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Prediction)) return false;
            Prediction o = (Prediction) other;
            return Double.compare(probability, o.probability) == 0
                    && Double.compare(stdErr, o.stdErr) == 0
                    && Double.compare(low95, o.low95) == 0
                    && Double.compare(high95, o.high95) == 0
                    && trials == o.trials
                    && java.util.Objects.equals(outcomes, o.outcomes);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(probability, stdErr, low95, high95, trials, outcomes);
        }

        @Override
        public String toString() {
            return "Prediction[probability=" + probability + ", stdErr=" + stdErr + ", low95=" + low95 + ", high95=" + high95 + ", trials=" + trials + ", outcomes=" + outcomes + "]";
        }
    }

    /**
     * @param rpmBeforeShot actual wheel speed when the ball is fed (from a burst simulation), or
     *                      NaN to use the aim solution's target speed (rested flywheel)
     */
    public static Prediction predict(Launcher launcher, String ball, Aperture target, AimSolver.Solution aim,
                                     double bearingRad, double rpmBeforeShot, Errors e, int trials, Rng rng) {
        Map<Projectile.Outcome, Integer> outcomes = new EnumMap<>(Projectile.Outcome.class);
        if (!aim.reachable()) {
            outcomes.put(Projectile.Outcome.SHORT, trials);
            return new Prediction(0, 0, 0, Stats.wilson(0, trials)[1], trials, outcomes);
        }
        Launcher.Params lp = launcher.params();
        Projectile.Ball b = lp.ball(ball).ball();
        AimSolver.Spot believed = AimSolver.spot(lp, target, aim.distanceM(), bearingRad);
        double baseRpm = Double.isNaN(rpmBeforeShot) ? aim.rpm() : rpmBeforeShot;
        int made = 0;
        for (int i = 0; i < trials; i++) {
            // where the robot really is: believed spot plus localization error; it still aims from the believed spot
            double dx = e.poseAxisM().sample(rng), dy = e.poseAxisM().sample(rng);
            Vec3 launch = believed.launchPoint().plus(new Vec3(dx, dy, 0));
            double rpm = baseRpm + e.rpm().sample(rng) + e.loopRpm().sample(rng);
            double speed = launcher.exitSpeed(ball, rpm) * (1 + e.exitSpeedFrac().sample(rng));
            // the hood cannot go past its stops; ball-to-ball angle scatter is on top of that
            double hood = Math.max(lp.hoodMinDeg(), Math.min(lp.hoodMaxDeg(), aim.hoodDeg() + e.hoodDeg().sample(rng)));
            double elev = Physics.rad(hood + e.exitAngleDeg().sample(rng));
            // believed heading error turns the whole aim (the turret angle is relative to the robot)
            double yaw = believed.yawToTarget() + Physics.rad(e.aimDeg().sample(rng) + e.headingDeg().sample(rng));
            Projectile.Result r = Projectile.fly(b, launch, Launcher.launchVelocity(yaw, elev, speed, Vec2.ZERO), target);
            outcomes.merge(r.outcome(), 1, Integer::sum);
            if (r.made()) made++;
        }
        double[] ci = Stats.wilson(made, trials);
        return new Prediction((double) made / trials, Stats.proportionStd(made, trials), ci[0], ci[1], trials, outcomes);
    }
}
