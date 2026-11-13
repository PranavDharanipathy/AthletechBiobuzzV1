package org.athletech.sim.scenarios;

import org.athletech.sim.config.RobotParams;
import org.athletech.sim.robot.CrServoTurret;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretRoute;
import org.firstinspires.ftc.teamcode.Tuners.TurretAutotuneEngine;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

//code in the loop: the robot's TurretAutotuneEngine + TurretController + TurretRoute on a sim CR-servo turret from the profile
public final class TurretScenario {

    private TurretScenario() {}

    static final double H = 0.0005;
    //the robot's servos ignore commands this long after START
    static final double SERVO_WAKE = 1.1;
    static final double[] TURN_ANGLES = {2, 5, 10, 20, 30, 45, 60, 90};
    static final double[] MATCH_SPINS = {1.5, 3, 6};

    static final class Setup {

        CrServoTurret.Params plant;
        double tpd;
        CodeLoop loop;
    }

    static Setup setup(RobotParams rp, SimContext ctx) {

        double tpd = rp.req("turret.ticks_per_degree");
        double gain = rp.req("turret.servo_speed_dps_per_v"), tau = rp.req("turret.servo_time_constant_s"), dead = rp.req("turret.servo_dead_time_s");
        double kinetic = rp.req("turret.kinetic_friction_v"), breakaway = rp.req("turret.breakaway_friction_v");
        double noise = rp.opt("turret.velocity_noise_dps", 0.44);
        double top = rp.opt("turret.max_speed_dps", Double.POSITIVE_INFINITY), accel = rp.opt("turret.max_accel_dps2", Double.POSITIVE_INFINITY);
        rp.requireAll();
        Setup s = new Setup();
        s.tpd = tpd;
        s.plant = new CrServoTurret.Params(gain * tpd, tau, dead, kinetic, breakaway, noise * tpd, top * tpd, accel * tpd);
        s.plant.startDelay = Math.max(dead, rp.opt("turret.servo_start_delay_s", dead));
        s.plant.coastDecel = rp.opt("turret.coast_decel_dps2", Double.POSITIVE_INFINITY) * tpd;
        s.plant.neutralDecel = rp.opt("turret.neutral_decel_dps2", Double.POSITIVE_INFINITY) * tpd;
        s.loop = CodeLoop.from(ctx.args, rp, ctx.seed);
        return s;
    }

    static double[] currentKnobs() {
        return new double[] {TurretConstants.TURRET_KP_FAR, TurretConstants.TURRET_KP_CLOSE, TurretConstants.TURRET_KI_FAR, TurretConstants.TURRET_KI_CLOSE, TurretConstants.TURRET_KD_FAR, TurretConstants.TURRET_KD_CLOSE, TurretConstants.TURRET_K_HOLD_VOLTS, TurretConstants.TURRET_FRICTION_DEADBAND_DEGREES, TurretConstants.TURRET_MAX_I, TurretConstants.TURRET_STRIBECK_DEGREES_PER_SECOND};
    }

    //the autotuner against the sim turret, from the robot's current values
    static TurretAutotuneEngine autotune(Setup s, CrServoTurret turret, double maxSeconds, long seed) {

        TurretAutotuneEngine e = new TurretAutotuneEngine(s.tpd, currentKnobs(), TurretConstants.TURRET_GAIN, TurretConstants.TURRET_KINETIC_FRICTION, TurretConstants.TURRET_BREAKAWAY_FRICTION);
        CodeLoop loop = new CodeLoop(s.loop.meanSeconds, s.loop.jitter, seed);
        double t = 0, next = 0;
        while (t < maxSeconds && !e.isDone() && !e.isFaulted()) {
            if (t >= next) {
                e.update(t, turret.encoderPosition());
                double v = t < SERVO_WAKE ? 0 : e.getVolts();
                turret.command(t, v, v);
                next += loop.next();
            }
            turret.step(t, H);
            t += H;
        }
        return e;
    }

    //the robot's controller with TurretConstants, or with what an autotune found
    static TurretController controller(Setup s, Map<String, Double> tuned) {

        TurretController c = new TurretController();
        c.applyConstants(s.tpd);
        if (tuned == null || tuned.isEmpty()) return c;
        c.gain = tuned.get("gain");
        c.kCoulomb = tuned.get("kinetic_friction");
        c.kStatic = tuned.get("breakaway_friction");
        c.kpFar = tuned.get("kp_far");
        c.kpClose = tuned.get("kp_close");
        c.kiFar = tuned.get("ki_far");
        c.kiClose = tuned.get("ki_close");
        c.kdFar = tuned.get("kd_far");
        c.kdClose = tuned.get("kd_close");
        c.kHold = tuned.get("k_hold_volts");
        c.frictionDeadband = tuned.get("friction_deadband_degrees") * s.tpd;
        c.maxI = tuned.get("max_i");
        c.minI = -c.maxI;
        c.stribeckVelocity = tuned.get("stribeck_degrees_per_second") * s.tpd;
        return c;
    }

    //TurretBase's limits: hard limit past the travel, dead-encoder cutoff (latches)
    static final class Limits {

        double frozenSince = Double.NaN, lastVolts, lastPosition = Double.NaN, firstPosition = Double.NaN, firstTime = Double.NaN, tpd;
        boolean frozen, counted;
        TurretController controller;

        Limits(double tpd) {
            this.tpd = tpd;
        }

        double apply(double now, double volts, double position) {

            double margin = TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES;
            double deg = ShooterConstants.TURRET_AIM_DIRECTION * position / tpd, outward = ShooterConstants.TURRET_AIM_DIRECTION * volts;
            if ((deg > ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES + margin && outward > 0) || (deg < ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES - margin && outward < 0)) volts = 0;
            if (Double.isNaN(firstPosition)) firstPosition = position;
            if (Double.isNaN(firstTime)) firstTime = now;
            if (Math.abs(position - firstPosition) >= 3) counted = true;
            boolean awake = now - firstTime >= TurretConstants.TURRET_SERVO_WAKE_SECONDS;
            if (!awake || position != lastPosition || Math.abs(lastVolts) < TurretConstants.TURRET_BREAKAWAY_FRICTION + TurretConstants.TURRET_FROZEN_ENCODER_OVER_BREAKAWAY_VOLTS) frozenSince = Double.NaN;
            else if (Double.isNaN(frozenSince)) frozenSince = now;
            else if (now - frozenSince >= TurretConstants.TURRET_FROZEN_ENCODER_SECONDS) frozen = true;
            if (awake && controller != null && controller.pulsesWithoutCounts >= TurretConstants.TURRET_PULSE_DEAD_ENCODER_PUSHES) frozen = true;
            if (!counted) volts = Math.max(-(TurretConstants.TURRET_BREAKAWAY_FRICTION + TurretConstants.TURRET_UNCOUNTED_OVER_BREAKAWAY_VOLTS), Math.min((TurretConstants.TURRET_BREAKAWAY_FRICTION + TurretConstants.TURRET_UNCOUNTED_OVER_BREAKAWAY_VOLTS), volts));
            if (frozen) volts = 0;
            lastVolts = volts;
            lastPosition = position;
            return Math.max(-TurretConstants.TURRET_NOMINAL_VOLTS, Math.min(TurretConstants.TURRET_NOMINAL_VOLTS, volts));
        }
    }

    static final class Track {

        double settleS, overshootDeg, finalDeg, peakSpeedDps;
    }

    //a step from rest: settle = stays within 1 deg
    static Track step(Setup s, TurretController c, double degrees, long seed) {

        CrServoTurret turret = new CrServoTurret(s.plant, seed);
        CodeLoop loop = new CodeLoop(s.loop.meanSeconds, s.loop.jitter, seed);
        Limits limits = new Limits(s.tpd);
        limits.controller = c;
        c.reset();
        Track r = new Track();
        double target = degrees * s.tpd, t = 0, next = 0, last = 0, settled = Double.NaN;
        while (t < 3) {
            if (t >= next) {
                double position = turret.encoderPosition();
                double v = limits.apply(t, c.update(t - last, target, 0, position), position);
                last = t;
                next += loop.next();
                turret.command(t, v, v);
            }
            turret.step(t, H);
            double e = (target - turret.position()) / s.tpd;
            r.overshootDeg = Math.max(r.overshootDeg, -Math.signum(degrees) * e);
            r.peakSpeedDps = Math.max(r.peakSpeedDps, Math.abs(turret.velocity()) / s.tpd);
            if (Math.abs(e) > 1) settled = Double.NaN;
            else if (Double.isNaN(settled)) settled = t;
            t += H;
        }
        r.settleS = Double.isNaN(settled) ? Double.POSITIVE_INFINITY : settled;
        r.finalDeg = (target - turret.position()) / s.tpd;
        return r;
    }

    static double heading(double t) {
        return 0.6 * Math.sin(2 * Math.PI * 0.25 * t) + 0.3 * Math.sin(2 * Math.PI * 0.7 * t);
    }

    static double headingRate(double t) {
        return 0.6 * 2 * Math.PI * 0.25 * Math.cos(2 * Math.PI * 0.25 * t) + 0.3 * 2 * Math.PI * 0.7 * Math.cos(2 * Math.PI * 0.7 * t);
    }

    static double headingAccel(double t) {
        return -0.6 * Math.pow(2 * Math.PI * 0.25, 2) * Math.sin(2 * Math.PI * 0.25 * t) - 0.3 * Math.pow(2 * Math.PI * 0.7, 2) * Math.sin(2 * Math.PI * 0.7 * t);
    }

    //RMS aim error (deg) holding a field direction while the robot turns back and forth, aim rate fed forward like the Shooter does
    static double spin(Setup s, TurretController c, long seed) {

        CrServoTurret turret = new CrServoTurret(s.plant, seed);
        CodeLoop loop = new CodeLoop(s.loop.meanSeconds, s.loop.jitter, seed);
        Limits limits = new Limits(s.tpd);
        limits.controller = c;
        c.reset();
        double toTicks = ShooterConstants.TURRET_AIM_DIRECTION * -180 / Math.PI * s.tpd;
        double t = 0, next = 0, last = 0, ee = 0, n = 0;
        while (t < 20) {
            turret.setRobotAcceleration(headingAccel(t) * 180 / Math.PI * s.tpd);
            if (t >= next) {
                double position = turret.encoderPosition();
                double v = limits.apply(t, c.update(t - last, toTicks * heading(t), TurretConstants.TURRET_RATE_FEEDFORWARD_SCALE * toTicks * headingRate(t), position), position);
                last = t;
                next += loop.next();
                turret.command(t, v, v);
                if (t > 2) {
                    double e = (toTicks * heading(t) - turret.position()) / s.tpd;
                    ee += e * e;
                    n++;
                }
            }
            turret.step(t, H);
            t += H;
        }
        return Math.sqrt(ee / Math.max(1, n));
    }

    static final class MatchSpin {

        double maxTravelDeg, firstSwingDeg = Double.NaN, aimedFraction;
        int swings;
    }

    //the match aim (TurretRoute) while the robot spins round one way then back
    static MatchSpin matchSpin(Setup s, TurretController c, double spin, double seconds, long seed) {

        CrServoTurret turret = new CrServoTurret(s.plant, seed);
        CodeLoop loop = new CodeLoop(s.loop.meanSeconds, s.loop.jitter, seed + 7);
        Limits limits = new Limits(s.tpd);
        limits.controller = c;
        TurretRoute route = new TurretRoute();
        c.reset();
        double min = ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, max = ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES;
        double toAim = ShooterConstants.TURRET_AIM_DIRECTION * -180 / Math.PI;
        MatchSpin r = new MatchSpin();
        double t = 0, next = 0, last = 0, aimed = 0, n = 0, half = seconds / 2;
        while (t < seconds) {
            if (t >= next) {
                double dt = t - last;
                last = t;
                next += loop.next();
                double aim = toAim * (heading(t) + spin * (t < half ? t : 2 * half - t));
                double aimRate = toAim * (headingRate(t) + (t < half ? spin : -spin));
                double position = turret.encoderPosition();
                double tt = route.update(aim, aimRate, position / s.tpd, min, max, ShooterConstants.TURRET_SWING_MARGIN_DEGREES, t);
                if (route.hasSwung() && r.swings++ == 0) r.firstSwingDeg = Math.abs(position / s.tpd);
                double rate = route.isOutside(min, max) ? 0 : TurretConstants.TURRET_RATE_FEEDFORWARD_SCALE * aimRate * s.tpd;
                double v = limits.apply(t, c.update(dt, tt * s.tpd, rate, position), position);
                turret.command(t, v, v);
                if (Math.abs(Math.IEEEremainder(position / s.tpd - aim, 360)) < 2) aimed++;
                n++;
            }
            turret.step(t, H);
            r.maxTravelDeg = Math.max(r.maxTravelDeg, Math.abs(turret.position()) / s.tpd);
            t += H;
        }
        r.aimedFraction = n > 0 ? aimed / n : 0;
        return r;
    }

    public static void run(SimContext ctx) throws Exception {

        RobotParams rp = new RobotParams(ctx.db);
        Setup s = setup(rp, ctx);
        ctx.log.println("== Turret (robot's real TurretAutotuneEngine + TurretController, sim CR-servo turret) ==");
        ctx.log.printf(Locale.ROOT, "  sim turret: %.1f deg/s per V, lag %.0f ms, dead time %.0f ms (%.0f ms from rest), friction %.2f / %.2f V, %.2f ticks/deg; loop %s%n", s.plant.gain / s.tpd, 1000 * s.plant.timeConstant, 1000 * s.plant.deadTime, 1000 * s.plant.startDelay, s.plant.kineticFriction, s.plant.breakawayFriction, s.tpd, s.loop.describe());

        TurretAutotuneEngine e = autotune(s, new CrServoTurret(s.plant, ctx.seed), 3000, ctx.seed + 1);
        ctx.log.println("  Turret Autotuner: " + (e.isDone() ? e.getStatus() : e.isFaulted() ? "FAULT - " + e.getStatus() : "didn't finish, stuck in " + e.getStage()));
        List<String> h = e.getHistory();
        for (int k = Math.max(0, h.size() - 6); k < h.size(); k++) ctx.log.println("    " + h.get(k));

        TurretController now = controller(s, null), tuned = e.isDone() ? controller(s, e.getResults()) : null;
        TurretController best = tuned != null ? tuned : now;
        Track nowStep = step(s, now, 30, ctx.seed), bestStep = step(s, best, 30, ctx.seed);
        double nowSpin = spin(s, now, ctx.seed), bestSpin = spin(s, best, ctx.seed);
        ctx.log.printf(Locale.ROOT, "  30 deg step: TurretConstants overshoot %.2f deg, settle %.2f s; %s overshoot %.2f deg, settle %.2f s%n", nowStep.overshootDeg, nowStep.settleS, tuned != null ? "autotuned" : "(same)", bestStep.overshootDeg, bestStep.settleS);
        ctx.log.printf(Locale.ROOT, "  robot turning back and forth: RMS aim error %.2f deg (TurretConstants), %.2f deg (best)%n", nowSpin, bestSpin);

        List<double[]> turn = new ArrayList<>();
        StringBuilder turnText = new StringBuilder();
        for (double deg : TURN_ANGLES) {
            Track tr = step(s, best, deg, ctx.seed + (long) deg);
            turn.add(new double[] {deg, tr.settleS});
            turnText.append(String.format(Locale.ROOT, " %.0f deg %.2f s,", deg, tr.settleS));
        }
        ctx.log.println("  time to turn and settle:" + turnText.substring(0, turnText.length() - 1));

        StringBuilder spinText = new StringBuilder();
        for (double rate : MATCH_SPINS) {
            MatchSpin m = matchSpin(s, best, rate, 30, ctx.seed + 11);
            spinText.append(String.format(Locale.ROOT, " %.1f rad/s: max travel %.0f deg, %d swings, aimed %.0f%%;", rate, m.maxTravelDeg, m.swings, 100 * m.aimedFraction));
        }
        ctx.log.println("  match aim, robot spinning round one way then back (aimed = within 2 deg):" + spinText.substring(0, spinText.length() - 1));

        SimContext.Saver save = ctx.saver(rp, "sim turret");
        save.param("turret.step_overshoot_deg", bestStep.overshootDeg);
        save.param("turret.spin_tracking_error_deg", bestSpin);
        save.curve("turret.time_to_turn", turn);
        save.report();
        if (!rp.placeholderInputs().isEmpty()) ctx.log.println("  uses PLACEHOLDER inputs: " + String.join(", ", rp.placeholderInputs()));
        if (!rp.assumptions().isEmpty()) ctx.log.println("  assumptions: " + String.join("; ", rp.assumptions()));
    }
}
