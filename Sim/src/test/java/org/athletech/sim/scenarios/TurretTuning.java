package org.athletech.sim.scenarios;

import org.athletech.sim.robot.CrServoTurret;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretController;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretRoute;
import org.junit.Assume;
import org.junit.Test;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

//TURRET_TUNE=1 gradlew :Sim:test --tests *TurretTuning*: main's aim from real matches (Sim/out/turret-traces, tools/turret_fit.py export) through the turret controller on the measured turret model, a stickier one and the one fitted from the logs
public class TurretTuning {

    static final double H = 0.001;

    static final String[] KNOBS = {"kpFar", "kpClose", "kiFar", "kiClose", "kdFar", "kdClose", "velFilter", "dFilter", "kHold", "deadband", "maxI", "stribeck"};

    static final String[] PLANTS = {"main model", "main model, sticky", "fitted", "fitted, fast"};

    static double tpd() {
        return ShooterConstants.TURRET_TICKS_PER_DEGREE;
    }

    static CrServoTurret plant(int which, long seed) {

        CrServoTurret.Params p;
        if (which <= 1) p = TurretPlants.params(1, which == 1 ? 1.1 : 1, which == 1 ? 1.13 : 1);
        else {
            p = new CrServoTurret.Params(9566 * (which == 3 ? 1.2 : 1), 0.0289, 0.066, 2 * 0.9166, 2 * 0.9633, 0.44 * tpd(), 24490, 161700);
            p.startDelay = 0.1097;
            p.coastDecel = 123300;
            p.neutralDecel = 123300;
        }
        return new CrServoTurret(p, seed);
    }

    static double[] current() {
        return new double[] {TurretConstants.TURRET_KP_FAR, TurretConstants.TURRET_KP_CLOSE, TurretConstants.TURRET_KI_FAR, TurretConstants.TURRET_KI_CLOSE, TurretConstants.TURRET_KD_FAR, TurretConstants.TURRET_KD_CLOSE, TurretConstants.TURRET_VELOCITY_FILTER, TurretConstants.TURRET_D_FILTER, TurretConstants.TURRET_K_HOLD_VOLTS, TurretConstants.TURRET_FRICTION_DEADBAND_DEGREES, TurretConstants.TURRET_MAX_I, TurretConstants.TURRET_STRIBECK_DEGREES_PER_SECOND};
    }

    static TurretController controller(double[] k) {

        TurretController c = new TurretController();
        c.applyConstants(tpd());
        c.kpFar = k[0];
        c.kpClose = k[1];
        c.kiFar = k[2];
        c.kiClose = k[3];
        c.kdFar = k[4];
        c.kdClose = k[5];
        c.kVelocityFilter = clamp(k[6], 0.05, 1);
        c.kDFilter = clamp(k[7], 0.05, 1);
        c.kHold = k[8];
        c.frictionDeadband = Math.max(0, k[9]) * tpd();
        c.minI = -k[10];
        c.maxI = k[10];
        c.stribeckVelocity = Math.max(1e-3, k[11]) * tpd();
        return c;
    }

    //TurretBase's limits after the controller: hard limit, frozen encoder cutoff (latches), servo range
    static final class Limits {

        double frozenSince = Double.NaN, lastVolts, lastPosition = Double.NaN, firstPosition = Double.NaN, firstTime = Double.NaN;
        boolean frozen, counted;
        TurretController controller;

        double apply(double now, double volts, double position) {

            double tpd = tpd(), margin = TurretConstants.TURRET_HARD_LIMIT_MARGIN_DEGREES;
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

    static final class Trace {

        String name;
        double[] t, omega, aim, aimRate;
        boolean[] aiming;
    }

    static List<Trace> traces() throws IOException {

        List<Trace> out = new ArrayList<>();
        File[] files = new File("out/turret-traces").listFiles();
        if (files == null) return out;
        java.util.Arrays.sort(files);
        for (File f : files) {
            if (!f.getName().endsWith(".csv")) continue;
            List<double[]> rows = new ArrayList<>();
            try (BufferedReader r = new BufferedReader(new FileReader(f))) {
                r.readLine();
                String line;
                while ((line = r.readLine()) != null) {
                    String[] s = line.split(",");
                    double[] v = new double[s.length];
                    for (int i = 0; i < s.length; i++) v[i] = parse(s[i]);
                    rows.add(v);
                }
            }
            if (rows.isEmpty() || rows.get(0).length < 7) continue;
            Trace tr = new Trace();
            tr.name = f.getName();
            int n = rows.size();
            tr.t = new double[n];
            tr.omega = new double[n];
            tr.aim = new double[n];
            tr.aimRate = new double[n];
            tr.aiming = new boolean[n];
            for (int i = 0; i < n; i++) {
                double[] v = rows.get(i);
                tr.t[i] = v[0];
                tr.omega[i] = Double.isNaN(v[2]) ? 0 : v[2];
                tr.aim[i] = v[3];
                tr.aimRate[i] = Double.isNaN(v[6]) ? 0 : v[6];
                tr.aiming[i] = !Double.isNaN(v[3]) && v[4] > 0.5 && v[5] < 0.5;
            }
            out.add(tr);
        }
        return out;
    }

    static double parse(String s) {

        try {
            return Double.parseDouble(s);
        }
        catch (NumberFormatException e) {
            return Double.NaN;
        }
    }

    static double[][] buckets;

    //main's Shooter: route the aim, feedforward = aim rate x TURRET_RATE_FEEDFORWARD_SCALE, 0 waiting at an end
    //{share within 1 deg, share within 2 deg, rms deg, holding volt flips per s, froze 0/1}
    static double[] replay(Trace tr, double[] k, int which, long seed) {

        CrServoTurret turret = plant(which, seed);
        TurretController c = controller(k);
        Limits limits = new Limits();
        limits.controller = c;
        TurretRoute route = new TurretRoute();
        double min = ShooterConstants.MIN_TURRET_POSITION_IN_DEGREES, max = ShooterConstants.MAX_TURRET_POSITION_IN_DEGREES;
        double dir = ShooterConstants.TURRET_AIM_DIRECTION;
        double lastVolts = 0, ee = 0, n = 0, in1 = 0, in2 = 0, flips = 0, holdTime = 0;
        int i = 1;
        double next = tr.t[1] - tr.t[0], sim = 0;
        while (i < tr.t.length) {
            if (sim >= next) {
                double dt = tr.t[i] - tr.t[i - 1];
                double position = turret.encoderPosition();
                double target = 0, rate = 0;
                boolean usable = tr.aiming[i];
                if (usable) {
                    double tt = route.update(tr.aim[i], tr.aimRate[i], dir * position / tpd(), min, max, ShooterConstants.TURRET_SWING_MARGIN_DEGREES, sim);
                    target = dir * tt * tpd();
                    usable = !route.isOutside(min, max);
                    rate = usable ? dir * TurretConstants.TURRET_RATE_FEEDFORWARD_SCALE * tr.aimRate[i] * tpd() : 0;
                }
                else route.reset();
                double volts = limits.apply(sim, c.update(dt, target, rate, position), position);
                turret.command(sim, volts, volts);
                if (usable && sim > 1) {
                    double e = Math.IEEEremainder(tr.aim[i] - dir * turret.position() / tpd(), 360);
                    ee += Math.min(e * e, 900);
                    n++;
                    if (Math.abs(e) < 1) in1++;
                    if (Math.abs(e) < 2) in2++;
                    if (buckets != null) {
                        double ar = Math.abs(tr.omega[i]);
                        int b = ar < 20 ? 0 : ar < 60 ? 1 : ar < 150 ? 2 : 3;
                        buckets[b][0]++;
                        if (Math.abs(e) < 2) buckets[b][1]++;
                        buckets[b][2] += Math.min(e * e, 900);
                    }
                }
                if (Math.abs(tr.omega[i]) < 3 && Math.abs(target - turret.position()) < 3 * tpd()) {
                    holdTime += dt;
                    if (lastVolts * volts < 0) flips++;
                }
                lastVolts = volts;
                i++;
                if (i < tr.t.length) next += tr.t[i] - tr.t[i - 1];
            }
            turret.step(sim, H);
            sim += H;
        }
        return new double[] {n > 0 ? in1 / n : 0, n > 0 ? in2 / n : 0, n > 0 ? Math.sqrt(ee / n) : 99, holdTime > 0 ? flips / holdTime : 0, limits.frozen ? 1 : 0};
    }

    //{settle s (stays within 1 deg), overshoot deg, final error deg, froze 0/1}
    static double[] step(double degrees, double[] k, int which, long seed) {

        CrServoTurret turret = plant(which, seed);
        TurretController c = controller(k);
        Limits limits = new Limits();
        limits.controller = c;
        Random loops = new Random(seed);
        double target = degrees * tpd(), t = 0, next = 0, last = 0, settled = Double.NaN, overshoot = 0;
        while (t < 3) {
            if (t >= next) {
                double position = turret.encoderPosition();
                double volts = limits.apply(t, c.update(t - last, target, 0, position), position);
                last = t;
                next += Math.max(0.012, 0.0255 + 0.0069 * loops.nextGaussian());
                turret.command(t, volts, volts);
            }
            turret.step(t, H);
            double error = (target - turret.position()) / tpd();
            overshoot = Math.max(overshoot, -Math.signum(degrees) * error);
            if (Math.abs(error) > 1) settled = Double.NaN;
            else if (Double.isNaN(settled)) settled = t;
            t += H;
        }
        return new double[] {Double.isNaN(settled) ? 3 : settled, overshoot, (target - turret.position()) / tpd(), limits.frozen ? 1 : 0};
    }

    static final double[] STEPS = {1.5, 3, 10, 30, 90, -45, 170};

    static double cost(List<Trace> trs, double[] k, StringBuilder why) {

        double total = 0;
        for (int w = 0; w < PLANTS.length; w++) {
            StringBuilder line = why == null ? null : new StringBuilder();
            total += cost1(trs, k, w, line) / PLANTS.length;
            if (why != null) why.append(System.lineSeparator()).append("      ").append(PLANTS[w]).append(": ").append(line);
        }
        return total;
    }

    static double cost1(List<Trace> trs, double[] k, int which, StringBuilder why) {

        double in1 = 0, in2 = 0, rms = 0, flips = 0, froze = 0;
        for (Trace tr : trs) {
            double[] r = replay(tr, k, which, 3);
            in1 += r[0] / trs.size();
            in2 += r[1] / trs.size();
            rms += r[2] / trs.size();
            flips += r[3] / trs.size();
            froze += r[4];
        }
        double c = 10 * (1 - in2) + 5 * (1 - in1) + 0.2 * rms + 0.3 * flips + 20 * froze;
        double settle = 0, over = 0, fin = 0;
        for (double s : STEPS) {
            double[] r = step(s, k, which, 5);
            settle += r[0] / STEPS.length;
            over = Math.max(over, r[1]);
            fin = Math.max(fin, Math.abs(r[2]));
            froze += r[3];
        }
        c += 2 * settle + 0.5 * Math.max(0, over - 1) + 20 * Math.max(0, fin - 0.5) + 20 * froze;
        if (why != null) why.append(String.format(Locale.ROOT, "matches: within 1 deg %.1f%%, within 2 deg %.1f%%, rms %.2f deg, holding volt flips %.2f/s | steps: settle %.2f s avg, worst overshoot %.2f deg, worst final %.2f deg | froze %.0f | cost %.3f", 100 * in1, 100 * in2, rms, flips, settle, over, fin, froze, c));
        return c;
    }

    static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    @Test
    public void tune() throws IOException {

        Assume.assumeTrue("set TURRET_TUNE=1", "1".equals(System.getenv("TURRET_TUNE")));
        List<Trace> trs = traces();
        Assume.assumeTrue("no traces in Sim/out/turret-traces (python -I tools/turret_fit.py export)", !trs.isEmpty());
        double[] best = current();
        StringBuilder why = new StringBuilder();
        double bestCost = cost(trs, best, why);
        System.out.println("TURRET now:" + why);
        double[] scale = {1.5, 1.5, 2, 2, 0.15, 0.15, 0.2, 0.2, 0.2, 0.3, 500, 15};
        double[] lo = {0, 0, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0, 0};
        double[] hi = {30, 30, 40, 40, 2, 2, 1, 1, 1.5, 3, 20000, 200};
        Random rnd = new Random(11);
        int rounds = Integer.parseInt(System.getenv().getOrDefault("TURRET_ROUNDS", "8"));
        for (int round = 0; round < rounds; round++) {
            for (int j = 0; j < KNOBS.length; j++) {
                for (double dir : new double[] {1, -1}) {
                    double[] k = best.clone();
                    k[j] = clamp(k[j] + dir * scale[j], lo[j], hi[j]);
                    double c = cost(trs, k, null);
                    if (c < bestCost) {
                        bestCost = c;
                        best = k;
                        scale[j] *= 1.3;
                        break;
                    }
                    if (dir < 0) scale[j] *= 0.6;
                }
            }
            for (int r = 0; r < 6; r++) {
                double[] k = best.clone();
                for (int j = 0; j < k.length; j++) k[j] = clamp(k[j] + 0.3 * scale[j] * rnd.nextGaussian(), lo[j], hi[j]);
                double c = cost(trs, k, null);
                if (c < bestCost) {
                    bestCost = c;
                    best = k;
                }
            }
            System.out.printf(Locale.ROOT, "TURRET round %d: cost %.3f%n", round, bestCost);
        }
        why.setLength(0);
        cost(trs, best, why);
        System.out.println("TURRET final:" + why);
        StringBuilder knobs = new StringBuilder("TURRET best:");
        for (int j = 0; j < KNOBS.length; j++) knobs.append(String.format(Locale.ROOT, " %s=%.4g", KNOBS[j], best[j]));
        System.out.println(knobs);
        buckets = new double[4][3];
        for (Trace tr : trs) replay(tr, best, 0, 3);
        String[] names = {"|omega| < 20", "20-60", "60-150", "> 150 deg/s"};
        for (int b = 0; b < 4; b++) System.out.printf(Locale.ROOT, "TURRET main model, %-12s %5.1f%% of loops: within 2 deg %.1f%%, rms %.2f%n", names[b], 100 * buckets[b][0] / (buckets[0][0] + buckets[1][0] + buckets[2][0] + buckets[3][0]), 100 * buckets[b][1] / buckets[b][0], Math.sqrt(buckets[b][2] / buckets[b][0]));
        buckets = null;
        for (double s : STEPS) {
            double[] r = step(s, best, 0, 5);
            double[] r1 = step(s, best, 1, 5);
            System.out.printf(Locale.ROOT, "TURRET step %5.1f deg: main model settle %.2f s, overshoot %.2f, final %.2f | sticky settle %.2f s, final %.2f%n", s, r[0], r[1], r[2], r1[0], r1[2]);
        }
    }
}
