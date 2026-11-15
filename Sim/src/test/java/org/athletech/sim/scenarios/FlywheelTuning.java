package org.athletech.sim.scenarios;

import org.athletech.sim.scenarios.FlywheelBursts.Config;
import org.athletech.sim.scenarios.FlywheelBursts.Result;
import org.firstinspires.ftc.teamcode.Constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.Systems.Flywheel.FlywheelPIDVSCoefficients;
import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.util.Locale;

//FLYWHEEL_TUNE=1 gradlew :Sim:test --tests *FlywheelTuning*: small steps on the flywheel's gains, tried on the wheel, balls, battery
//and loops the robot logs show (tools/flywheel_fit.py -> tuning-data/flywheel-fit/plant.properties) and on that wheel off every way
//the logs leave room for. Match setup (no feed gate), the gate checked after. Proposes; FlywheelConstants is changed by hand
public class FlywheelTuning {

    static final String[] KNOBS = {"kp_close", "kp_far", "kd", "max_p", "observer_gain", "observer_lag"};
    static final double[] STEPS = {0.8, 0.9, 1.1, 1.25};

    interface Change {
        void apply(Config c);
    }

    static final String[] CASES = {"as logged", "accel 130", "accel 200", "coast 0.4", "battery 11.5->10.3", "drive sags x2", "balls 100", "loops 50 ms"};
    static final Change[] CHANGES = {
            c -> {},
            c -> c.accel = 130,
            c -> c.accel = 200,
            c -> c.coast = 0.4,
            c -> {
                c.startVolts = 11.5;
                c.endVolts = 10.3;
            },
            c -> {
                c.driveAmps = 45;
                c.drivePerSecond = 1.0;
            },
            c -> c.ballDrop = 100,
            c -> c.loopMedian = 0.05
    };

    static double[] current() {

        FlywheelPIDVSCoefficients k = FlywheelConstants.FLYWHEEL_PIDVS_COEFFICIENTS;
        return new double[] {k.kpClose, k.kpFar, k.kd, k.maxP, FlywheelConstants.FLYWHEEL_OBSERVER_GAIN, FlywheelConstants.FLYWHEEL_OBSERVER_LAG};
    }

    static Result run(int scenario, double[] knobs, boolean gate) {
        return run(scenario, knobs, gate, 300, 1, Double.NaN);
    }

    static Result run(int scenario, double[] knobs, boolean gate, double seconds, long seed, double interval) {

        //FLYWHEEL_PLANT=default: the Sim's own guess at the balls (95 +- 15, 0.30 s apart) instead of the logs'
        Config c = "default".equals(System.getenv("FLYWHEEL_PLANT")) ? new Config() : Config.fitted();
        c.observer = true;
        c.gate = gate;
        c.seconds = seconds;
        c.seed = seed;
        CHANGES[scenario].apply(c);
        if (interval == interval) c.interval = interval;
        return FlywheelBursts.run(c, (f, g) -> {
            f.getCoefficients().kpClose = knobs[0];
            f.getCoefficients().kpFar = knobs[1];
            f.kd = knobs[2];
            f.maxP = knobs[3];
            f.observer.gain = knobs[4];
            f.observer.lag = knobs[5];
        }, null);
    }

    //a miss is worth 10 ticks/s of typical error; later balls are what recovery is about, so p95 counts as much as p50
    static double score(Result r) {
        return 10.0 * r.misses() / r.balls.size() * 100 + r.percentile(-1, 0.5) + r.percentile(-1, 0.95) + 0.5 * r.steadyRms();
    }

    static double total(double[] knobs, double[] perCase, int[] misses, boolean gate) {

        double sum = 0;
        for (int s = 0; s < CASES.length; s++) {
            Result r = run(s, knobs, gate);
            double x = score(r);
            if (perCase != null) perCase[s] = x;
            if (misses != null) misses[s] = r.misses();
            sum += x;
        }
        return sum;
    }

    @Test
    public void search() throws Exception {

        Assume.assumeTrue("set FLYWHEEL_TUNE=1", "1".equals(System.getenv("FLYWHEEL_TUNE")));
        double[] start = current(), best = start.clone();
        double[] startCase = new double[CASES.length];
        int[] startMisses = new int[CASES.length];
        double startScore = total(start, startCase, startMisses, false), bestScore = startScore;
        StringBuilder log = new StringBuilder();
        log.append(String.format(Locale.ROOT, "now: %s -> score %.1f%n", describe(start), startScore));
        for (int pass = 0; pass < 2; pass++) {
            for (int k = 0; k < KNOBS.length; k++) {
                for (double step : STEPS) {
                    double[] trial = best.clone();
                    trial[k] = start[k] * step;
                    if (trial[k] == best[k]) continue;
                    double s = total(trial, null, null, false);
                    if (s < bestScore) {
                        bestScore = s;
                        best = trial;
                        log.append(String.format(Locale.ROOT, "  %s x%.2f -> %.1f%n", KNOBS[k], step, s));
                    }
                }
            }
        }
        double[] bestCase = new double[CASES.length];
        int[] bestMisses = new int[CASES.length];
        total(best, bestCase, bestMisses, false);
        int[] gateBefore = new int[CASES.length], gateAfter = new int[CASES.length];
        total(start, null, gateBefore, true);
        total(best, null, gateAfter, true);
        boolean worse = false;
        log.append(String.format(Locale.ROOT, "%-20s %8s %8s %6s %6s %10s%n", "case", "score", "->", "miss", "->", "gate miss"));
        for (int s = 0; s < CASES.length; s++) {
            log.append(String.format(Locale.ROOT, "%-20s %8.1f %8.1f %6d %6d %4d -> %d%n", CASES[s], startCase[s], bestCase[s], startMisses[s], bestMisses[s], gateBefore[s], gateAfter[s]));
            worse |= bestMisses[s] > startMisses[s] || gateAfter[s] > gateBefore[s];
        }
        boolean worth = bestScore < 0.97 * startScore && !worse;
        log.append(String.format(Locale.ROOT, "best: %s -> score %.1f (%.0f%% better)%n", describe(best), bestScore, 100 * (1 - bestScore / startScore)));
        log.append(worth ? "-> worth putting in FlywheelConstants\n" : "-> keep the gains: under 3% better, or a case missed more\n");
        System.out.print(log);
        File out = new File("out/flywheel-tune.txt");
        out.getParentFile().mkdirs();
        try (Writer w = new OutputStreamWriter(new FileOutputStream(out), Charset.forName("UTF-8"))) {
            w.write(log.toString());
        }
    }

    //FLYWHEEL_CANDIDATE=kp_close,kp_far,kd,max_p,observer_gain,observer_lag: the gains now against it, 3 seeds x 15 min a case, with
    //and without the gate, and what spacing the balls further apart would do
    @Test
    public void verify() {

        String spec = System.getenv("FLYWHEEL_CANDIDATE");
        Assume.assumeTrue("set FLYWHEEL_CANDIDATE", spec != null);
        String[] parts = spec.split(",");
        double[] start = current(), cand = new double[parts.length];
        String base = System.getenv("FLYWHEEL_BASE");
        if (base != null) {
            String[] b = base.split(",");
            for (int k = 0; k < b.length; k++) start[k] = Double.parseDouble(b[k].trim());
        }
        for (int k = 0; k < parts.length; k++) cand[k] = Double.parseDouble(parts[k].trim());
        System.out.println("now:  " + describe(start));
        System.out.println("cand: " + describe(cand));
        boolean quick = "1".equals(System.getenv("FLYWHEEL_QUICK"));
        for (double interval : quick ? new double[] {Double.NaN} : new double[] {Double.NaN, 0.22, 0.30}) {
            for (int gate = 0; gate < (quick ? 1 : 2); gate++) {
                int[] tot = new int[2];
                int balls = 0;
                double[] spb = new double[2];
                StringBuilder row = new StringBuilder();
                for (int s = 0; s < CASES.length; s++) {
                    int[] m = new int[2];
                    for (long seed = 1; seed <= 3; seed++) {
                        for (int which = 0; which < 2; which++) {
                            Result r = run(s, which == 0 ? start : cand, gate == 1, 900, seed, interval);
                            m[which] += r.misses();
                            spb[which] += r.secondsPerBall();
                            if (which == 0) balls += r.balls.size();
                        }
                    }
                    tot[0] += m[0];
                    tot[1] += m[1];
                    row.append(String.format(Locale.ROOT, " %s %d->%d;", CASES[s], m[0], m[1]));
                }
                System.out.println(String.format(Locale.ROOT, "VERIFY spacing %s, %s: misses %d -> %d of ~%d (%.1f%% -> %.1f%%), %.2f -> %.2f s/ball |%s", interval == interval ? String.format(Locale.ROOT, "%.2f s", interval) : "as logged", gate == 1 ? "gate" : "no gate", tot[0], tot[1], balls, 100.0 * tot[0] / balls, 100.0 * tot[1] / balls, spb[0] / (3 * CASES.length), spb[1] / (3 * CASES.length), row));
            }
        }
    }

    static String describe(double[] k) {
        return String.format(Locale.ROOT, "kp_close %.4f, kp_far %.4f, kd %.5f, max_p %.2f, observer_gain %.3f, observer_lag %.4f", k[0], k[1], k[2], k[3], k[4], k[5]);
    }
}
