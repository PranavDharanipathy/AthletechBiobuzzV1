package org.athletech.sim.undeterminables;

import org.athletech.sim.utility.Rng;
import org.athletech.sim.utility.Stats;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class Disconnects {
    private Disconnects() {
    }

    public static final String[] CAUSES = {"static discharge", "brownout", "impact", "current surge", "current drop", "motor noise", "random"};
    public static final int ESD = 0, BROWNOUT = 1, IMPACT = 2, SURGE = 3, DROP = 4, EMI = 5, RANDOM = 6;

    public static final class Mode {
        public final String id;
        public final boolean stopsRobot;
        public final double esd50J, brownoutV, impactG50, surge50, drop50, emiPerSecAt40A, randomPerSec;
        public final Distribution outageS;

        public Mode(String id, boolean stopsRobot, double esd50J, double brownoutV, double impactG50, double surge50, double drop50, double emiPerSecAt40A, double randomPerSec, Distribution outageS) {

            this.id = id;
            this.stopsRobot = stopsRobot;
            this.esd50J = esd50J;
            this.brownoutV = brownoutV;
            this.impactG50 = impactG50;
            this.surge50 = surge50;
            this.drop50 = drop50;
            this.emiPerSecAt40A = emiPerSecAt40A;
            this.randomPerSec = randomPerSec;
            this.outageS = outageS;
        }
    }

    public static final class Stress {
        public final double matchS, activeS;
        public final List<double[]> esd;
        public final List<double[]> impacts;
        public final float[] busV, current;
        public final double dt, holdS, didtWindowS;

        public Stress(double matchS, double activeS, List<double[]> esd, List<double[]> impacts, float[] busV, float[] current, double dt, double holdS, double didtWindowS) {

            this.matchS = matchS;
            this.activeS = activeS;
            this.esd = esd;
            this.impacts = impacts;
            this.busV = busV;
            this.current = current;
            this.dt = dt;
            this.holdS = holdS;
            this.didtWindowS = didtWindowS;
        }
    }

    public static final class Result {
        public final double[] expected, atLeastOne;
        public final double[][] byCause;
        public final double anyProb;
        public final List<double[]> timeline;
        public final double downtimeS, partialDowntimeS;
        public final double[] hazardPerBin;

        Result(double[] expected, double[] atLeastOne, double[][] byCause, double anyProb, List<double[]> timeline, double downtimeS, double partialDowntimeS, double[] hazardPerBin) {

            this.expected = expected;
            this.atLeastOne = atLeastOne;
            this.byCause = byCause;
            this.anyProb = anyProb;
            this.timeline = timeline;
            this.downtimeS = downtimeS;
            this.partialDowntimeS = partialDowntimeS;
            this.hazardPerBin = hazardPerBin;
        }
    }

    public static final double BIN_S = 10;

    public static double fragility(double x, double x50, double spread) {

        if (!(x > 0) || !(x50 > 0) || !Double.isFinite(x50)) return 0;
        return Stats.normalCdf(Math.log(x / x50) / Math.max(1e-6, spread));
    }

    public static List<double[]> slopePeaks(float[] current, double dt, double windowS, double floor, boolean rising) {

        List<double[]> out = new ArrayList<>();
        int w = Math.max(1, (int) Math.round(windowS / dt));
        double peak = 0, tPeak = 0;
        boolean in = false;
        for (int i = w; i < current.length; i++) {
            double s = (current[i] - current[i - w]) / (w * dt * 1000);
            if (!rising) s = -s;
            if (s > floor) {
                if (!in || s > peak) {
                    peak = s;
                    tPeak = i * dt;
                }
                in = true;
            }
            else if (in && s < floor / 2) {
                out.add(new double[]{tPeak, peak});
                in = false;
                peak = 0;
            }
        }
        if (in) out.add(new double[]{tPeak, peak});
        return out;
    }

    public static List<Double> brownouts(float[] busV, double dt, double v, double holdS) {

        List<Double> out = new ArrayList<>();
        if (!(v > 0)) return out;
        int need = Math.max(1, (int) Math.ceil(holdS / dt)), run = 0;
        boolean fired = false;
        for (int i = 0; i < busV.length; i++) {
            if (busV[i] < v) {
                run++;
                if (run >= need && !fired) {
                    out.add(i * dt);
                    fired = true;
                }
            }
            else {
                run = 0;
                fired = false;
            }
        }
        return out;
    }

    public static Result evaluate(List<Mode> modes, Stress s, double spread, Rng rng) {

        int nm = modes.size();
        double[] expected = new double[nm], atLeast = new double[nm];
        double[][] byCause = new double[nm][CAUSES.length];
        int bins = (int) Math.ceil(s.matchS / BIN_S);
        double[] hazard = new double[bins];
        List<double[]> candidates = new ArrayList<>();

        double minSurge = Double.POSITIVE_INFINITY, minDrop = Double.POSITIVE_INFINITY;
        for (Mode m : modes) {
            if (m.surge50 > 0) minSurge = Math.min(minSurge, m.surge50);
            if (m.drop50 > 0) minDrop = Math.min(minDrop, m.drop50);
        }
        List<double[]> surges = Double.isFinite(minSurge) ? slopePeaks(s.current, s.dt, s.didtWindowS, minSurge * Math.exp(-5 * spread), true) : new ArrayList<double[]>();
        List<double[]> drops = Double.isFinite(minDrop) ? slopePeaks(s.current, s.dt, s.didtWindowS, minDrop * Math.exp(-5 * spread), false) : new ArrayList<double[]>();

        double[] emiExposure = new double[bins];
        for (int i = 0; i < s.current.length; i++) {
            double r = s.current[i] / 40.0;
            int b = Math.min(bins - 1, (int) ((i * s.dt) / BIN_S));
            emiExposure[b] += r * r * s.dt;
        }

        for (int mi = 0; mi < nm; mi++) {
            Mode m = modes.get(mi);
            double logNone = 0;
            boolean certain = false;
            for (double[] e : s.esd) logNone += add(fragility(e[1], m.esd50J, spread), e[0], mi, ESD, byCause, hazard, candidates, rng);
            for (double[] e : s.impacts) logNone += add(fragility(e[1], m.impactG50, spread), e[0], mi, IMPACT, byCause, hazard, candidates, rng);
            for (double[] e : surges) logNone += add(fragility(e[1], m.surge50, spread), e[0], mi, SURGE, byCause, hazard, candidates, rng);
            for (double[] e : drops) logNone += add(fragility(e[1], m.drop50, spread), e[0], mi, DROP, byCause, hazard, candidates, rng);
            for (double t : brownouts(s.busV, s.dt, m.brownoutV, s.holdS)) {
                add(1, t, mi, BROWNOUT, byCause, hazard, candidates, rng);
                certain = true;
            }
            double lambda = 0;
            for (int b = 0; b < bins; b++) {
                double emi = (m.emiPerSecAt40A > 0 ? m.emiPerSecAt40A * emiExposure[b] : 0);
                double rnd = m.randomPerSec > 0 ? m.randomPerSec * Math.min(BIN_S, s.matchS - b * BIN_S) * s.activeS / s.matchS : 0;
                lambda += emi + rnd;
                byCause[mi][EMI] += emi;
                byCause[mi][RANDOM] += rnd;
                hazard[b] += emi + rnd;
                double tot = emi + rnd;
                double u = rng.uniform(), pk = Math.exp(-tot), cum = pk;
                int count = 0;
                while (u > cum && count < 5) {
                    count++;
                    pk *= tot / count;
                    cum += pk;
                }
                for (int c = 0; c < count; c++) candidates.add(new double[]{(b + rng.uniform()) * BIN_S, mi, emi >= rnd ? EMI : RANDOM});
            }
            double sum = 0;
            for (double v : byCause[mi]) sum += v;
            expected[mi] = sum;
            atLeast[mi] = certain ? 1 : 1 - Math.exp(logNone - lambda);
        }
        double none = 1;
        for (double p : atLeast) none *= 1 - p;

        Collections.sort(candidates, (a, b) -> Double.compare(a[0], b[0]));
        double[] downUntil = new double[nm];
        int hub = -1;
        for (int i = 0; i < nm; i++) if (modes.get(i).id.equals("control_hub_reboot")) hub = i;
        List<double[]> timeline = new ArrayList<>();
        List<double[]> full = new ArrayList<>(), partial = new ArrayList<>();
        for (double[] c : candidates) {
            int mi = (int) c[1];
            if (c[0] > s.matchS) continue;
            if (c[0] < downUntil[mi]) continue;
            if (hub >= 0 && c[0] < downUntil[hub]) continue;
            Mode m = modes.get(mi);
            double out = m.outageS.isKnown() ? Math.max(0, m.outageS.sample(rng)) : 0;
            double end = Math.min(s.matchS, c[0] + out);
            downUntil[mi] = end;
            timeline.add(new double[]{c[0], mi, end - c[0], c[2]});
            (m.stopsRobot ? full : partial).add(new double[]{c[0], end});
        }
        double fullS = unionLength(full);
        double partialS = Math.max(0, unionLength(concat(full, partial)) - fullS);
        return new Result(expected, atLeast, byCause, 1 - none, timeline, fullS, partialS, hazard);
    }

    private static double add(double p, double t, int mi, int cause, double[][] byCause, double[] hazard, List<double[]> cand, Rng rng) {

        if (p <= 0) return 0;
        byCause[mi][cause] += p;
        int b = Math.min(hazard.length - 1, Math.max(0, (int) (t / BIN_S)));
        hazard[b] += p;
        if (rng.uniform() < p) cand.add(new double[]{t, mi, cause});
        return p >= 1 ? Double.NEGATIVE_INFINITY : Math.log1p(-p);
    }

    private static List<double[]> concat(List<double[]> a, List<double[]> b) {

        List<double[]> out = new ArrayList<>(a);
        out.addAll(b);
        return out;
    }

    static double unionLength(List<double[]> intervals) {

        List<double[]> s = new ArrayList<>(intervals);
        Collections.sort(s, (a, b) -> Double.compare(a[0], b[0]));
        double total = 0, curS = Double.NaN, curE = Double.NaN;
        for (double[] iv : s) {
            if (Double.isNaN(curS) || iv[0] > curE) {
                if (!Double.isNaN(curS)) total += curE - curS;
                curS = iv[0];
                curE = iv[1];
            }
            else {
                curE = Math.max(curE, iv[1]);
            }
        }
        if (!Double.isNaN(curS)) total += curE - curS;
        return total;
    }
}
