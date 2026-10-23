package org.athletech.sim.scenarios;

import org.athletech.sim.config.RobotParams;
import org.athletech.sim.undeterminables.OdometryErrorModel;
import org.athletech.sim.utility.Rng;
import org.athletech.sim.visualization.HtmlReport;
import org.athletech.sim.visualization.SvgChart;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

/**
 * Localization drift: drives many simulated matches of random waypoint-to-waypoint driving
 * (translating, turning, pausing to score) and compares the odometry estimate with the truth.
 * Produces how the believed pose drifts with time and with distance driven.
 */
public final class LocalizationScenario {
    private LocalizationScenario() {
    }

    static final double DT = 0.01;
    static final double TIME_BIN = 1.0;
    static final double DIST_BIN = 1.0;

    /** Sums of squared errors per time / distance bin, merged across runs. */
    static final class Acc {
        final double[] posT, headT, posD;
        final long[] nT, nD;

        Acc(int timeBins, int distBins) {
            posT = new double[timeBins];
            headT = new double[timeBins];
            nT = new long[timeBins];
            posD = new double[distBins];
            nD = new long[distBins];
        }

        void merge(Acc o) {
            for (int i = 0; i < posT.length; i++) {
                posT[i] += o.posT[i];
                headT[i] += o.headT[i];
                nT[i] += o.nT[i];
            }
            for (int i = 0; i < posD.length; i++) {
                posD[i] += o.posD[i];
                nD[i] += o.nD[i];
            }
        }
    }

    /** One match of driving. */
    static Acc simulateMatch(OdometryErrorModel.Params op, double speed, double turnRateRad, double half, double matchS, Rng rng, int distBins) {
        Acc acc = new Acc((int) Math.ceil(matchS / TIME_BIN) + 1, distBins);
        double x = rng.uniform(-half, half), y = rng.uniform(-half, half), h = rng.uniform(-Math.PI, Math.PI);
        OdometryErrorModel odo = new OdometryErrorModel(op, rng, x, y, h);
        double t = 0, driven = 0, pause = 0;
        double wx = rng.uniform(-half, half), wy = rng.uniform(-half, half), wh = rng.uniform(-Math.PI, Math.PI);
        double nextSample = 0;
        int nextDistBin = 0;
        while (t < matchS) {
            double dx = 0, dy = 0, dh = 0;
            if (pause > 0) {
                pause -= DT;
            } else {
                double ex = wx - x, ey = wy - y, dist = Math.hypot(ex, ey);
                double step = Math.min(dist, speed * DT);
                if (dist > 1e-9) {
                    dx = ex / dist * step;
                    dy = ey / dist * step;
                }
                double herr = Math.IEEEremainder(wh - h, 2 * Math.PI);
                dh = Math.max(-turnRateRad * DT, Math.min(turnRateRad * DT, herr));
                if (dist < 0.01 && Math.abs(herr) < 0.01) {
                    pause = rng.uniform(0, 2.5); // stop to intake / score
                    wx = rng.uniform(-half, half);
                    wy = rng.uniform(-half, half);
                    wh = rng.uniform(-Math.PI, Math.PI);
                }
            }
            double mid = h + dh / 2;
            double fwd = Math.cos(mid) * dx + Math.sin(mid) * dy, left = -Math.sin(mid) * dx + Math.cos(mid) * dy;
            x += dx;
            y += dy;
            h += dh;
            driven += Math.hypot(dx, dy);
            odo.step(fwd, left, dh, DT, x, y, h);
            t += DT;
            if (t >= nextSample) {
                double pe2 = Math.pow(odo.estX() - x, 2) + Math.pow(odo.estY() - y, 2);
                double he = Math.toDegrees(Math.IEEEremainder(odo.estHeading() - h, 2 * Math.PI));
                int ti = (int) Math.round(t / TIME_BIN);
                if (ti < acc.posT.length) {
                    acc.posT[ti] += pe2;
                    acc.headT[ti] += he * he;
                    acc.nT[ti]++;
                }
                nextSample += TIME_BIN;
            }
            // distance bins: one sample per run, when the driven distance first reaches the bin's center
            while (nextDistBin < acc.posD.length && driven >= (nextDistBin + 0.5) * DIST_BIN) {
                acc.posD[nextDistBin] += Math.pow(odo.estX() - x, 2) + Math.pow(odo.estY() - y, 2);
                acc.nD[nextDistBin]++;
                nextDistBin++;
            }
        }
        return acc;
    }

    public static void run(SimContext ctx) throws Exception {
        RobotParams rp = new RobotParams(ctx.db);
        OdometryErrorModel.Params op = rp.odometry();
        double match = rp.opt("match.auto_s", 30) + rp.opt("match.teleop_s", 120);
        double field = rp.opt("field.size_m", 3.6575);
        double vmax = rp.opt("drive.max_speed_mps", Double.NaN), turnMax = rp.opt("drive.max_turn_rate_dps", Double.NaN);
        double speed = Double.isNaN(vmax) ? 1.0 : 0.6 * vmax;
        double turn = Math.toRadians(Double.isNaN(turnMax) ? 180 : 0.5 * turnMax);
        if (Double.isNaN(vmax)) rp.assumptions().add("drive.max_speed_mps not set (run 'sim drive --save' first): cruising at 1.0 m/s");
        rp.requireAll();
        SimContext.Saver save = ctx.saver(rp, "sim localization");
        int runs = Math.max(20, ctx.trials / 4);
        double half = field / 2 - 0.3;
        int distBins = (int) Math.ceil(speed * match / DIST_BIN) + 1;
        ctx.log.printf("Localization sim  (%d simulated matches of %.0f s, cruising %.2f m/s)%n", runs, match, speed);

        Acc total = IntStream.range(0, runs).parallel()
                .mapToObj(i -> simulateMatch(op, speed, turn, half, match, new Rng(ctx.seed * 7_919L + i), distBins))
                .reduce(new Acc((int) Math.ceil(match / TIME_BIN) + 1, distBins), (a, b) -> {
                    Acc m = new Acc(a.posT.length, a.posD.length);
                    m.merge(a);
                    m.merge(b);
                    return m;
                });
        List<double[]> posVsT = new ArrayList<>(), headVsT = new ArrayList<>(), posVsD = new ArrayList<>();
        double meanSq = 0, headSq = 0;
        long meanN = 0;
        for (int i = 1; i < total.posT.length; i++) {
            if (total.nT[i] == 0) continue;
            double rms = Math.sqrt(total.posT[i] / total.nT[i]);
            posVsT.add(new double[]{i * TIME_BIN, rms, Double.NaN, total.nT[i]});
            headVsT.add(new double[]{i * TIME_BIN, Math.sqrt(total.headT[i] / total.nT[i]), Double.NaN, total.nT[i]});
            meanSq += total.posT[i];
            headSq += total.headT[i];
            meanN += total.nT[i];
        }
        for (int i = 0; i < total.posD.length; i++) {
            if (total.nD[i] < Math.max(5, runs / 5)) continue; // too few runs got this far
            posVsD.add(new double[]{(i + 0.5) * DIST_BIN, Math.sqrt(total.posD[i] / total.nD[i]), Double.NaN, total.nD[i]});
        }
        double endPos = posVsT.isEmpty() ? Double.NaN : posVsT.get(posVsT.size() - 1)[1];
        double endHead = headVsT.isEmpty() ? Double.NaN : headVsT.get(headVsT.size() - 1)[1];
        // per-axis 1-sigma error at a random moment of the match (time-averaged mean square / 2 axes)
        double axisSigma = meanN == 0 ? Double.NaN : Math.sqrt(meanSq / meanN / 2);
        double headSigma = meanN == 0 ? Double.NaN : Math.sqrt(headSq / meanN);
        ctx.log.printf("  after a full match: %.3f m position error, %.2f deg heading error (RMS); typical %.3f m per axis%n", endPos, endHead, axisSigma);

        save.curve("localization.position_error_vs_time", posVsT);
        save.curve("localization.heading_error_vs_time", headVsT);
        save.curve("localization.position_error_vs_distance", posVsD);
        save.param("localization.position_error_after_match_m", endPos);
        save.param("localization.heading_error_after_match_deg", endHead);
        save.dist("localization.pose_error_m", "normal", 0, axisSigma);
        save.dist("localization.heading_error_deg", "normal", 0, headSigma);

        HtmlReport r = new HtmlReport("Localization drift sim");
        Reports.provenance(r, rp, ctx);
        r.paragraph(runs + " simulated matches of random driving between waypoints at " + Reports.round(speed) + " m/s with pauses. "
                + "Errors are RMS over all matches. Vision fixes: " + (op.visionIntervalS() > 0 ? "every " + op.visionIntervalS() + " s" : "none") + ".");
        r.table(Arrays.asList("quantity", "value", "unit"), Arrays.asList(
                Reports.row("position error after a full match", endPos, "m"),
                Reports.row("heading error after a full match", endHead, "deg"),
                Reports.row("typical error per axis at a random moment", axisSigma, "m"),
                Reports.row("typical heading error at a random moment", headSigma, "deg")));
        r.chart(new SvgChart("Position error through a match", "time (s)", "m").zeroBased().add(Reports.series("RMS", posVsT)));
        r.chart(new SvgChart("Heading error through a match", "time (s)", "deg").zeroBased().add(Reports.series("RMS", headVsT)));
        r.chart(new SvgChart("Position error vs distance driven", "distance driven (m)", "m").zeroBased().add(Reports.series("RMS", posVsD)));
        r.write(ctx.outDir.resolve("localization/report.html"));
        ctx.log.println("  report: " + ctx.outDir.resolve("localization/report.html"));
        save.report();
    }
}
