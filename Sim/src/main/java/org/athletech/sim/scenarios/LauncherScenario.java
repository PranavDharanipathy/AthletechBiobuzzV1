package org.athletech.sim.scenarios;

import org.athletech.sim.config.Catalog;
import org.athletech.sim.config.RobotParams;
import org.athletech.sim.constants.Physics;
import org.athletech.sim.field.Aperture;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.physics.Projectile;
import org.athletech.sim.predict.AimSolver;
import org.athletech.sim.predict.ShotPredictor;
import org.athletech.sim.robot.Flywheel;
import org.athletech.sim.robot.Launcher;
import org.athletech.sim.robot.Turret;
import org.athletech.sim.telemetry.Recorder;
import org.athletech.sim.utility.Rng;
import org.athletech.sim.utility.Stats;
import org.athletech.sim.utility.Vec2;
import org.athletech.sim.utility.Vec3;
import org.athletech.sim.visualization.HtmlReport;
import org.athletech.sim.visualization.SvgChart;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

/**
 * Launcher sims: flywheel spin-up and recovery (with battery sag), the shooting lookup table
 * (hood + rpm per distance), Monte Carlo make probabilities for single shots and full bursts, and
 * how a draining battery changes them. Fills the launcher.* derived entries.
 */
public final class LauncherScenario {
    private LauncherScenario() {
    }

    static final double DT = 0.001;
    static final double[] VOLTS = {11.5, 12.0, 12.5, 13.0, 13.5};

    /** Spin-up from {@code startRpm} to {@code target}: time until within tolerance (and staying there 0.1 s). */
    static double spinUp(Launcher.Params lp, Battery b, double other, double startRpm, double target, Recorder rec) {
        Flywheel f = new Flywheel(lp.flywheel());
        f.setRpm(startRpm);
        f.setTargetRpm(target);
        double t = 0, inSince = Double.NaN;
        while (t < 8) {
            f.step(DT, b, other);
            t += DT;
            if (rec != null) rec.sample(t, f.rpm(), f.motorCurrent(), f.batteryCurrent(), f.busVolts());
            boolean in = Math.abs(f.rpm() - target) <= lp.readyToleranceRpm();
            if (in && Double.isNaN(inSince)) inSince = t;
            if (!in) inSince = Double.NaN;
            if (!Double.isNaN(inSince) && t - inSince >= 0.1 && rec == null) return inSince;
        }
        return Double.isNaN(inSince) ? Double.NaN : inSince;
    }

    /** Rpm drop of one shot at {@code rpm} and the time to recover within tolerance. */
    static double[] shotAndRecover(Launcher.Params lp, Battery b, double other, String ball, double rpm) {
        Flywheel f = new Flywheel(lp.flywheel());
        f.setTargetRpm(rpm);
        f.setRpm(rpm);
        for (int i = 0; i < 500; i++) f.step(DT, b, other); // settle at target under the controller
        double before = f.rpm();
        Launcher.BallFit fit = lp.ball(ball);
        f.fire(fit.ball().massKg(), fit.exitSpeedRatio(), fit.energyLossFactor());
        double drop = before - f.rpm();
        // recovered = 90% of the lost speed back (measured against the pre-shot speed, which can sit a
        // little below target on a weak battery because of controller steady-state error)
        double t = 0, done = Math.max(1, 0.1 * drop);
        while (t < 8 && before - f.rpm() > done) {
            f.step(DT, b, other);
            t += DT;
        }
        return new double[]{drop, t};
    }

    /** Fires {@code count} balls back to back from a flywheel at speed; returns rpm before each shot and total time. */
    static double[] burst(Launcher.Params lp, Battery b, double other, String ball, double target, int count) {
        Flywheel f = new Flywheel(lp.flywheel());
        f.setTargetRpm(target);
        f.setRpm(target);
        for (int i = 0; i < 300; i++) f.step(DT, b, other);
        Launcher.BallFit fit = lp.ball(ball);
        double[] out = new double[count + 1];
        double t = 0, lastShot = Double.NEGATIVE_INFINITY;
        int fired = 0;
        while (fired < count) {
            boolean intervalOk = t - lastShot >= lp.feedIntervalS();
            // after 10 s of waiting for one ball give up on "ready" (a weak battery may never reach tolerance)
            boolean ready = !lp.waitForReady() || Math.abs(f.rpm() - target) <= lp.readyToleranceRpm()
                    || t - Math.max(lastShot, 0) > 10;
            if (intervalOk && ready) {
                out[fired] = f.rpm();
                f.fire(fit.ball().massKg(), fit.exitSpeedRatio(), fit.energyLossFactor());
                if (fired == 0) out[count] = -t;
                lastShot = t;
                fired++;
                continue;
            }
            f.step(DT, b, other);
            t += DT;
        }
        out[count] = lastShot + out[count]; // time from first to last ball
        return out;
    }

    public static void run(SimContext ctx) throws Exception {
        RobotParams rp = new RobotParams(ctx.db);
        Launcher.Params lp = rp.launcher();
        Aperture cell = rp.cell();
        rp.battery(Double.NaN);
        double rInt = rp.req("battery.internal_resistance_ohm"), drop = rp.opt("battery.voltage_drop_per_ah", 0.6);
        java.util.function.DoubleFunction<Battery> bat = v -> new Battery(v, rInt, drop); // no DB access: safe in parallel
        double other = rp.electronicsCurrent();
        int[] held = new int[Catalog.BALLS.size()];
        for (int i = 0; i < held.length; i++) held[i] = (int) Math.round(rp.req("robot.max_" + Catalog.BALLS.get(i) + "_held"));
        ShotPredictor.Errors errors = errors(rp);
        Turret turret = rp.turret();
        double[] dists = distances(Math.sqrt(2) * rp.opt("field.size_m", 3.6575));
        rp.requireAll();
        double voc = ctx.db.value("battery.open_circuit_v");
        Launcher L = new Launcher(lp);
        SimContext.Saver save = ctx.saver(rp, "sim launcher");
        HtmlReport r = new HtmlReport("Launcher sim");
        Reports.provenance(r, rp, ctx);
        ctx.log.println("Launcher sim  (flywheel free speed " + Math.round(lp.flywheel().freeSpeedRpm()) + " rpm, max " + lp.maxRpm() + " rpm, battery " + voc + " V)");

        // ---- spin-up
        Recorder spin = new Recorder(0.01, "t_s", "rpm", "motor_current_a", "battery_current_a", "bus_v");
        spinUp(lp, bat.apply(voc), other, 0, lp.maxRpm(), spin);
        double spinT = spinUp(lp, bat.apply(voc), other, 0, lp.maxRpm(), null);
        ctx.log.printf("  spin-up to %.0f rpm: %.2f s%n", lp.maxRpm(), spinT);
        List<double[]> spinCurve = new ArrayList<>(), spinCur = new ArrayList<>();
        double[] st = spin.column("t_s"), srpm = spin.column("rpm"), scur = spin.column("motor_current_a");
        for (int i = 0; i < st.length && st[i] <= Math.max(1, (Double.isNaN(spinT) ? 4 : spinT) * 1.5); i += 2) {
            spinCurve.add(new double[]{st[i], srpm[i]});
            spinCur.add(new double[]{st[i], scur[i]});
        }
        List<double[]> spinVsV = new ArrayList<>();
        for (double v : VOLTS) spinVsV.add(new double[]{v, spinUp(lp, bat.apply(v), other, 0, lp.maxRpm(), null)});
        double[] battI = spin.column("battery_current_a");
        double hold = battI[battI.length - 1];
        save.param("launcher.spinup_time_s", spinT);
        save.param("launcher.hold_current_a", hold);
        save.curve("launcher.rpm_vs_time", spinCurve);
        save.curve("launcher.spinup_time_vs_battery_v", spinVsV);
        save.action("launcher.spinup_from_idle", spinT, 0, 1);
        r.heading("Flywheel");
        r.chart(new SvgChart("Spin-up to " + Math.round(lp.maxRpm()) + " rpm", "time (s)", "rpm").zeroBased().add(Reports.series("flywheel", spinCurve)));
        r.chart(new SvgChart("Launcher motor current during spin-up", "time (s)", "A").zeroBased().add(Reports.series("all launcher motors", spinCur)));
        r.chart(new SvgChart("Spin-up time vs battery", "resting voltage (V)", "s").zeroBased().add(Reports.series("spin-up", spinVsV)));

        List<List<String>> summary = new ArrayList<>();
        summary.add(Reports.row("spin-up time to max_rpm", spinT, "s"));
        summary.add(Reports.row("battery current holding max_rpm", hold, "A"));
        SvgChart recoveryChart = new SvgChart("Recovery after one shot vs battery", "resting voltage (V)", "s").zeroBased();
        SvgChart burstTimeChart = new SvgChart("Time to fire a full load vs battery", "resting voltage (V)", "s").zeroBased();
        SvgChart exitChart = new SvgChart("Exit speed vs flywheel speed", "flywheel (rpm)", "m/s").zeroBased();
        SvgChart rpmLut = new SvgChart("Shooting table: flywheel speed", "distance to CELL (m)", "rpm");
        SvgChart hoodLut = new SvgChart("Shooting table: hood angle", "distance to CELL (m)", "deg");
        SvgChart makeChart = new SvgChart("Make probability", "distance to CELL (m)", "probability").yRange(0, 1);

        for (int bi = 0; bi < Catalog.BALLS.size(); bi++) {
            String ball = Catalog.BALLS.get(bi);
            String B = ball.toUpperCase();
            int count = Math.max(1, held[bi]);

            // ---- one shot: drop and recovery
            double[] sr = shotAndRecover(lp, bat.apply(voc), other, ball, lp.maxRpm());
            List<double[]> recVsV = new ArrayList<>(), burstVsV = new ArrayList<>();
            for (double v : VOLTS) {
                recVsV.add(new double[]{v, shotAndRecover(lp, bat.apply(v), other, ball, lp.maxRpm())[1]});
                double[] bt = burst(lp, bat.apply(v), other, ball, lp.maxRpm(), count);
                burstVsV.add(new double[]{v, bt[count]});
            }
            save.param("launcher." + ball + ".rpm_drop_per_shot_rpm", sr[0]);
            save.param("launcher." + ball + ".recovery_time_s", sr[1]);
            save.curve("launcher." + ball + ".recovery_time_vs_battery_v", recVsV);
            save.curve("launcher." + ball + ".burst_time_vs_battery_v", burstVsV);
            recoveryChart.add(Reports.series(B, recVsV));
            burstTimeChart.add(Reports.series(B + " x" + count, burstVsV));
            summary.add(Reports.row(B + ": rpm lost per shot at max_rpm", sr[0], "rpm"));
            summary.add(Reports.row(B + ": recovery time", sr[1], "s"));

            List<double[]> exit = new ArrayList<>();
            for (int k = 0; k <= 20; k++) {
                double rpm = lp.maxRpm() * k / 20;
                exit.add(new double[]{rpm, L.exitSpeed(ball, rpm)});
            }
            save.curve("launcher." + ball + ".exit_speed_vs_rpm", exit);
            exitChart.add(Reports.series(B, exit));

            // ---- shooting table: arc-consistent across distances (see AimSolver.solveTable)
            java.util.function.DoubleUnaryOperator speedSigma = errors.speedSigma(L, ball);
            AimSolver.Table table = AimSolver.solveTable(L, ball, cell, dists, 0, speedSigma, errors.angleSigmaDeg());
            AimSolver.Solution[] sol = table.solutions();
            List<double[]> rpmPts = new ArrayList<>(), hoodPts = new ArrayList<>();
            for (AimSolver.Solution s : table.lookup()) {
                rpmPts.add(new double[]{s.distanceM(), s.rpm()});
                hoodPts.add(new double[]{s.distanceM(), s.hoodDeg()});
            }
            double maxRange = maxRange(L, ball, cell, dists, sol, speedSigma, errors.angleSigmaDeg());
            int reachable = 0;
            for (AimSolver.Solution s : sol) if (s.reachable()) reachable++;
            save.curve("launcher." + ball + ".target_rpm_vs_distance", rpmPts);
            save.curve("launcher." + ball + ".hood_vs_distance", hoodPts);
            save.param("launcher." + ball + ".max_range_m", maxRange);
            rpmLut.add(Reports.series(B, rpmPts));
            hoodLut.add(Reports.series(B, hoodPts));
            ctx.log.printf("  %s: %d of %d distances reachable (max range %.2f m%s); 1 shot drops %.0f rpm, recovers in %.2f s%n",
                    B, reachable, dists.length, maxRange, table.switchPoints().isEmpty() ? "" : ", " + table.switchPoints().size() / 2 + " arc switch(es)",
                    sr[0], sr[1]);

            // ---- Monte Carlo: single shot (rested) and full burst
            final int bIdx = bi;
            double[][] single = new double[dists.length][];
            double[][] burstP = new double[dists.length][];
            IntStream.range(0, dists.length).parallel().forEach(i -> {
                Rng rng = new Rng(ctx.seed * 1_000_003L + bIdx * 7919L + i);
                ShotPredictor.Prediction p = ShotPredictor.predict(L, ball, cell, sol[i], 0, Double.NaN, errors, ctx.trials, rng);
                single[i] = new double[]{dists[i], p.probability(), p.stdErr(), p.trials()};
                burstP[i] = burstMake(L, lp, bat, voc, other, ball, cell, sol[i], errors, count, ctx.trials, rng, dists[i]);
            });
            // unreachable distances stay in as 0 % so "hold the end value" readers never extrapolate a make
            List<double[]> singleList = Arrays.asList(single), burstList = Arrays.asList(burstP);
            save.curve("launcher." + ball + ".make_prob_vs_distance", singleList);
            save.curve("launcher." + ball + ".burst_make_prob_vs_distance", burstList);
            makeChart.add(Reports.series(B + " single shot", singleList, true));
            makeChart.add(Reports.series(B + " burst of " + count, burstList, true));

            // ---- burst make probability vs distance and battery (fewer trials per point)
            List<double[]> surf = new ArrayList<>();
            int sTrials = Math.max(100, ctx.trials / 3);
            for (double v : VOLTS) {
                double[][] row = new double[dists.length][];
                IntStream.range(0, dists.length).parallel().forEach(i -> {
                    Rng rng = new Rng(ctx.seed * 31L + Math.round(v * 100) * 131L + bIdx * 17L + i);
                    row[i] = burstMake(L, lp, bat, v, other, ball, cell, sol[i], errors, count, sTrials, rng, dists[i]);
                });
                for (int i = 0; i < dists.length; i++) surf.add(new double[]{dists[i], v, row[i][1], row[i][2], row[i][3]});
            }
            save.surface("launcher." + ball + ".burst_make_prob_vs_distance_battery", surf);

            r.heading(B);
            List<List<String>> lut = new ArrayList<>();
            for (int i = 0; i < dists.length; i++) {
                AimSolver.Solution s = sol[i];
                lut.add(s.reachable()
                        ? Reports.row(dists[i], s.rpm(), s.hoodDeg(), s.exitSpeed(), 100 * s.tolerance(), single[i][1], burstP[i][1])
                        : Reports.row(dists[i], "out of reach", "", "", "", "", ""));
            }
            r.table(Arrays.asList("distance m", "flywheel rpm", "hood deg", "exit speed m/s", "speed tolerance %", "P(make) single", "P(make) burst"), lut);
            for (int k = 0; k + 1 < table.switchPoints().size(); k += 2) {
                AimSolver.Solution a = table.switchPoints().get(k), b = table.switchPoints().get(k + 1);
                r.note("info", "Arc switch at " + Reports.round(0.5 * (a.distanceM() + b.distanceM())) + " m: hood " + Reports.round(a.hoodDeg())
                        + " -> " + Reports.round(b.hoodDeg()) + " deg. The table has two points 1 mm apart there, so interpolating never mixes the two arcs.");
            }
            SvgChart sc = new SvgChart(B + ": burst make probability by battery", "distance to CELL (m)", "probability").yRange(0, 1);
            for (double v : VOLTS) {
                List<double[]> line = new ArrayList<>();
                for (double[] q : surf) if (q[1] == v) line.add(new double[]{q[0], q[2]});
                sc.add(Reports.series(v + " V", line));
            }
            r.chart(sc);
            r.chart(trajectoryChart(L, ball, cell, sol));
        }
        r.heading("Shooting table and accuracy");
        r.chart(rpmLut);
        r.chart(hoodLut);
        r.chart(makeChart);
        r.chart(exitChart);
        r.chart(recoveryChart);
        r.chart(burstTimeChart);

        if (turret != null) {
            List<double[]> tt = new ArrayList<>();
            for (int a = 0; a <= 180; a += 15) tt.add(new double[]{a, turret.timeToTurn(a)});
            save.curve("turret.time_to_turn", tt);
            r.chart(new SvgChart("Turret move time", "angle (deg)", "s").zeroBased().add(Reports.series("turret", tt)));
        } else {
            r.note("warn", "turret.max_speed_dps / turret.max_accel_dps2 not set: turret timing skipped.");
        }
        r.heading("Summary");
        r.table(Arrays.asList("quantity", "value", "unit"), summary);
        r.heading("Error model used for the Monte Carlo");
        r.bullets(Arrays.asList("flywheel speed error: " + errors.rpm().describe(), "hood error: " + errors.hoodDeg().describe(),
                "exit speed variation: " + errors.exitSpeedFrac().describe(), "exit angle variation: " + errors.exitAngleDeg().describe(),
                "turret aim error: " + errors.aimDeg().describe(), "robot position error (per axis): " + errors.poseAxisM().describe(),
                "robot heading error: " + errors.headingDeg().describe(),
                "flywheel speed error from loop timing: " + (errors.loopRpm().isKnown() ? errors.loopRpm().describe()
                        : "not added (run sim loop first, or launcher.rpm_error is measured and already includes it)")));
        r.write(ctx.outDir.resolve("launcher/report.html"));
        spin.write(ctx.outDir.resolve("launcher/spinup.csv"));
        ctx.log.println("  report: " + ctx.outDir.resolve("launcher/report.html"));
        save.report();
    }

    /**
     * The shot error model from the profile. Loop-timing flywheel error (from the loop sim) is added
     * unless launcher.rpm_error is measured, because real shot logs already include it.
     */
    public static ShotPredictor.Errors errors(RobotParams rp) throws java.sql.SQLException {
        org.athletech.sim.config.ProfileDb.DistRow measured = rp.db().dist("launcher.rpm_error");
        boolean rpmMeasured = measured != null && measured.hasData() && measured.sourceKind() == org.athletech.sim.config.SourceKind.MEASURED;
        org.athletech.sim.config.ProfileDb.DistRow loop = rp.db().dist("control.flywheel_rpm_error_from_loop");
        org.athletech.sim.undeterminables.Distribution loopRpm = !rpmMeasured && loop != null && loop.hasData()
                ? rp.dist("control.flywheel_rpm_error_from_loop") : org.athletech.sim.undeterminables.Distribution.none("loop rpm", "rpm");
        return new ShotPredictor.Errors(rp.dist("launcher.rpm_error"), rp.dist("launcher.hood_error_deg"),
                rp.dist("launcher.exit_speed_variation"), rp.dist("launcher.exit_angle_variation_deg"), rp.dist("turret.aim_error_deg"),
                rp.dist("localization.pose_error_m"), rp.dist("localization.heading_error_deg"), loopRpm);
    }

    /** 0.5 m up to the field diagonal (the farthest any shot on the field can be), every 0.2 m. */
    static double[] distances(double maxM) {
        List<Double> d = new ArrayList<>();
        for (int i = 0; 0.5 + 0.2 * i <= maxM + 0.1; i++) d.add(Math.round((0.5 + 0.2 * i) * 100) / 100.0);
        return d.stream().mapToDouble(Double::doubleValue).toArray();
    }

    /**
     * Farthest scoring distance: bisection between the last reachable grid point and the next one,
     * or the grid end if everything out to the field diagonal is reachable.
     */
    static double maxRange(Launcher L, String ball, Aperture cell, double[] dists, AimSolver.Solution[] sol,
                           java.util.function.DoubleUnaryOperator speedSigma, double angleSigma) {
        int last = -1;
        for (int i = 0; i < sol.length; i++) if (sol[i].reachable()) last = i;
        if (last < 0) return Double.NaN;
        if (last == sol.length - 1) return dists[last];
        double lo = dists[last], hi = dists[last + 1];
        for (int k = 0; k < 6; k++) {
            double m = 0.5 * (lo + hi);
            if (AimSolver.solve(L, ball, cell, m, 0, speedSigma, angleSigma).reachable()) lo = m;
            else hi = m;
        }
        return lo;
    }

    /** Average make probability per ball over a full burst at battery {@code voc}: {distance, p, stdErr, n}. */
    public static double[] burstMake(Launcher L, Launcher.Params lp, java.util.function.DoubleFunction<Battery> bat, double voc, double other,
                              String ball, Aperture cell, AimSolver.Solution s, ShotPredictor.Errors errors, int count, int trials,
                              Rng rng, double dist) {
        if (!s.reachable()) return new double[]{dist, 0, 0, 0};
        double[] before = burst(lp, bat.apply(voc), other, ball, s.rpm(), count);
        long made = 0, n = 0;
        for (int k = 0; k < count; k++) {
            ShotPredictor.Prediction p = ShotPredictor.predict(L, ball, cell, s, 0, before[k], errors, Math.max(1, trials / count), rng);
            made += Math.round(p.probability() * p.trials());
            n += p.trials();
        }
        return new double[]{dist, (double) made / n, Stats.proportionStd(made, n), n};
    }

    /** Side view of the nominal trajectories at a few distances (robot on the +x side of the CELL). */
    static SvgChart trajectoryChart(Launcher L, String ball, Aperture cell, AimSolver.Solution[] sol) {
        SvgChart c = new SvgChart(ball.toUpperCase() + ": nominal trajectories (side view)", "distance from CELL center (m)", "height (m)").zeroBased();
        Launcher.Params lp = L.params();
        Projectile.Ball b = lp.ball(ball).ball();
        for (int i = 0; i < sol.length; i += 4) {
            AimSolver.Solution s = sol[i];
            if (!s.reachable()) continue;
            AimSolver.Spot spot = AimSolver.spot(lp, cell, s.distanceM(), 0);
            List<Vec3> trace = new ArrayList<>();
            Projectile.fly(b, spot.launchPoint(), Launcher.launchVelocity(spot.yawToTarget(), Math.toRadians(s.hoodDeg()), s.exitSpeed(), Vec2.ZERO),
                    cell, Physics.AIR_DENSITY, 0.002, 4, trace);
            List<double[]> pts = new ArrayList<>();
            for (int k = 0; k < trace.size(); k += 5) pts.add(new double[]{trace.get(k).x() - cell.center().x(), trace.get(k).z()});
            Vec3 last = trace.get(trace.size() - 1);
            pts.add(new double[]{last.x() - cell.center().x(), last.z()});
            c.add(Reports.series(s.distanceM() + " m", pts));
        }
        Vec3 a = cell.center().plus(cell.heightAxis().times(cell.height() / 2)), z = cell.center().minus(cell.heightAxis().times(cell.height() / 2));
        c.add(new SvgChart.Series("CELL opening", new double[]{a.x() - cell.center().x(), z.x() - cell.center().x()}, new double[]{a.z(), z.z()}));
        return c;
    }
}
