package org.athletech.sim.scenarios;

import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.RobotParams;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.predict.LoopControlSim;
import org.athletech.sim.robot.Launcher;
import org.athletech.sim.robot.MecanumDrive;
import org.athletech.sim.undeterminables.LoopTimingModel;
import org.athletech.sim.utility.Rng;
import org.athletech.sim.utility.Stats;
import org.athletech.sim.visualization.HtmlReport;
import org.athletech.sim.visualization.SvgChart;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;

/**
 * Loop-timing sim: what one loop costs (your code + every hardware read and write + randomness),
 * and what that costs the robot - turn and move settle times, overshoot, flywheel speed error at
 * the feed, odometry integration error and pose lag - at this robot's loop timing and across a
 * sweep of loop times. Fills the control.* derived entries.
 */
public final class LoopTimingScenario {
    private LoopTimingScenario() {
    }

    static final double[] LOOP_MS = {2, 3, 5, 8, 12, 16, 20, 25, 30, 40, 50, 60};
    static final double[] JITTER_FRAC = {0, 0.25, 0.5};
    static final double TURN_RAD = Math.PI / 2, DRIVE_M = 1.0, TURN_S = 2.5, DRIVE_S = 3.0;

    /** Averages of a batch of runs at one loop setting. */
    static final class Point {
        double turnSettle, turnSettleStd, turnOver, driveSettle, driveSettleStd, driveEnd, rpmStd = Double.NaN, rpmMean = Double.NaN, burstS = Double.NaN,
                odoEuler = Double.NaN, odoExp = Double.NaN, lag, unsettled;
    }

    /** Builds the robot's loop model, or explains (via rp.missing) what is needed. */
    static LoopTimingModel robotLoop(RobotParams rp, List<String> notes) {
        RobotParams probe = new RobotParams(rp.db());
        String[] core = {"control.logic_time_ms", "control.motor_read_ms", "control.motor_reads_per_loop", "control.motor_write_ms",
                "control.motor_writes_per_loop", "control.sensor_read_ms", "control.sensor_reads_per_loop"};
        for (String k : core) probe.req(k);
        if (probe.missing().isEmpty()) {
            LoopTimingModel.Params p = new LoopTimingModel.Params(rp.req("control.logic_time_ms"), rp.req("control.motor_read_ms"),
                    rp.req("control.motor_reads_per_loop"), rp.req("control.motor_write_ms"), rp.req("control.motor_writes_per_loop"),
                    rp.opt("control.servo_write_ms", 0), rp.opt("control.servo_writes_per_loop", 0),
                    rp.req("control.sensor_read_ms"), rp.req("control.sensor_reads_per_loop"),
                    rp.opt("control.other_read_ms", 0), rp.opt("control.other_reads_per_loop", 0),
                    rp.opt("control.io_latency_cv", 0.3), rp.dist("control.loop_jitter_ms"),
                    rp.opt("control.loop_spike_prob", 0), rp.dist("control.loop_spike_ms"));
            return LoopTimingModel.of(p);
        }
        try {
            ProfileDb.DistRow measured = rp.db().dist("control.loop_time_ms");
            if (measured != null && measured.hasData()) {
                notes.add("The loop's parts (control.logic_time_ms, reads, writes) are not all set, so loop times come from the measured "
                        + "control.loop_time_ms distribution, with reads taking 30% and writes 30% of each loop (assumed). "
                        + "Run profile import-monitor on sessions with events.csv to fill the parts.");
                return LoopTimingModel.fromDistribution(rp.dist("control.loop_time_ms"), 0.3, 0.3);
            }
        } catch (java.sql.SQLException e) {
            throw new IllegalStateException(e);
        }
        for (String k : core) rp.req(k); // records them as missing
        return null;
    }

    public static void run(SimContext ctx) throws Exception {
        RobotParams rp = new RobotParams(ctx.db);
        List<String> notes = new ArrayList<>();
        LoopTimingModel loop = robotLoop(rp, notes);
        MecanumDrive.Params dp = rp.drive();
        rp.battery(Double.NaN);
        double other = rp.electronicsCurrent();
        double turnTol = Math.toRadians(rp.opt("control.turn_tolerance_deg", 1.0)), driveTol = rp.opt("control.drive_tolerance_m", 0.02);
        double imuHz = rp.opt("control.imu_rate_hz", 100);
        boolean onDevice = rp.flag("localization.integrates_on_device", true);
        double devHz = rp.opt("localization.device_rate_hz", 1000);
        boolean poseExp = rp.flag("localization.pose_exponential", true);
        double hkp = rp.opt("control.heading_kp", Double.NaN), hkd = rp.opt("control.heading_kd", Double.NaN);
        double dkp = rp.opt("control.drive_kp", Double.NaN), dkd = rp.opt("control.drive_kd", Double.NaN);
        rp.requireAll();
        double voc = ctx.db.value("battery.open_circuit_v");
        double rInt = ctx.db.value("battery.internal_resistance_ohm");
        double drop = rp.opt("battery.voltage_drop_per_ah", 0.6);
        Supplier<Battery> bat = () -> new Battery(voc, rInt, drop); // no DB access: safe in parallel

        // launcher is optional here
        RobotParams lprobe = new RobotParams(ctx.db);
        Launcher.Params lprobeP = lprobe.launcher();
        Launcher.Params lp = lprobe.missing().isEmpty() && lprobeP != null ? rp.launcher() : null;
        boolean onHub = lp != null && rp.flag("launcher.velocity_on_hub", false);
        double hubHz = rp.opt("launcher.hub_velocity_hz", 50);
        int balls = lp == null ? 0 : Math.max(1, (int) Math.round(rp.opt("robot.max_pollen_held", 3)));
        if (lp == null) notes.add("Launcher inputs are incomplete: the flywheel part is skipped.");

        SimContext.Saver save = ctx.saver(rp, "sim loop");
        Rng rng = new Rng(ctx.seed * 104_729L);
        int runs = Math.max(4, ctx.trials / 100);

        // ---- the robot's loop
        double[] periods = loop.samplePeriods(new Rng(ctx.seed + 11), 20000);
        double meanMs = Stats.mean(periods), p95 = Stats.percentile(periods, 95), cv = Stats.std(periods) / meanMs;
        double timerLate = timerOvershootMs(periods);
        List<double[]> missVs = missedEventCurve(periods);
        double minPulse = minReliablePulseMs(periods, 0.01);
        double readFrac = loop.readFraction(), writeFrac = loop.writeFraction();
        ctx.log.printf("Loop timing sim  (loop %.2f ms average, %.2f ms p95, randomness %.0f%% of the average)%n", meanMs, p95, 100 * cv);

        // ---- gains: the profile's, or tuned on a fast loop
        LoopControlSim.Gains hg = Double.isNaN(hkp) ? LoopControlSim.tune(dp, bat, other, true, TURN_RAD, turnTol, imuHz, rng)
                : new LoopControlSim.Gains(hkp, Double.isNaN(hkd) ? 0 : hkd, false);
        LoopControlSim.Gains dg = Double.isNaN(dkp) ? LoopControlSim.tune(dp, bat, other, false, DRIVE_M, driveTol, onDevice ? devHz : Double.NaN, rng)
                : new LoopControlSim.Gains(dkp, Double.isNaN(dkd) ? 0 : dkd, false);
        if (hg.tuned) notes.add("control.heading_kp/kd not set: using gains tuned by the sim on a 2 ms loop (kp " + Reports.round(hg.kp) + ", kd " + Reports.round(hg.kd)
                + "). Set your real gains to see your robot's numbers.");
        if (dg.tuned) notes.add("control.drive_kp/kd not set: using tuned gains (kp " + Reports.round(dg.kp) + ", kd " + Reports.round(dg.kd) + ").");

        // ---- at the robot's own loop, and on a near-ideal loop for reference
        Point robot = point(loop, runs * 2, dp, bat, other, hg, dg, imuHz, onDevice, devHz, turnTol, driveTol, lp, balls, onHub, hubHz, ctx.seed * 7 + 1);
        Point ideal = point(LoopTimingModel.synthetic(0.5, 0, readFrac, writeFrac), runs, dp, bat, other, hg, dg, imuHz, onDevice, devHz,
                turnTol, driveTol, lp, balls, onHub, hubHz, ctx.seed * 7 + 2);
        ctx.log.printf("  90 deg turn settles in %.2f s (%.2f s on an ideal loop), 1 m move in %.2f s (%.2f s); pose is %.1f ms old when used%n",
                robot.turnSettle, ideal.turnSettle, robot.driveSettle, ideal.driveSettle, robot.lag);

        // ---- sweeps over the average loop time (randomness scaled like this robot's)
        Point[] sweep = new Point[LOOP_MS.length];
        final double cvUse = Double.isFinite(cv) ? cv : 0.2;
        IntStream.range(0, LOOP_MS.length).parallel().forEach(i -> sweep[i] = point(
                LoopTimingModel.synthetic(LOOP_MS[i], cvUse * LOOP_MS[i], readFrac, writeFrac), runs, dp, bat, other, hg, dg, imuHz,
                onDevice, devHz, turnTol, driveTol, lp, balls, onHub, hubHz, ctx.seed * 1000 + i));
        double[][] surf = new double[JITTER_FRAC.length][LOOP_MS.length];
        IntStream.range(0, JITTER_FRAC.length * LOOP_MS.length).parallel().forEach(k -> {
            int j = k / LOOP_MS.length, i = k % LOOP_MS.length;
            LoopTimingModel m = LoopTimingModel.synthetic(LOOP_MS[i], JITTER_FRAC[j] * LOOP_MS[i], readFrac, writeFrac);
            Rng r = new Rng(ctx.seed * 31 + k);
            double s = 0;
            for (int n = 0; n < runs; n++) {
                LoopControlSim.StepResult res = LoopControlSim.step(dp, bat, other, m, hg.kp, hg.kd, imuHz, true, TURN_RAD, turnTol, TURN_S, r);
                s += Double.isFinite(res.settleS) ? res.settleS : TURN_S;
            }
            surf[j][i] = s / runs;
        });

        List<double[]> turnVs = new ArrayList<>(), overVs = new ArrayList<>(), driveVs = new ArrayList<>(), endVs = new ArrayList<>(),
                rpmVs = new ArrayList<>(), burstVs = new ArrayList<>(), odoVs = new ArrayList<>(), odoEuler = new ArrayList<>(), odoExp = new ArrayList<>(), lagVs = new ArrayList<>();
        for (int i = 0; i < LOOP_MS.length; i++) {
            Point q = sweep[i];
            turnVs.add(new double[]{LOOP_MS[i], q.turnSettle, q.turnSettleStd, runs});
            overVs.add(new double[]{LOOP_MS[i], q.turnOver});
            driveVs.add(new double[]{LOOP_MS[i], q.driveSettle, q.driveSettleStd, runs});
            endVs.add(new double[]{LOOP_MS[i], q.driveEnd});
            if (Double.isFinite(q.rpmStd)) rpmVs.add(new double[]{LOOP_MS[i], q.rpmStd});
            if (Double.isFinite(q.burstS)) burstVs.add(new double[]{LOOP_MS[i], q.burstS});
            odoEuler.add(new double[]{LOOP_MS[i], q.odoEuler});
            odoExp.add(new double[]{LOOP_MS[i], q.odoExp});
            if (!onDevice) odoVs.add(new double[]{LOOP_MS[i], poseExp ? q.odoExp : q.odoEuler});
            lagVs.add(new double[]{LOOP_MS[i], q.lag});
        }
        List<double[]> surface = new ArrayList<>();
        for (int j = 0; j < JITTER_FRAC.length; j++)
            for (int i = 0; i < LOOP_MS.length; i++) surface.add(new double[]{LOOP_MS[i], JITTER_FRAC[j] * LOOP_MS[i], surf[j][i]});

        // ---- what-if: more or fewer hardware calls
        List<double[]> vsSensors = new ArrayList<>(), vsWrites = new ArrayList<>();
        LoopTimingModel.Params parts = loop.params();
        if (parts != null) {
            for (int n = 0; n <= 6; n++) vsSensors.add(new double[]{n, parts.withSensorReads(n).meanMs()});
            for (int n = 0; n <= 12; n++) vsWrites.add(new double[]{n, parts.withMotorWrites(n).meanMs()});
        }

        // ---- flywheel error from loop timing alone: robot loop vs an ideal loop
        double loopRpmMean = Double.NaN, loopRpmStd = Double.NaN;
        if (Double.isFinite(robot.rpmStd) && Double.isFinite(ideal.rpmStd)) {
            loopRpmMean = robot.rpmMean - ideal.rpmMean;
            loopRpmStd = Math.sqrt(Math.max(0, robot.rpmStd * robot.rpmStd - ideal.rpmStd * ideal.rpmStd));
            ctx.log.printf("  flywheel at the feed: %.0f rpm std (%.0f rpm on an ideal loop) -> loop timing adds %.0f rpm std, %.0f rpm bias%n",
                    robot.rpmStd, ideal.rpmStd, loopRpmStd, loopRpmMean);
        }

        save.param("control.loop_time_model_ms", meanMs);
        save.param("control.loop_time_model_p95_ms", p95);
        save.param("control.turn_settle_time_s", robot.turnSettle);
        save.param("control.turn_overshoot_deg", robot.turnOver);
        save.param("control.drive_settle_time_s", robot.driveSettle);
        save.param("control.drive_end_error_m", robot.driveEnd);
        save.param("control.pose_lag_ms", robot.lag);
        save.param("control.timer_overshoot_ms", timerLate);
        save.param("control.min_reliable_pulse_ms", minPulse);
        save.curve("control.missed_event_prob_vs_pulse_ms", missVs);
        save.param("control.flywheel_rpm_error_std_rpm", robot.rpmStd);
        save.dist("control.flywheel_rpm_error_from_loop", "normal", loopRpmMean, loopRpmStd);
        save.curve("control.loop_time_vs_sensor_reads", vsSensors);
        save.curve("control.loop_time_vs_motor_writes", vsWrites);
        save.curve("control.turn_settle_time_vs_loop_ms", turnVs);
        save.curve("control.turn_overshoot_vs_loop_ms", overVs);
        save.curve("control.drive_settle_time_vs_loop_ms", driveVs);
        save.curve("control.drive_end_error_vs_loop_ms", endVs);
        save.curve("control.flywheel_rpm_error_vs_loop_ms", rpmVs);
        save.curve("control.flywheel_burst_time_vs_loop_ms", burstVs);
        save.curve("control.odometry_error_per_m_vs_loop_ms", odoVs);
        save.curve("control.pose_lag_vs_loop_ms", lagVs);
        save.surface("control.turn_settle_time_vs_loop_jitter", surface);

        // ---- report
        HtmlReport r = new HtmlReport("Loop timing sim");
        Reports.provenance(r, rp, ctx);
        for (String n : notes) r.note("warn", n);
        r.paragraph("Every loop reads sensors, runs your code, then writes to the motors. The controllers below run once per loop with "
                + "real read and write timing against the drive and flywheel physics, so slow or uneven loops show up as slower, "
                + "less accurate moves. Each point averages " + runs + " runs.");
        r.heading("This robot's loop");
        if (parts != null) {
            List<List<String>> rows = new ArrayList<>();
            double[][] b = parts.breakdown();
            for (int i = 0; i < b.length; i++) {
                rows.add(Reports.row(LoopTimingModel.Params.BREAKDOWN_NAMES[i], Double.isNaN(b[i][0]) ? "" : Reports.round(b[i][0]),
                        b[i][1], b[i][2], 100 * b[i][2] / parts.meanMs()));
            }
            rows.add(Reports.row("random jitter + spikes", "", "", parts.meanMs() - Arrays.stream(b).mapToDouble(x -> x[2]).sum(), ""));
            r.table(Arrays.asList("part", "per loop", "ms each", "ms per loop", "% of loop"), rows);
        }
        r.table(Arrays.asList("quantity", "this robot", "ideal loop (0.5 ms)", "unit"), Arrays.asList(
                Reports.row("average loop time", meanMs, 0.5, "ms"),
                Reports.row("95th percentile loop time", p95, "", "ms"),
                Reports.row("90 deg turn settle time", robot.turnSettle, ideal.turnSettle, "s"),
                Reports.row("90 deg turn overshoot", robot.turnOver, ideal.turnOver, "deg"),
                Reports.row("1 m move settle time", robot.driveSettle, ideal.driveSettle, "s"),
                Reports.row("1 m move error, last 0.5 s of 3 s (RMS)", robot.driveEnd, ideal.driveEnd, "m"),
                Reports.row("time-based action ends late by (average)", timerLate, "", "ms"),
                Reports.row("shortest sensor pulse caught 99% of the time", minPulse, "", "ms"),
                Reports.row("pose age when used", robot.lag, ideal.lag, "ms"),
                Reports.row("flywheel rpm std at the feed", robot.rpmStd, ideal.rpmStd, "rpm"),
                Reports.row("time to fire " + balls + " balls", robot.burstS, ideal.burstS, "s"),
                Reports.row("runs that never settled", 100 * robot.unsettled, 100 * ideal.unsettled, "%")));
        SvgChart hist = new SvgChart("Loop time distribution (model)", "loop time (ms)", "share of loops").zeroBased();
        hist.add(Reports.series("loops", histogram(periods, 40)));
        r.chart(hist);
        if (!vsSensors.isEmpty()) {
            r.chart(new SvgChart("What-if: I2C sensor reads per loop", "reads per loop", "average loop (ms)").zeroBased().add(Reports.series("loop", vsSensors)));
            r.chart(new SvgChart("What-if: motor writes per loop", "writes per loop", "average loop (ms)").zeroBased().add(Reports.series("loop", vsWrites)));
        }
        r.heading("Accuracy vs loop time");
        r.chart(new SvgChart("90 deg turn settle time", "average loop (ms)", "s").zeroBased().add(Reports.series("settle", turnVs, true)));
        r.chart(new SvgChart("90 deg turn overshoot", "average loop (ms)", "deg").zeroBased().add(Reports.series("overshoot", overVs)));
        r.chart(new SvgChart("1 m move settle time", "average loop (ms)", "s").zeroBased().add(Reports.series("settle", driveVs, true)));
        r.chart(new SvgChart("1 m move error, last 0.5 s of 3 s (RMS)", "average loop (ms)", "m").zeroBased().add(Reports.series("error", endVs)));
        if (!rpmVs.isEmpty()) r.chart(new SvgChart("Flywheel speed scatter at the feed", "average loop (ms)", "rpm (std)").zeroBased().add(Reports.series(onHub ? "hub velocity loop" : "loop velocity control", rpmVs)));
        if (!burstVs.isEmpty()) r.chart(new SvgChart("Time to fire " + balls + " balls (first to last)", "average loop (ms)", "s").zeroBased().add(Reports.series("burst", burstVs)));
        r.chart(new SvgChart("Chance of missing a short sensor pulse (this robot's loop)", "pulse length (ms)", "probability").yRange(0, 1)
                .add(Reports.series("missed", missVs)));
        r.chart(new SvgChart("Pose age when the motors use it", "average loop (ms)", "ms").zeroBased().add(Reports.series("age", lagVs)));
        r.chart(new SvgChart("Odometry error if your loop integrates", "average loop (ms)", "m per m driven").zeroBased()
                .add(Reports.series("straight steps", odoEuler)).add(Reports.series("arc (pose exponential)", odoExp)));
        SvgChart js = new SvgChart("Turn settle time by loop randomness", "average loop (ms)", "s").zeroBased();
        for (int j = 0; j < JITTER_FRAC.length; j++) {
            List<double[]> line = new ArrayList<>();
            for (int i = 0; i < LOOP_MS.length; i++) line.add(new double[]{LOOP_MS[i], surf[j][i]});
            js.add(Reports.series("std = " + Math.round(100 * JITTER_FRAC[j]) + "% of the loop", line));
        }
        r.chart(js);
        r.bullets(Arrays.asList(
                "Gains used: turn kp " + Reports.round(hg.kp) + " / kd " + Reports.round(hg.kd) + (hg.tuned ? " (tuned by the sim)" : "")
                        + "; drive kp " + Reports.round(dg.kp) + " / kd " + Reports.round(dg.kd) + (dg.tuned ? " (tuned by the sim)" : ""),
                "IMU updates at " + Reports.round(imuHz) + " Hz; odometry " + (onDevice ? "integrated on the device at " + Reports.round(devHz) + " Hz" : "integrated in your loop"
                        + (poseExp ? " with arc steps" : " with straight steps")),
                "Moving at 1.5 m/s, a " + Reports.round(robot.lag) + " ms old pose puts the robot " + Reports.round(1.5 * robot.lag) + " mm further along than it thinks."));
        r.write(ctx.outDir.resolve("loop/report.html"));
        ctx.log.println("  report: " + ctx.outDir.resolve("loop/report.html"));
        save.report();
    }

    /**
     * {@code predict loop}: this robot's loop and what it costs, optionally with changed parts
     * (--logic MS, --motor-reads N, --motor-writes N, --servo-writes N, --sensor-reads N,
     * --other-reads N) or a given average loop (--loop-ms MS [--jitter-ms MS]).
     */
    public static int predict(org.athletech.sim.config.ProfileDb db, org.athletech.sim.utility.Args a, java.io.PrintStream out) throws Exception {
        RobotParams rp = new RobotParams(db);
        List<String> notes = new ArrayList<>();
        LoopTimingModel loop = robotLoop(rp, notes);
        MecanumDrive.Params dp = rp.drive();
        rp.battery(Double.NaN);
        double other = rp.electronicsCurrent();
        double turnTol = Math.toRadians(rp.opt("control.turn_tolerance_deg", 1.0)), driveTol = rp.opt("control.drive_tolerance_m", 0.02);
        double imuHz = rp.opt("control.imu_rate_hz", 100);
        boolean onDevice = rp.flag("localization.integrates_on_device", true);
        double devHz = rp.opt("localization.device_rate_hz", 1000);
        double hkp = rp.opt("control.heading_kp", Double.NaN), hkd = rp.opt("control.heading_kd", Double.NaN);
        double dkp = rp.opt("control.drive_kp", Double.NaN), dkd = rp.opt("control.drive_kd", Double.NaN);
        rp.requireAll();
        double voc = db.value("battery.open_circuit_v"), rInt = db.value("battery.internal_resistance_ohm");
        double drop = rp.opt("battery.voltage_drop_per_ah", 0.6);
        Supplier<Battery> bat = () -> new Battery(voc, rInt, drop);
        Rng rng = new Rng((long) a.num("seed", 1));
        LoopControlSim.Gains hg = Double.isNaN(hkp) ? LoopControlSim.tune(dp, bat, other, true, TURN_RAD, turnTol, imuHz, rng)
                : new LoopControlSim.Gains(hkp, Double.isNaN(hkd) ? 0 : hkd, false);
        LoopControlSim.Gains dg = Double.isNaN(dkp) ? LoopControlSim.tune(dp, bat, other, false, DRIVE_M, driveTol, onDevice ? devHz : Double.NaN, rng)
                : new LoopControlSim.Gains(dkp, Double.isNaN(dkd) ? 0 : dkd, false);

        LoopTimingModel what = null;
        LoopTimingModel.Params p = loop.params();
        String[] keys = {"logic", "motor-reads", "motor-writes", "servo-writes", "sensor-reads", "other-reads"};
        boolean changed = false;
        for (String k : keys) changed |= a.has(k);
        if (a.has("loop-ms")) {
            double ms = a.num("loop-ms", 10);
            what = LoopTimingModel.synthetic(ms, a.num("jitter-ms", 0.15 * ms), loop.readFraction(), loop.writeFraction());
        } else if (changed && p != null) {
            what = LoopTimingModel.of(new LoopTimingModel.Params(a.num("logic", p.logicMs), p.motorReadMs, a.num("motor-reads", p.motorReads),
                    p.motorWriteMs, a.num("motor-writes", p.motorWrites), p.servoWriteMs, a.num("servo-writes", p.servoWrites),
                    p.sensorReadMs, a.num("sensor-reads", p.sensorReads), p.otherReadMs, a.num("other-reads", p.otherReads),
                    p.ioCv, p.jitterMs, p.spikeProb, p.spikeMs));
        } else if (changed) {
            out.println("The loop's parts are not in the profile, so --logic/--*-reads/--*-writes cannot be changed; use --loop-ms.");
        }
        int runs = Math.max(4, a.integer("trials", 1000) / 100);
        for (String n : notes) out.println("Note: " + n);
        Point now = point(loop, runs, dp, bat, other, hg, dg, imuHz, onDevice, devHz, turnTol, driveTol, null, 0, false, 0, 11);
        double[] per = loop.samplePeriods(new Rng(3), 10000);
        out.printf(java.util.Locale.ROOT, "This robot's loop: %.2f ms average, %.2f ms p95%n", Stats.mean(per), Stats.percentile(per, 95));
        out.printf(java.util.Locale.ROOT, "  90 deg turn settles in %.2f s (overshoot %.1f deg), 1 m move in %.2f s, pose %.1f ms old when used%n",
                now.turnSettle, now.turnOver, now.driveSettle, now.lag);
        if (what != null) {
            Point w = point(what, runs, dp, bat, other, hg, dg, imuHz, onDevice, devHz, turnTol, driveTol, null, 0, false, 0, 12);
            double[] wp = what.samplePeriods(new Rng(4), 10000);
            out.printf(java.util.Locale.ROOT, "What-if loop: %.2f ms average, %.2f ms p95%n", Stats.mean(wp), Stats.percentile(wp, 95));
            out.printf(java.util.Locale.ROOT, "  90 deg turn settles in %.2f s (overshoot %.1f deg), 1 m move in %.2f s, pose %.1f ms old when used%n",
                    w.turnSettle, w.turnOver, w.driveSettle, w.lag);
        }
        if (hg.tuned || dg.tuned) out.println("  (gains tuned by the sim: set control.heading_kp/kd and control.drive_kp/kd for your robot's numbers)");
        if (!rp.placeholderInputs().isEmpty()) out.println("  uses PLACEHOLDER inputs: " + String.join(", ", rp.placeholderInputs()));
        return 0;
    }

    static Point point(LoopTimingModel m, int runs, MecanumDrive.Params dp, Supplier<Battery> bat, double other,
                       LoopControlSim.Gains hg, LoopControlSim.Gains dg, double imuHz, boolean onDevice, double devHz,
                       double turnTol, double driveTol, Launcher.Params lp, int balls, boolean onHub, double hubHz, long seed) {
        Rng rng = new Rng(seed);
        double[] ts = new double[runs], to = new double[runs], ds = new double[runs], de = new double[runs];
        int unsettled = 0;
        List<Double> rpm = new ArrayList<>();
        double burst = 0;
        int bursts = 0;
        double odoE = 0, odoX = 0;
        for (int n = 0; n < runs; n++) {
            LoopControlSim.StepResult t = LoopControlSim.step(dp, bat, other, m, hg.kp, hg.kd, imuHz, true, TURN_RAD, turnTol, TURN_S, rng);
            LoopControlSim.StepResult d = LoopControlSim.step(dp, bat, other, m, dg.kp, dg.kd, onDevice ? devHz : Double.NaN, false, DRIVE_M, driveTol, DRIVE_S, rng);
            if (!Double.isFinite(t.settleS)) unsettled++;
            if (!Double.isFinite(d.settleS)) unsettled++;
            ts[n] = Double.isFinite(t.settleS) ? t.settleS : TURN_S;
            to[n] = Math.toDegrees(t.overshoot);
            ds[n] = Double.isFinite(d.settleS) ? d.settleS : DRIVE_S;
            de[n] = d.endError;
            if (lp != null) {
                Launcher.BallFit fit = lp.ball("pollen");
                LoopControlSim.FlywheelResult fr = LoopControlSim.flywheelFeedErrors(lp.flywheel(), lp.maxRpm(), lp.readyToleranceRpm(), lp.feedIntervalS(), balls,
                        fit.ball().massKg(), fit.exitSpeedRatio(), fit.energyLossFactor(), bat, other, m, onHub, hubHz, rng);
                for (double e : fr.errors) rpm.add(e);
                if (Double.isFinite(fr.burstS)) {
                    burst += fr.burstS;
                    bursts++;
                }
            }
            odoE += LoopControlSim.odometryErrorPerM(m, false, 8, rng);
            odoX += LoopControlSim.odometryErrorPerM(m, true, 8, rng);
        }
        Point p = new Point();
        p.turnSettle = Stats.mean(ts);
        p.turnSettleStd = runs > 1 ? Stats.std(ts) : 0;
        p.turnOver = Stats.mean(to);
        p.driveSettle = Stats.mean(ds);
        p.driveSettleStd = runs > 1 ? Stats.std(ds) : 0;
        p.driveEnd = Stats.rms(de);
        p.unsettled = unsettled / (2.0 * runs);
        if (rpm.size() > 1) {
            double[] a = rpm.stream().mapToDouble(Double::doubleValue).toArray();
            p.rpmStd = Stats.std(a);
            p.rpmMean = Stats.mean(a);
        }
        if (bursts > 0) p.burstS = burst / bursts;
        p.odoEuler = odoE / runs;
        p.odoExp = odoX / runs;
        p.lag = LoopControlSim.poseLagMs(m, onDevice, devHz, rng, 4000);
        return p;
    }

    /**
     * How late a time-based action ends on average: the wait ends at the first loop after the
     * deadline, and a random moment falls in a loop of length T with probability T / E[T], leaving
     * on average T / 2 - so E[T^2] / (2 E[T]).
     */
    public static double timerOvershootMs(double[] periods) {
        double s1 = 0, s2 = 0;
        for (double t : periods) {
            s1 += t;
            s2 += t * t;
        }
        return s1 > 0 ? s2 / (2 * s1) : Double.NaN;
    }

    /**
     * Chance a pulse of length w (a beam break, a limit switch) falls entirely between two reads,
     * reads once per loop: E[max(0, T - w)] / E[T] (a random pulse start lands in a loop of length T
     * with probability T / E[T]). Sampled from 0 to the longest loops.
     */
    public static double missedEventProb(double[] periods, double w) {
        double num = 0, den = 0;
        for (double t : periods) {
            num += Math.max(0, t - w);
            den += t;
        }
        return den > 0 ? num / den : Double.NaN;
    }

    static List<double[]> missedEventCurve(double[] periods) {
        double hi = Stats.percentile(periods, 100);
        List<double[]> out = new ArrayList<>();
        int steps = 30;
        for (int i = 0; i <= steps; i++) {
            double w = hi * i / steps;
            out.add(new double[]{w, missedEventProb(periods, w)});
        }
        return out;
    }

    /** Shortest pulse missed at most {@code p} of the time (bisection; the chance falls as w grows). */
    public static double minReliablePulseMs(double[] periods, double p) {
        double lo = 0, hi = Stats.percentile(periods, 100);
        if (missedEventProb(periods, lo) <= p) return 0;
        for (int k = 0; k < 50; k++) {
            double mid = 0.5 * (lo + hi);
            if (missedEventProb(periods, mid) > p) lo = mid;
            else hi = mid;
        }
        return hi;
    }

    static List<double[]> histogram(double[] v, int bins) {
        double lo = Stats.percentile(v, 0), hi = Stats.percentile(v, 99.5);
        if (!(hi > lo)) hi = lo + 1;
        double w = (hi - lo) / bins;
        double[] c = new double[bins];
        for (double x : v) {
            int i = (int) Math.floor((x - lo) / w);
            if (i >= 0 && i < bins) c[i]++;
        }
        List<double[]> out = new ArrayList<>();
        for (int i = 0; i < bins; i++) out.add(new double[]{lo + (i + 0.5) * w, c[i] / v.length});
        return out;
    }

}
