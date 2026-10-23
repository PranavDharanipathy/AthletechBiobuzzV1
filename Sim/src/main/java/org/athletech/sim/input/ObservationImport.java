package org.athletech.sim.input;

import org.athletech.sim.config.Catalog;
import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.SourceKind;
import org.athletech.sim.utility.Csv;
import org.athletech.sim.utility.Stats;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Path;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.TreeMap;
import java.util.stream.Collectors;

/**
 * Imports logged real-robot results: shots (makes and misses) and pose checks (where the robot
 * really was vs where it thought it was). After importing, the measured curves are rebuilt from
 * every observation in the profile, so they replace the sim estimates.
 *
 * <p>Templates: data/templates/shots_template.csv and poses_template.csv.
 */
public final class ObservationImport {
    private ObservationImport() {
    }

    static final double DIST_BIN = 0.25;

    public static void importShots(ProfileDb db, Path csv, PrintStream log, boolean force) throws IOException, SQLException {
        Csv.Table t = Csv.read(csv);
        for (String col : Arrays.asList("ball", "distance_m", "result")) {
            if (!t.has(col)) throw new IOException(csv + ": missing column '" + col + "' (see data/templates/shots_template.csv)");
        }
        List<ProfileDb.ShotObs> shots = new ArrayList<>();
        int line = 1;
        for (String[] r : t.rows) {
            line++;
            String ball = t.str(r, "ball").toLowerCase(Locale.ROOT);
            String result = t.str(r, "result").toLowerCase(Locale.ROOT);
            double dist = t.num(r, "distance_m");
            if (!Catalog.BALLS.contains(ball)) throw new IOException(csv + " row " + line + ": ball must be one of " + Catalog.BALLS + ", got '" + ball + "'");
            if (!ProfileDb.SHOT_RESULTS.contains(result)) throw new IOException(csv + " row " + line + ": result must be one of " + ProfileDb.SHOT_RESULTS);
            if (!Double.isFinite(dist)) throw new IOException(csv + " row " + line + ": distance_m is required");
            String target = t.str(r, "target");
            double burst = t.num(r, "shot_in_burst");
            shots.add(new ProfileDb.ShotObs(t.str(r, "recorded"), ball, target.isEmpty() ? "cell" : target, dist, opt(t, r, "bearing_deg"),
                    opt(t, r, "robot_speed_mps"), opt(t, r, "target_rpm"), opt(t, r, "actual_rpm"), opt(t, r, "hood_deg"),
                    opt(t, r, "turret_error_deg"), opt(t, r, "battery_v"), Double.isFinite(burst) ? (int) burst : null, result, t.str(r, "notes")));
        }
        long src = db.addSource(SourceKind.MEASURED, "shot log " + csv.getFileName(), csv.toAbsolutePath().toString());
        db.addShots(shots, src);
        log.println("Imported " + shots.size() + " shots from " + csv);
        rebuildShotCurves(db, src, log, force);
    }

    private static Double opt(Csv.Table t, String[] r, String col) {
        double v = t.num(r, col);
        return Double.isFinite(v) ? v : null;
    }

    /** Rebuilds the measured make-probability curves and error distributions from all shot observations. */
    static void rebuildShotCurves(ProfileDb db, long src, PrintStream log, boolean force) throws SQLException {
        List<Double> rpmErr = new ArrayList<>(), aimErr = new ArrayList<>();
        for (String ball : Catalog.BALLS) {
            List<ProfileDb.ShotObs> all = db.shots(ball, "cell");
            if (all.isEmpty()) continue;
            List<ProfileDb.ShotObs> single = all.stream().filter(s -> s.shotInBurst() == null || s.shotInBurst() <= 1).collect(Collectors.toList());
            List<ProfileDb.ShotObs> burst = all.stream().filter(s -> s.shotInBurst() != null).collect(Collectors.toList());
            // separate measured curves: a handful of real shots must not replace the sim curve
            write(db, "launcher." + ball + ".measured_make_prob_vs_distance", single, src, force, log);
            write(db, "launcher." + ball + ".measured_burst_make_prob_vs_distance", burst, src, force, log);
            for (ProfileDb.ShotObs s : all) {
                if (s.targetRpm() != null && s.actualRpm() != null) rpmErr.add(s.actualRpm() - s.targetRpm());
                if (s.turretErrorDeg() != null) aimErr.add(s.turretErrorDeg());
            }
        }
        if (rpmErr.size() >= 10) {
            double[] v = rpmErr.stream().mapToDouble(Double::doubleValue).toArray();
            db.setDist("launcher.rpm_error", "normal", Stats.mean(v), Stats.std(v), null, null, null, null, src, force);
            log.printf("  launcher.rpm_error: mean %.1f, std %.1f rpm (%d shots)%n", Stats.mean(v), Stats.std(v), v.length);
        }
        if (aimErr.size() >= 10) {
            double[] v = aimErr.stream().mapToDouble(Double::doubleValue).toArray();
            db.setDist("turret.aim_error_deg", "normal", Stats.mean(v), Stats.std(v), null, null, null, null, src, force);
            log.printf("  turret.aim_error_deg: mean %.2f, std %.2f deg (%d shots)%n", Stats.mean(v), Stats.std(v), v.length);
        }
    }

    private static void write(ProfileDb db, String curve, List<ProfileDb.ShotObs> shots, long src, boolean force, PrintStream log) throws SQLException {
        if (shots.isEmpty()) return;
        TreeMap<Long, long[]> bins = new TreeMap<>();
        for (ProfileDb.ShotObs s : shots) {
            long[] b = bins.computeIfAbsent(Math.round(s.distanceM() / DIST_BIN), k -> new long[2]);
            b[1]++;
            if (s.made()) b[0]++;
        }
        List<double[]> pts = new ArrayList<>();
        for (java.util.Map.Entry<Long, long[]> e : bins.entrySet()) {
            long k = e.getValue()[0], n = e.getValue()[1];
            if (n < 3) continue;
            pts.add(new double[]{e.getKey() * DIST_BIN, (double) k / n, Stats.proportionStd(k, n), n});
        }
        if (pts.isEmpty()) {
            log.println("  " + curve + ": not enough shots per distance yet (need 3+ in a 0.25 m bin)");
            return;
        }
        ProfileDb.Write w = db.setCurve(curve, pts, src, force);
        log.println("  " + curve + ": " + pts.size() + " distance bins from " + shots.size() + " shots" + (w == ProfileDb.Write.WRITTEN ? "" : " (kept existing)"));
    }

    public static void importPoses(ProfileDb db, Path csv, PrintStream log, boolean force) throws IOException, SQLException {
        Csv.Table t = Csv.read(csv);
        for (String col : Arrays.asList("true_x_m", "true_y_m", "est_x_m", "est_y_m")) {
            if (!t.has(col)) throw new IOException(csv + ": missing column '" + col + "' (see data/templates/poses_template.csv)");
        }
        List<ProfileDb.PoseObs> poses = new ArrayList<>();
        int line = 1;
        for (String[] r : t.rows) {
            line++;
            for (String col : Arrays.asList("true_x_m", "true_y_m", "est_x_m", "est_y_m")) {
                if (!Double.isFinite(t.num(r, col))) throw new IOException(csv + " row " + line + ": " + col + " must be a number");
            }
            poses.add(new ProfileDb.PoseObs(t.str(r, "recorded"), opt(t, r, "elapsed_s"), opt(t, r, "distance_driven_m"),
                    t.num(r, "true_x_m"), t.num(r, "true_y_m"), opt(t, r, "true_heading_deg"),
                    t.num(r, "est_x_m"), t.num(r, "est_y_m"), opt(t, r, "est_heading_deg"), t.str(r, "notes")));
        }
        long src = db.addSource(SourceKind.MEASURED, "pose log " + csv.getFileName(), csv.toAbsolutePath().toString());
        db.addPoses(poses, src);
        log.println("Imported " + poses.size() + " pose checks from " + csv);

        List<ProfileDb.PoseObs> all = db.poses();
        TreeMap<Long, Stats.Running> byDist = new TreeMap<>(), byTime = new TreeMap<>();
        for (ProfileDb.PoseObs p : all) {
            double e2 = p.positionError() * p.positionError();
            if (p.distanceDrivenM() != null) byDist.computeIfAbsent((long) Math.floor(p.distanceDrivenM() / 0.5), k -> new Stats.Running()).add(e2);
            if (p.elapsedS() != null) byTime.computeIfAbsent((long) Math.floor(p.elapsedS() / 10), k -> new Stats.Running()).add(e2);
        }
        rms(db, "localization.measured_position_error_vs_distance", byDist, 0.5, src, force, log);
        rms(db, "localization.measured_position_error_vs_time", byTime, 10, src, force, log);
    }

    private static void rms(ProfileDb db, String curve, TreeMap<Long, Stats.Running> bins, double w, long src, boolean force, PrintStream log) throws SQLException {
        List<double[]> pts = new ArrayList<>();
        for (java.util.Map.Entry<Long, Stats.Running> e : bins.entrySet()) {
            if (e.getValue().n() < 2) continue;
            pts.add(new double[]{(e.getKey() + 0.5) * w, Math.sqrt(e.getValue().mean()), Double.NaN, e.getValue().n()});
        }
        if (pts.isEmpty()) return;
        db.setCurve(curve, pts, src, force);
        log.println("  " + curve + ": " + pts.size() + " bins");
    }
}
