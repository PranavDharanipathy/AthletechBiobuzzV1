package org.athletech.sim.input;

import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.SourceKind;
import org.athletech.sim.tuning.TuningAnalyzer;

import java.io.File;
import java.io.PrintStream;
import java.nio.file.Path;
import java.util.Locale;
import java.util.Map;

//Turret Autotuner result.csv (the robot measured its own turret) -> turret.* as measured
public final class AutotuneImport {

    private AutotuneImport() {}

    /// path = one run's folder, its result.csv, or a folder of runs (takes the newest finished one)
    public static int run(ProfileDb db, Path path, PrintStream out, boolean force) throws Exception {

        File f = path.toFile();
        File result;
        if (f.isFile()) result = f;
        else if (new File(f, "result.csv").isFile()) result = new File(f, "result.csv");
        else {
            File latest = TuningAnalyzer.latestAutotune(f);
            if (latest == null) {
                out.println("No finished Turret Autotuner run in " + f.getAbsolutePath() + " (adb pull /sdcard/FIRST/data/TurretAutotune, or build with the robot on adb)");
                return 1;
            }
            result = new File(latest, "result.csv");
        }
        Map<String, String> r = TuningAnalyzer.readKeyValues(result);
        if (!"done".equals(r.get("status"))) {
            out.println(result + " didn't finish (status " + r.get("status") + "), nothing imported");
            return 1;
        }
        double tpd = num(r, "ticks_per_degree");
        if (!(tpd > 0)) {
            out.println(result + " has no ticks_per_degree");
            return 1;
        }
        long src = db.addSource(SourceKind.MEASURED, "Turret Autotuner " + result.getParentFile().getName(), result.getAbsolutePath());
        String[][] map = {
                {"turret.servo_time_constant_s", "time_constant"},
                {"turret.servo_dead_time_s", "dead_time"},
                {"turret.kinetic_friction_v", "kinetic_friction"},
                {"turret.breakaway_friction_v", "breakaway_friction"},
                {"turret.velocity_noise_dps", "rest_velocity_noise_deg_s"}
        };
        int written = 0;
        written += write(db, out, "turret.ticks_per_degree", tpd, src, force);
        written += write(db, out, "turret.servo_speed_dps_per_v", num(r, "gain") / tpd, src, force);
        for (String[] m : map) written += write(db, out, m[0], num(r, m[1]), src, force);
        if (num(r, "max_speed") > 0) written += write(db, out, "turret.max_speed_dps", num(r, "max_speed") / tpd, src, force);
        if (num(r, "max_deceleration") > 0) written += write(db, out, "turret.coast_decel_dps2", num(r, "max_deceleration") / tpd, src, force);
        if (num(r, "start_delay") > 0) written += write(db, out, "turret.servo_start_delay_s", num(r, "start_delay"), src, force);
        if (num(r, "max_acceleration") > 0) written += write(db, out, "turret.max_accel_dps2", num(r, "max_acceleration") / tpd, src, force);
        out.printf(Locale.ROOT, "Imported %d turret values from %s (%s mode) as measured.%n", written, result.getParentFile().getName(), r.get("mode"));
        return 0;
    }

    static int write(ProfileDb db, PrintStream out, String key, double value, long src, boolean force) throws Exception {

        if (!Double.isFinite(value)) return 0;
        ProfileDb.Write w = db.setParam(key, value, null, null, src, force);
        out.println("  " + key + " = " + org.athletech.sim.utility.Csv.fmt(value) + (w == ProfileDb.Write.WRITTEN ? "" : "  (kept existing higher-priority value)"));
        return w == ProfileDb.Write.WRITTEN ? 1 : 0;
    }

    static double num(Map<String, String> r, String key) {

        try {
            return Double.parseDouble(r.get(key));
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }
}
