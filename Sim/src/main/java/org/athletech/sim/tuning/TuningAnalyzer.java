package org.athletech.sim.tuning;

import org.firstinspires.ftc.teamcode.Constants.TunedConstants;
import org.firstinspires.ftc.teamcode.Tuners.FlywheelVoltageFilterFit;
import org.firstinspires.ftc.teamcode.Utility.Encoder;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public final class TuningAnalyzer {

    private static final int MAX_SESSIONS = 5;
    private static final int MIN_ROWS = 300;
    private static final double MIN_IMPROVEMENT = 0.02;

    static final class Session {
        String name;
        Map<String, String> meta = new HashMap<>();
        Map<String, double[]> columns = new LinkedHashMap<>();
        int rows;

        double[] col(String name) {

            double[] c = columns.get(name);
            return c == null ? new double[rows] : c;
        }

        boolean flag(String key) {
            return "true".equalsIgnoreCase(meta.get(key));
        }

        double number(String key, double fallback) {

            try {
                return Double.parseDouble(meta.get(key));
            } catch (RuntimeException e) {
                return fallback;
            }
        }
    }

    private final Map<String, String> values = new LinkedHashMap<>();
    private final List<String> report = new ArrayList<>();

    public static void main(String[] args) throws IOException {

        File sessionsDir = new File(args.length > 0 ? args[0] : "tuning-data/sessions");
        File constantsFile = new File(args.length > 1 ? args[1] : "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Constants/TunedConstants.java");
        File reportFile = new File(args.length > 2 ? args[2] : "tuning-data/REPORT.md");
        File autotuneDir = new File(args.length > 3 ? args[3] : "tuning-data/turret-autotune");

        List<Session> sessions = load(sessionsDir);
        File autotune = latestAutotune(autotuneDir);
        if (sessions.isEmpty() && autotune == null) {
            System.out.println("tuning: no TunerTeleOp sessions or Turret Autotuner results, TunedConstants left alone");
            return;
        }

        TuningAnalyzer analyzer = new TuningAnalyzer();
        analyzer.keepCurrent();
        if (!sessions.isEmpty()) analyzer.analyze(sessions);
        analyzer.turretModel(autotune);

        boolean changed = writeIfChanged(constantsFile, analyzer.constantsSource());
        writeIfChanged(reportFile, analyzer.reportText(sessions));
        System.out.println("tuning: " + sessions.size() + " session(s), TunedConstants " + (changed ? "UPDATED" : "unchanged") + ", see " + reportFile);
    }

    static List<Session> load(File dir) throws IOException {

        List<Session> out = new ArrayList<>();
        File[] folders = dir.listFiles();
        if (folders == null) return out;
        Arrays.sort(folders);

        for (int k = folders.length - 1; k >= 0 && out.size() < MAX_SESSIONS; k--) {
            File loop = new File(folders[k], "loop.csv");
            if (!loop.isFile()) continue;
            Session s = readSession(folders[k].getName(), loop, new File(folders[k], "meta.csv"));
            if (s.rows >= MIN_ROWS) out.add(s);
        }
        Collections.reverse(out);
        return out;
    }

    static Session readSession(String name, File loop, File meta) throws IOException {

        Session s = new Session();
        s.name = name;

        if (meta.isFile()) {
            for (String line : Files.readAllLines(meta.toPath(), StandardCharsets.UTF_8)) {
                int comma = line.indexOf(',');
                if (comma > 0) s.meta.put(line.substring(0, comma).trim(), line.substring(comma + 1).trim());
            }
        }

        List<String> lines = Files.readAllLines(loop.toPath(), StandardCharsets.UTF_8);
        if (lines.isEmpty()) return s;
        String[] header = lines.get(0).split(",");
        List<double[]> rows = new ArrayList<>();
        for (int i = 1; i < lines.size(); i++) {
            String[] parts = lines.get(i).split(",");
            if (parts.length != header.length) continue;
            double[] row = new double[parts.length];
            boolean ok = true;
            for (int c = 0; c < parts.length && ok; c++) {
                try {
                    row[c] = Double.parseDouble(parts[c]);
                } catch (NumberFormatException e) {
                    ok = false;
                }
            }
            if (ok) rows.add(row);
        }
        s.rows = rows.size();
        for (int c = 0; c < header.length; c++) {
            double[] col = new double[s.rows];
            for (int r = 0; r < s.rows; r++) col[r] = rows.get(r)[c];
            s.columns.put(header[c].trim(), col);
        }
        return s;
    }

    void analyze(List<Session> sessions) {

        StringBuilder names = new StringBuilder();
        for (Session s : sessions) names.append(names.length() == 0 ? "" : " ").append(s.name);
        put("SESSIONS", "\"" + names + "\"");

        double loopMs = loopTimes(sessions);
        voltageFilters(sessions);
        velocityFilter(sessions, "FLYWHEEL", "fw_raw", loopMs, TunedConstants.FLYWHEEL_FILTER_MIN_BANDWIDTH, TunedConstants.FLYWHEEL_FILTER_MAX_BANDWIDTH, TunedConstants.FLYWHEEL_FILTER_SCORE_MEMORY, TunedConstants.FLYWHEEL_FILTER_STALE_TIME);
        velocityFilter(sessions, "TURRET", "tu_raw", loopMs, TunedConstants.TURRET_FILTER_MIN_BANDWIDTH, TunedConstants.TURRET_FILTER_MAX_BANDWIDTH, TunedConstants.TURRET_FILTER_SCORE_MEMORY, TunedConstants.TURRET_FILTER_STALE_TIME);
        shotGate(sessions, loopMs);
        turretVibration(sessions);
    }

    //everything starts as what's in TunedConstants now, so anything not measured stays put
    void keepCurrent() {

        for (java.lang.reflect.Field field : TunedConstants.class.getFields()) {
            try {
                Object value = field.get(null);
                if (value instanceof String) put(field.getName(), "\"" + value + "\"");
                else if (value instanceof Double) put(field.getName(), fmt((Double) value));
            } catch (IllegalAccessException e) {
                throw new IllegalStateException(e);
            }
        }
    }

    public static File latestAutotune(File dir) throws IOException {

        File[] folders = dir.listFiles();
        if (folders == null) return null;
        Arrays.sort(folders);
        for (int k = folders.length - 1; k >= 0; k--) {
            File result = new File(folders[k], "result.csv");
            if (result.isFile() && "done".equals(readKeyValues(result).get("status"))) return folders[k];
        }
        return null;
    }

    public static Map<String, String> readKeyValues(File file) throws IOException {

        Map<String, String> out = new LinkedHashMap<>();
        for (String line : Files.readAllLines(file.toPath(), StandardCharsets.UTF_8)) {
            int comma = line.indexOf(',');
            if (comma > 0 && !out.containsKey(line.substring(0, comma).trim())) out.put(line.substring(0, comma).trim(), line.substring(comma + 1).trim());
        }
        return out;
    }

    void turretModel(File folder) throws IOException {

        if (folder == null) {
            report.add("- Turret model kept: no finished Turret Autotuner run pulled yet.");
            return;
        }
        Map<String, String> r = readKeyValues(new File(folder, "result.csv"));
        String[][] keys = {
                {"gain", "TURRET_GAIN"}, {"time_constant", "TURRET_TIME_CONSTANT"}, {"dead_time", "TURRET_DEAD_TIME"},
                {"kinetic_friction", "TURRET_KINETIC_FRICTION"}, {"breakaway_friction", "TURRET_BREAKAWAY_FRICTION"},
                {"bandwidth", "TURRET_BANDWIDTH"}, {"damping", "TURRET_DAMPING"},
                {"integral_ratio", "TURRET_INTEGRAL_RATIO"}, {"friction_scale", "TURRET_FRICTION_SCALE"}
        };
        Map<String, Double> parsed = new LinkedHashMap<>();
        for (String[] key : keys) {
            double v;
            try {
                v = Double.parseDouble(r.get(key[0]));
            } catch (RuntimeException e) {
                v = Double.NaN;
            }
            if (!finite(v) || v < 0 || (key[0].equals("gain") && v <= 0)) {
                report.add("- Turret model kept: " + folder.getName() + " has a bad " + key[0] + ".");
                return;
            }
            parsed.put(key[1], v);
        }
        for (Map.Entry<String, Double> e : parsed.entrySet()) put(e.getKey(), fmt(e.getValue()));
        put("TURRET_MAX_SPEED", fmt(positiveOrZero(r.get("max_speed"))));
        put("TURRET_MAX_ACCELERATION", fmt(positiveOrZero(r.get("max_acceleration"))));
        put("TURRET_START_DELAY", fmt(positiveOrZero(r.get("start_delay"))));
        put("TURRET_MAX_DECELERATION", fmt(positiveOrZero(r.get("max_deceleration"))));
        //what DRIVING showed rejection may soften to; a STATIONARY run hasn't tested that, so no softening till DRIVING runs
        double shake = positiveOrZero(r.get("shake_bandwidth"));
        put("TURRET_SHAKE_BANDWIDTH", fmt("driving".equals(r.get("mode")) && shake > 0 ? Math.min(shake, parsed.get("TURRET_BANDWIDTH")) : parsed.get("TURRET_BANDWIDTH")));
        put("TURRET_AUTOTUNE", "\"" + folder.getName() + "\"");
        report.add(String.format(Locale.US, "- **Turret model** from Turret Autotuner %s (%s): %.1f deg/s per V, lag %.0f ms, dead time %.0f ms, friction %.2f / %.2f V, bandwidth %.1f rad/s.", folder.getName(), r.get("mode"), parsed.get("TURRET_GAIN") / tpd(r), 1000 * parsed.get("TURRET_TIME_CONSTANT"), 1000 * parsed.get("TURRET_DEAD_TIME"), parsed.get("TURRET_KINETIC_FRICTION"), parsed.get("TURRET_BREAKAWAY_FRICTION"), parsed.get("TURRET_BANDWIDTH")));
        for (Map.Entry<String, String> e : r.entrySet()) {
            if (e.getKey().endsWith("_deg") || e.getKey().endsWith("_deg_s") || e.getKey().endsWith("_pct") || e.getKey().equals("settle_s"))
                report.add("  - " + e.getKey() + ": " + e.getValue());
        }
    }

    /// older runs didn't measure it: 0 = unknown
    static double positiveOrZero(String text) {

        try {
            double v = Double.parseDouble(text);
            return finite(v) && v > 0 ? v : 0;
        } catch (RuntimeException e) {
            return 0;
        }
    }

    static double tpd(Map<String, String> r) {

        try {
            return Double.parseDouble(r.get("ticks_per_degree"));
        } catch (RuntimeException e) {
            return 68.27;
        }
    }

    double loopTimes(List<Session> sessions) {

        List<Double> loops = new ArrayList<>();
        for (Session s : sessions) {
            double[] loop = s.col("loop_ms"), extra = s.col("extra_read_ms");
            for (int i = 0; i < s.rows; i++) {
                double v = loop[i] - extra[i];
                if (finite(v) && v > 0 && v < 1000) loops.add(v);
            }
        }
        if (loops.size() < MIN_ROWS) {
            keep("LOOP_TIME_MS", TunedConstants.LOOP_TIME_MS, "not enough loop data");
            keep("LOOP_TIME_P95_MS", TunedConstants.LOOP_TIME_P95_MS, "");
            return TunedConstants.LOOP_TIME_MS;
        }
        Collections.sort(loops);
        double median = loops.get(loops.size() / 2), p95 = loops.get((int) (loops.size() * 0.95)), p99 = loops.get((int) (loops.size() * 0.99));
        put("LOOP_TIME_MS", fmt(Math.round(median)));
        put("LOOP_TIME_P95_MS", fmt(round(p95, 1)));
        report.add(String.format(Locale.US, "- **Loop time** (logging reads removed): median %.1f ms, p95 %.1f ms, p99 %.1f ms, worst %.1f ms over %d loops.", median, p95, p99, loops.get(loops.size() - 1), loops.size()));
        return median;
    }

    void voltageFilters(List<Session> sessions) {

        FlywheelVoltageFilterFit.Result fw = bestVoltageFit(sessions, "fw_hub_volts", false);
        if (fw == null) {
            keep("FLYWHEEL_VOLTAGE_SAMPLE_PERIOD", TunedConstants.FLYWHEEL_VOLTAGE_SAMPLE_PERIOD, "no trusted voltage sag");
            keep("FLYWHEEL_VOLTAGE_FILTER_TAU", TunedConstants.FLYWHEEL_VOLTAGE_FILTER_TAU, "");
        }
        else {
            put("FLYWHEEL_VOLTAGE_SAMPLE_PERIOD", fmt(fw.samplePeriod));
            put("FLYWHEEL_VOLTAGE_FILTER_TAU", fmt(fw.filterTau));
            report.add(String.format(Locale.US, "- **Flywheel voltage filter:** period %.3f s, tau %.3f s. Tracking error %.4f V vs %.4f V with the old one. Sag seen %.2f V.", fw.samplePeriod, fw.filterTau, fw.rmsVolts, fw.currentRmsVolts, fw.sagVolts));
        }

        boolean separate = false;
        for (Session s : sessions) separate |= s.flag("separate_turret_hub");
        FlywheelVoltageFilterFit.Result tu = separate ? bestVoltageFit(sessions, "tu_hub_volts", true) : fw;
        if (tu == null) {
            keep("TURRET_VOLTAGE_SAMPLE_PERIOD", TunedConstants.TURRET_VOLTAGE_SAMPLE_PERIOD, "no trusted voltage sag");
            keep("TURRET_VOLTAGE_FILTER_TAU", TunedConstants.TURRET_VOLTAGE_FILTER_TAU, "");
        }
        else {
            put("TURRET_VOLTAGE_SAMPLE_PERIOD", fmt(tu.samplePeriod));
            put("TURRET_VOLTAGE_FILTER_TAU", fmt(tu.filterTau));
            report.add(String.format(Locale.US, "- **Turret voltage filter:** period %.3f s, tau %.3f s%s.", tu.samplePeriod, tu.filterTau, separate ? "" : " (same hub as the flywheel)"));
        }
    }

    FlywheelVoltageFilterFit.Result bestVoltageFit(List<Session> sessions, String column, boolean turret) {

        FlywheelVoltageFilterFit.Result best = null;
        for (Session s : sessions) {
            double[] t = s.col("t"), v = s.col(column), extra = s.col("extra_read_ms");
            double reads = s.flag("separate_turret_hub") ? 2 : 1;
            double readSum = 0;
            for (int i = 0; i < s.rows; i++) readSum += extra[i];
            double period = s.number(turret ? "turret_voltage_sample_period" : "flywheel_voltage_sample_period", 0.02);
            double tau = s.number(turret ? "turret_voltage_filter_tau" : "flywheel_voltage_filter_tau", 0.1);
            FlywheelVoltageFilterFit.Result r = FlywheelVoltageFilterFit.fit(t, v, s.rows, readSum / s.rows / 1000 / reads, period, tau);
            if (r != null && r.isTrusted() && (best == null || r.sagVolts > best.sagVolts)) best = r;
        }
        return best;
    }

    void velocityFilter(List<Session> sessions, String name, String column, double loopMs, double minBw, double maxBw, double memory, double stale) {

        double updatePeriod = hubUpdatePeriod(sessions, column);
        double newStale = finite(updatePeriod) ? clamp(round(2.5 * updatePeriod, 3), 0.02, 0.2) : stale;

        double current = predictionError(sessions, column, minBw, maxBw, memory, newStale);
        if (!finite(current)) {
            keepFilter(name, minBw, maxBw, memory, stale, "not enough " + column + " data");
            return;
        }

        double nyquistCap = 0.4 * 1000 / Math.max(loopMs, 1);
        double[] memories = {0.8, 0.9, 0.95, 0.98};
        double[] mins = {0.25, 0.5, 1};
        double[] maxes = {16, 32, 64};

        double best = current, bestMin = minBw, bestMax = maxBw, bestMemory = memory;
        for (double mem : memories) for (double lo : mins) for (double hiRaw : maxes) {
            double hi = Math.min(hiRaw, nyquistCap);
            if (hi <= lo * 4) continue;
            double e = predictionError(sessions, column, lo, hi, mem, newStale);
            if (finite(e) && e < best) {
                best = e;
                bestMin = lo;
                bestMax = hi;
                bestMemory = mem;
            }
        }

        if (best > current * (1 - MIN_IMPROVEMENT)) {
            bestMin = minBw;
            bestMax = maxBw;
            bestMemory = memory;
            best = current;
        }

        put(name + "_FILTER_MIN_BANDWIDTH", fmt(bestMin));
        put(name + "_FILTER_MAX_BANDWIDTH", fmt(round(bestMax, 1)));
        put(name + "_FILTER_SCORE_MEMORY", fmt(bestMemory));
        put(name + "_FILTER_STALE_TIME", fmt(newStale));
        report.add(String.format(Locale.US, "- **%s velocity filter:** %.2f-%.1f Hz, memory %.2f, stale %.3f s (hub updates every %.3f s). Next-reading error %.1f ticks/s (was %.1f).", name.toLowerCase(Locale.US), bestMin, bestMax, bestMemory, newStale, updatePeriod, Math.sqrt(best), Math.sqrt(current)));
    }

    static double hubUpdatePeriod(List<Session> sessions, String column) {

        List<Double> gaps = new ArrayList<>();
        for (Session s : sessions) {
            double[] t = s.col("t"), raw = s.col(column);
            double lastChange = Double.NaN;
            for (int i = 1; i < s.rows; i++) {
                if (raw[i] != raw[i - 1] && Math.abs(raw[i]) > 50) {
                    if (finite(lastChange)) gaps.add(t[i] - lastChange);
                    lastChange = t[i];
                }
            }
        }
        if (gaps.size() < 50) return Double.NaN;
        Collections.sort(gaps);
        return gaps.get(gaps.size() / 2);
    }

    static double predictionError(List<Session> sessions, String column, double minBw, double maxBw, double memory, double stale) {

        double sum = 0;
        long count = 0;
        for (Session s : sessions) {
            double[] t = s.col("t"), raw = s.col(column);
            Encoder filter = new Encoder(null);
            filter.setVelocityFilterParameters(minBw, maxBw, memory, stale);
            double prevT = Double.NaN, prevRaw = Double.NaN;
            for (int i = 0; i < s.rows; i++) {
                if (!finite(t[i]) || !finite(raw[i])) continue;
                if (finite(prevT) && raw[i] != prevRaw && i > 50) {
                    double predicted = filter.getFilteredVelocity() + filter.getFilteredAcceleration() * (t[i] - prevT);
                    double error = raw[i] - predicted;
                    sum += error * error;
                    count++;
                }
                filter.updateFilteredVelocity(t[i], raw[i]);
                prevT = t[i];
                prevRaw = raw[i];
            }
        }
        return count < 200 ? Double.NaN : sum / count;
    }

    void shotGate(List<Session> sessions, double loopMs) {

        double sum = 0, crossings = 0, seconds = 0;
        long count = 0;
        for (Session s : sessions) {
            double[] t = s.col("t"), target = s.col("fw_target"), vel = s.col("fw_vel"), on = s.col("shooter_on"), clear = s.col("blocker_clear");
            double targetSince = 0, lastClear = -1e9, lastError = Double.NaN, lastT = Double.NaN;
            for (int i = 1; i < s.rows; i++) {
                if (Math.abs(target[i] - target[i - 1]) > 1) targetSince = t[i];
                if (clear[i] > 0.5) lastClear = t[i];
                double error = vel[i] - target[i];
                boolean steady = on[i] > 0.5 && target[i] > 0 && t[i] - targetSince > 0.5 && t[i] - lastClear > 0.5 && Math.abs(error) < 150;
                if (!steady) {
                    lastError = Double.NaN;
                    continue;
                }
                sum += error * error;
                count++;
                if (finite(lastError)) {
                    seconds += t[i] - lastT;
                    if (Math.signum(error) != Math.signum(lastError)) crossings++;
                }
                lastError = error;
                lastT = t[i];
            }
        }

        if (count < 1000) {
            keep("SHOT_GATE_ENTER_BAND", TunedConstants.SHOT_GATE_ENTER_BAND, "not enough steady flywheel time");
            keep("SHOT_GATE_HOLD_BAND", TunedConstants.SHOT_GATE_HOLD_BAND, "");
            keep("SHOT_GATE_DWELL", TunedConstants.SHOT_GATE_DWELL, "");
            keep("FLYWHEEL_VIBRATION_RMS", TunedConstants.FLYWHEEL_VIBRATION_RMS, "");
            return;
        }

        double rms = Math.sqrt(sum / count);
        double enter = clamp(Math.round(2 * rms), 10, 150);
        double dwell = clamp(Math.round(2 * loopMs / 5) * 0.005, 0.02, 0.1);
        put("SHOT_GATE_ENTER_BAND", fmt(enter));
        put("SHOT_GATE_HOLD_BAND", fmt(2 * enter));
        put("SHOT_GATE_DWELL", fmt(round(dwell, 3)));
        put("FLYWHEEL_VIBRATION_RMS", fmt(round(rms, 1)));
        report.add(String.format(Locale.US, "- **Flywheel vibration** (shooter on, settled, not feeding): %.1f ticks/s RMS, about %.1f Hz. Shot gate bands %s / %s, dwell %.3f s.", rms, seconds > 0 ? crossings / (2 * seconds) : Double.NaN, fmt(enter), fmt(2 * enter), dwell));
    }

    void turretVibration(List<Session> sessions) {

        double sum = 0;
        long count = 0;
        for (Session s : sessions) {
            double tpd = s.number("turret_ticks_per_degree", 68.27);
            double[] t = s.col("t"), target = s.col("tu_target"), targetVel = s.col("tu_target_vel"), error = s.col("tu_error"), vel = s.col("tu_vel");
            double targetSince = 0;
            for (int i = 1; i < s.rows; i++) {
                if (Math.abs(target[i] - target[i - 1]) > 1) targetSince = t[i];
                if (Math.abs(targetVel[i]) < 100 && Math.abs(error[i]) < 2 * tpd && t[i] - targetSince > 0.5) {
                    double degPerSecond = vel[i] / tpd;
                    sum += degPerSecond * degPerSecond;
                    count++;
                }
            }
        }
        if (count < 500) {
            keep("TURRET_VIBRATION_RMS", TunedConstants.TURRET_VIBRATION_RMS, "not enough turret holding time");
            return;
        }
        double rms = Math.sqrt(sum / count);
        put("TURRET_VIBRATION_RMS", fmt(round(rms, 2)));
        report.add(String.format(Locale.US, "- **Turret vibration** (holding a still target): %.2f deg/s RMS.", rms));
    }

    void keepFilter(String name, double minBw, double maxBw, double memory, double stale, String why) {

        keep(name + "_FILTER_MIN_BANDWIDTH", minBw, why);
        keep(name + "_FILTER_MAX_BANDWIDTH", maxBw, "");
        keep(name + "_FILTER_SCORE_MEMORY", memory, "");
        keep(name + "_FILTER_STALE_TIME", stale, "");
    }

    void keep(String key, double value, String why) {

        put(key, fmt(value));
        if (!why.isEmpty()) report.add("- " + key + " kept: " + why + ".");
    }

    void put(String key, String value) {
        values.put(key, value);
    }

    String constantsSource() {

        String[][] layout = {
                {"SESSIONS", "String"}, {""},
                {"LOOP_TIME_MS"}, {""},
                {"FLYWHEEL_VOLTAGE_SAMPLE_PERIOD"}, {"FLYWHEEL_VOLTAGE_FILTER_TAU"}, {""},
                {"TURRET_VOLTAGE_SAMPLE_PERIOD"}, {"TURRET_VOLTAGE_FILTER_TAU"}, {""},
                {"FLYWHEEL_FILTER_MIN_BANDWIDTH"}, {"FLYWHEEL_FILTER_MAX_BANDWIDTH"}, {"FLYWHEEL_FILTER_SCORE_MEMORY"}, {"FLYWHEEL_FILTER_STALE_TIME"}, {""},
                {"TURRET_FILTER_MIN_BANDWIDTH"}, {"TURRET_FILTER_MAX_BANDWIDTH"}, {"TURRET_FILTER_SCORE_MEMORY"}, {"TURRET_FILTER_STALE_TIME"}, {""},
                {"SHOT_GATE_ENTER_BAND"}, {"SHOT_GATE_HOLD_BAND"}, {"SHOT_GATE_DWELL"}, {""},
                {"TURRET_GAIN"}, {"TURRET_TIME_CONSTANT"}, {"TURRET_DEAD_TIME"}, {"TURRET_KINETIC_FRICTION"}, {"TURRET_BREAKAWAY_FRICTION"},
                {"TURRET_BANDWIDTH"}, {"TURRET_DAMPING"}, {"TURRET_INTEGRAL_RATIO"}, {"TURRET_FRICTION_SCALE"}, {"TURRET_MAX_SPEED"}, {"TURRET_MAX_ACCELERATION"}, {"TURRET_START_DELAY"}, {"TURRET_MAX_DECELERATION"}, {"TURRET_SHAKE_BANDWIDTH"}, {"TURRET_AUTOTUNE", "String"}, {""},
                {"LOOP_TIME_P95_MS"}, {"FLYWHEEL_VIBRATION_RMS"}, {"TURRET_VIBRATION_RMS"}
        };

        StringBuilder b = new StringBuilder();
        b.append("package org.firstinspires.ftc.teamcode.Constants;\n\n");
        b.append("//written by :Sim:analyzeTuning from TunerTeleOp + Turret Autotuner data on every build, don't edit by hand\n");
        b.append("public class TunedConstants {\n\n");
        for (String[] line : layout) {
            if (line[0].isEmpty()) {
                b.append('\n');
                continue;
            }
            String type = line.length > 1 ? line[1] : "double";
            b.append("    public static ").append(type).append(' ').append(line[0]).append(" = ").append(values.get(line[0])).append(";\n");
        }
        b.append("}\n");
        return b.toString();
    }

    String reportText(List<Session> sessions) {

        StringBuilder b = new StringBuilder("# Tuning report\n\nFrom TunerTeleOp sessions:");
        for (Session s : sessions) b.append(' ').append(s.name).append(String.format(Locale.US, " (%d rows)", s.rows));
        b.append("\n\nWritten into `TeamCode/.../Constants/TunedConstants.java`. Flywheel gains are never changed, the turret model comes from the Turret Autotuner.\n\n");
        for (String line : report) b.append(line).append('\n');
        return b.toString();
    }

    static boolean writeIfChanged(File file, String content) throws IOException {

        byte[] bytes = content.getBytes(StandardCharsets.UTF_8);
        if (file.isFile() && Arrays.equals(Files.readAllBytes(file.toPath()), bytes)) return false;
        File parent = file.getAbsoluteFile().getParentFile();
        if (parent != null) parent.mkdirs();
        Files.write(file.toPath(), bytes);
        return true;
    }

    static String fmt(double v) {

        if (Double.isNaN(v)) return "Double.NaN";
        if (v == Math.rint(v) && Math.abs(v) < 1e9) return String.format(Locale.US, "%.0f", v);
        String s = String.format(Locale.US, "%.6g", v);
        if (s.contains("e")) return s;
        s = s.contains(".") ? s.replaceAll("0+$", "").replaceAll("\\.$", "") : s;
        return s;
    }

    static double round(double v, int digits) {

        double scale = Math.pow(10, digits);
        return Math.round(v * scale) / scale;
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }
}
