package org.athletech.sim.input;

import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.SourceKind;
import org.athletech.sim.utility.Csv;
import org.athletech.sim.utility.Json;
import org.athletech.sim.utility.Stats;
import org.athletech.sim.visualization.HtmlReport;
import org.athletech.sim.visualization.SvgChart;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Turns HardwareMonitor sessions (copied off the robot with
 * {@code adb pull /sdcard/FIRST/data/HardwareMonitor}) into measured profile values:
 * battery internal resistance (from voltage sag vs current), the battery sag curve, loop time,
 * current draw, and drive top speed.
 */
public final class MonitorImport {
    private MonitorImport() {
    }

    /** Default drive motor names when drive.motor_names is not set. */
    static final Pattern DRIVE_NAME = Pattern.compile("(?i)^(left|right)[_ ]?(front|back|rear)([_ ]?(motor|drive|wheel))?$"
            + "|^(front|back|rear)[_ ]?(left|right)([_ ]?(motor|drive|wheel))?$"
            + "|^(fl|fr|bl|br|lf|rf|lb|rb)([_ ]?(motor|drive))?$");

    static final class Session {
        private final String id;
        private final String opMode;
        private final Path dir;
        private final double[] t;
        private final double[] volts;
        private final double[] amps;
        private final double[] loopMs;

        Session(String id, String opMode, Path dir, double[] t, double[] volts, double[] amps, double[] loopMs) {
            this.id = id;
            this.opMode = opMode;
            this.dir = dir;
            this.t = t;
            this.volts = volts;
            this.amps = amps;
            this.loopMs = loopMs;
        }

        public String id() { return id; }
        public String opMode() { return opMode; }
        public Path dir() { return dir; }
        public double[] t() { return t; }
        public double[] volts() { return volts; }
        public double[] amps() { return amps; }
        public double[] loopMs() { return loopMs; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Session)) return false;
            Session o = (Session) other;
            return java.util.Objects.equals(id, o.id)
                    && java.util.Objects.equals(opMode, o.opMode)
                    && java.util.Objects.equals(dir, o.dir)
                    && java.util.Objects.equals(t, o.t)
                    && java.util.Objects.equals(volts, o.volts)
                    && java.util.Objects.equals(amps, o.amps)
                    && java.util.Objects.equals(loopMs, o.loopMs);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(id, opMode, dir, t, volts, amps, loopMs);
        }

        @Override
        public String toString() {
            return "Session[id=" + id + ", opMode=" + opMode + ", dir=" + dir + ", t=" + t + ", volts=" + volts + ", amps=" + amps + ", loopMs=" + loopMs + "]";
        }
    }

    public static List<Path> findSessions(Path p) throws IOException {
        List<Path> out = new ArrayList<>();
        if (Files.isRegularFile(p.resolve("system.csv"))) {
            out.add(p);
            return out;
        }
        Path root = Files.isDirectory(p.resolve("sessions")) ? p.resolve("sessions") : p;
        if (!Files.isDirectory(root)) throw new IOException("not a folder: " + p);
        try (Stream<Path> s = Files.list(root)) {
            s.filter(d -> Files.isRegularFile(d.resolve("system.csv"))).sorted().forEach(out::add);
        }
        return out;
    }

    public static void run(ProfileDb db, Path path, PrintStream log, boolean force, Path outDir) throws IOException, SQLException {
        List<Path> dirs = findSessions(path);
        if (dirs.isEmpty()) throw new IOException("no HardwareMonitor sessions (folders with system.csv) under " + path);
        List<Session> sessions = new ArrayList<>();
        for (Path d : dirs) sessions.add(load(d));
        log.println("HardwareMonitor import: " + sessions.size() + " session(s) from " + path);

        // ---- battery: per-session linear fit V = Voc - R*I
        List<List<String>> table = new ArrayList<>();
        List<Double> rs = new ArrayList<>();
        List<double[]> allPairs = new ArrayList<>();
        for (Session s : sessions) {
            List<double[]> pairs = new ArrayList<>();
            for (int i = 0; i < s.t().length; i++) {
                if (Double.isFinite(s.volts()[i]) && Double.isFinite(s.amps()[i])) pairs.add(new double[]{s.amps()[i], s.volts()[i]});
            }
            allPairs.addAll(pairs);
            double[] x = pairs.stream().mapToDouble(q -> q[0]).toArray(), y = pairs.stream().mapToDouble(q -> q[1]).toArray();
            String rTxt = "not enough current variation";
            if (x.length >= 30 && Stats.percentile(x, 95) - Stats.percentile(x, 5) >= 2) {
                Stats.Fit f = Stats.linearFit(x, y);
                double r = -f.slope();
                if (r > 0 && r < 1) {
                    rs.add(r);
                    rTxt = Csv.fmt(round(r, 4)) + " ohm (+/- " + Csv.fmt(round(f.slopeStdErr(), 2)) + ", R^2 " + Csv.fmt(round(f.r2(), 2)) + ")";
                } else {
                    rTxt = "fit rejected (" + Csv.fmt(round(r, 3)) + " ohm)";
                }
            }
            double dur = s.t().length > 0 ? s.t()[s.t().length - 1] : 0;
            table.add(Arrays.asList(s.id(), s.opMode() == null ? "" : s.opMode(), Csv.fmt(round(dur, 3)), Integer.toString(x.length), rTxt));
        }

        long src = db.addSource(SourceKind.MEASURED, "HardwareMonitor import",
                "Sessions: " + String.join(", ", sessions.stream().map(Session::id).collect(Collectors.toList())) + "\nFrom: " + path.toAbsolutePath());
        List<String> written = new ArrayList<>(), kept = new ArrayList<>();
        Writer w = new Writer(db, src, force, written, kept);

        if (!rs.isEmpty()) {
            double[] arr = rs.stream().mapToDouble(Double::doubleValue).toArray();
            double r = Stats.percentile(arr, 50);
            Double std = arr.length > 1 ? Stats.std(arr) : null;
            w.param("battery.internal_resistance_ohm", r, std);
            log.printf("  battery internal resistance: %.4f ohm (median of %d session fits)%n", r, arr.length);
        } else {
            log.println("  battery internal resistance: not enough current variation in these sessions (need a few amps of swing)");
        }
        // binned sag curve, all sessions together
        List<double[]> sag = bin(allPairs, 1.0, 5);
        w.curve("battery.voltage_vs_current", sag);

        // longest session: voltage through the match
        Session longest = sessions.stream().max((a, b) -> Double.compare(a.t().length == 0 ? 0 : a.t()[a.t().length - 1],
                b.t().length == 0 ? 0 : b.t()[b.t().length - 1])).get();
        List<double[]> vt = new ArrayList<>();
        for (int i = 0; i < longest.t().length; i++) if (Double.isFinite(longest.volts()[i])) vt.add(new double[]{longest.t()[i], longest.volts()[i]});
        List<double[]> vtBinned = bin(vt, 2.0, 1);
        w.curve("battery.voltage_vs_match_time", vtBinned);

        // end-of-match voltage from sessions that look like full matches (>= 100 s)
        List<Double> ends = new ArrayList<>();
        for (Session s : sessions) {
            if (s.t().length == 0 || s.t()[s.t().length - 1] < 100) continue;
            double[] tail = tail(s.volts(), 20);
            if (tail.length > 0) ends.add(Stats.percentile(tail, 50));
        }
        if (!ends.isEmpty()) w.param("battery.end_of_match_v", Stats.percentile(ends.stream().mapToDouble(Double::doubleValue).toArray(), 50), null);

        // current + voltage + loop time
        double[] amps = concat(sessions.stream().map(Session::amps).collect(Collectors.toList()));
        double[] volts = concat(sessions.stream().map(Session::volts).collect(Collectors.toList()));
        double[] loops = concat(sessions.stream().map(Session::loopMs).collect(Collectors.toList()));
        if (amps.length > 0) {
            w.param("power.avg_current_a", Stats.mean(amps), Stats.std(amps));
            w.param("power.peak_current_a", Stats.percentile(amps, 95), null);
        }
        if (volts.length > 0) w.param("power.min_battery_v", Stats.percentile(volts, 0), null);
        if (loops.length > 0) {
            w.param("control.loop_time_ms", Stats.mean(loops), Stats.std(loops));
            w.param("control.loop_time_p95_ms", Stats.percentile(loops, 95), null);
            double[] sample = subsample(loops, 2000);
            ProfileDb.Write r = db.setDist("control.loop_time_ms", "empirical", Stats.mean(loops), Stats.std(loops),
                    Stats.percentile(loops, 0), Stats.percentile(loops, 100), null, sample, src, force);
            (r == ProfileDb.Write.WRITTEN ? written : kept).add("control.loop_time_ms (distribution)");
            log.printf("  loop time: %.2f ms average, %.2f ms p95 (%d samples)%n", Stats.mean(loops), Stats.percentile(loops, 95), loops.length);
        }
        if (amps.length > 0) log.printf("  total current: %.1f A average, %.1f A p95%n", Stats.mean(amps), Stats.percentile(amps, 95));

        // what each loop is made of (system.csv + devices.csv + events.csv)
        LoopParts parts = loopParts(sessions, log);
        if (parts != null) parts.write(w, db, src, force, written, kept);

        // drive top speed from motor velocities
        double topSpeed = driveTopSpeed(db, sessions, log);
        if (Double.isFinite(topSpeed)) w.param("drive.measured_top_speed_mps", topSpeed, null);

        log.println("  wrote " + written.size() + " entries as measured" + (kept.isEmpty() ? "" : "; kept higher-priority values for " + String.join(", ", kept)));

        HtmlReport rep = new HtmlReport("HardwareMonitor import");
        rep.paragraph("Imported " + sessions.size() + " session(s) from " + path.toAbsolutePath() + ".");
        rep.table(Arrays.asList("session", "OpMode", "duration s", "V/I samples", "battery resistance fit"), table);
        rep.chart(new SvgChart("Battery voltage vs total current (binned)", "current (A)", "V").add(series("all sessions", sag)));
        rep.chart(new SvgChart("Battery voltage through " + longest.id(), "time (s)", "V").add(series("battery", vtBinned)));
        if (loops.length > 0) {
            List<double[]> hist = histogram(loops, 30);
            rep.chart(new SvgChart("Loop time distribution", "loop time (ms)", "share of samples").zeroBased().add(series("loops", hist)));
        }
        rep.bullets(written.stream().map(s -> "saved " + s).collect(Collectors.toList()));
        rep.write(outDir.resolve("import-monitor/report.html"));
        log.println("  report: " + outDir.resolve("import-monitor/report.html"));
    }

    /** Writes measured values, remembering what was written or kept. */
    private static final class Writer {
        private final ProfileDb db;
        private final long src;
        private final boolean force;
        private final List<String> written;
        private final List<String> kept;

        private Writer(ProfileDb db, long src, boolean force, List<String> written, List<String> kept) {
            this.db = db;
            this.src = src;
            this.force = force;
            this.written = written;
            this.kept = kept;
        }

        public ProfileDb db() { return db; }
        public long src() { return src; }
        public boolean force() { return force; }
        public List<String> written() { return written; }
        public List<String> kept() { return kept; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Writer)) return false;
            Writer o = (Writer) other;
            return java.util.Objects.equals(db, o.db)
                    && src == o.src
                    && force == o.force
                    && java.util.Objects.equals(written, o.written)
                    && java.util.Objects.equals(kept, o.kept);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(db, src, force, written, kept);
        }

        @Override
        public String toString() {
            return "Writer[db=" + db + ", src=" + src + ", force=" + force + ", written=" + written + ", kept=" + kept + "]";
        }

        void param(String key, double v, Double std) throws SQLException {
            if (!Double.isFinite(v)) return;
            ProfileDb.Write r = db.setParam(key, v, null, std, src, force);
            (r == ProfileDb.Write.WRITTEN ? written : kept).add(key);
        }

        void curve(String name, List<double[]> pts) throws SQLException {
            if (pts.isEmpty()) return;
            ProfileDb.Write r = db.setCurve(name, pts, src, force);
            (r == ProfileDb.Write.WRITTEN ? written : kept).add(name);
        }
    
    }

    /** Loop composition measured from HardwareMonitor sessions. */
    static final class LoopParts {
        double logicMs = Double.NaN, cv = Double.NaN, spikeProb = Double.NaN, jitterStd = Double.NaN, spikeMean = Double.NaN, spikeStd = Double.NaN;
        final double[] ms = new double[CLASSES.length], perLoop = new double[CLASSES.length];
        final boolean[] has = new boolean[CLASSES.length];

        void write(Writer w, ProfileDb db, long src, boolean force, List<String> written, List<String> kept) throws SQLException {
            w.param("control.logic_time_ms", logicMs, null);
            for (int k = 0; k < CLASSES.length; k++) {
                if (!has[k]) continue;
                w.param("control." + CLASSES[k] + "_ms", ms[k], null);
                w.param("control." + CLASSES[k] + "s_per_loop", perLoop[k], null);
            }
            w.param("control.io_latency_cv", cv, null);
            w.param("control.loop_spike_prob", spikeProb, null);
            if (Double.isFinite(jitterStd)) {
                ProfileDb.Write r = db.setDist("control.loop_jitter_ms", "normal", 0.0, jitterStd, null, null, null, null, src, force);
                (r == ProfileDb.Write.WRITTEN ? written : kept).add("control.loop_jitter_ms (distribution)");
            }
            if (Double.isFinite(spikeMean) && Double.isFinite(spikeStd)) {
                ProfileDb.Write r = db.setDist("control.loop_spike_ms", "lognormal", spikeMean, spikeStd, null, null, null, null, src, force);
                (r == ProfileDb.Write.WRITTEN ? written : kept).add("control.loop_spike_ms (distribution)");
            }
        }
    }

    /** Hardware call classes, matching the control.<class>_ms / control.<class>s_per_loop entries. */
    static final String[] CLASSES = {"motor_read", "motor_write", "servo_write", "sensor_read", "other_read"};

    /** Which class a call on a device of {@code type} falls in (-1 = not counted). */
    static int classOf(String type, boolean read) {
        if (type == null) return -1;
        switch (type) {
            case "DcMotor":
            case "DcMotorEx":
                return read ? 0 : 1;
            case "Servo":
            case "CRServo":
                return read ? -1 : 2;
            case "IMU":
            case "ColorSensor":
            case "DistanceSensor":
                return read ? 3 : -1;
            case "TouchSensor":
            case "DigitalChannel":
            case "AnalogInput":
            case "VoltageSensor":
                return read ? 4 : -1;
            default:
                return -1;
        }
    }

    /**
     * Logic time (loop minus hardware time), per-class call times and counts per loop (uncached
     * reads, non-repeated writes), call-time scatter, loop jitter and spikes. Null if the sessions
     * have no loop columns.
     */
    static LoopParts loopParts(List<Session> sessions, PrintStream log) throws IOException {
        List<Double> logic = new ArrayList<>(), jitter = new ArrayList<>(), spikes = new ArrayList<>();
        long loopSamples = 0, spikeCount = 0;
        List<List<Double>> lat = new ArrayList<>();
        for (int k = 0; k < CLASSES.length; k++) lat.add(new ArrayList<Double>());
        double loopsWithEvents = 0;
        for (Session s : sessions) {
            Csv.Table sys = Csv.read(s.dir().resolve("system.csv"));
            if (sys.has("loop_avg_ms") && sys.has("hw_ms_per_loop")) {
                for (String[] row : sys.rows) {
                    double l = sys.num(row, "loop_avg_ms"), h = sys.num(row, "hw_ms_per_loop");
                    if (Double.isFinite(l) && Double.isFinite(h) && l > 0) logic.add(Math.max(0, l - h));
                }
            }
            // jitter and spikes from the per-row last-loop samples, against a running median
            double[] lm = s.loopMs();
            int win = 10;
            for (int i = 0; i < lm.length; i++) {
                double[] around = Arrays.copyOfRange(lm, Math.max(0, i - win), Math.min(lm.length, i + win + 1));
                double med = Stats.percentile(around, 50);
                if (!(med > 0)) continue;
                loopSamples++;
                if (lm[i] > 2 * med) {
                    spikeCount++;
                    spikes.add(lm[i] - med);
                } else {
                    jitter.add(lm[i] - med);
                }
            }
            Path dev = s.dir().resolve("devices.csv"), ev = s.dir().resolve("events.csv");
            if (!Files.isRegularFile(dev) || !Files.isRegularFile(ev)) continue;
            java.util.Map<String, String> types = new java.util.HashMap<>();
            Csv.Table dt = Csv.read(dev);
            for (String[] row : dt.rows) types.put(dt.str(row, "name"), dt.str(row, "type"));
            Csv.Table et = Csv.read(ev);
            double first = Double.NaN, last = Double.NaN;
            for (String[] row : et.rows) {
                double t = et.num(row, "t_s");
                if (Double.isFinite(t)) {
                    if (Double.isNaN(first)) first = t;
                    last = t;
                }
                boolean read = "R".equals(et.str(row, "kind"));
                boolean flagged = "1".equals(et.str(row, "flag")); // cache hit (read) or repeated value (write): no hub traffic
                if (flagged) continue;
                int k = classOf(types.get(et.str(row, "device")), read);
                double us = et.num(row, "latency_us");
                if (k >= 0 && Double.isFinite(us)) lat.get(k).add(us / 1000);
            }
            double meanLoop = sys.has("loop_avg_ms") ? Stats.mean(Arrays.stream(sys.column("loop_avg_ms")).filter(Double::isFinite).toArray()) : Stats.mean(lm);
            if (Double.isFinite(first) && last > first && meanLoop > 0) loopsWithEvents += (last - first) * 1000 / meanLoop;
        }
        if (logic.isEmpty() && loopsWithEvents == 0) return null;
        LoopParts p = new LoopParts();
        if (!logic.isEmpty()) p.logicMs = Stats.mean(logic.stream().mapToDouble(Double::doubleValue).toArray());
        double cvSum = 0, cvW = 0;
        for (int k = 0; k < CLASSES.length; k++) {
            double[] a = lat.get(k).stream().mapToDouble(Double::doubleValue).toArray();
            if (a.length == 0 || loopsWithEvents <= 0) continue;
            p.has[k] = true;
            p.ms[k] = Stats.mean(a);
            p.perLoop[k] = a.length / loopsWithEvents;
            if (a.length > 1 && p.ms[k] > 0) {
                cvSum += Stats.std(a) / p.ms[k] * a.length;
                cvW += a.length;
            }
        }
        if (cvW > 0) p.cv = cvSum / cvW;
        if (loopSamples > 0) p.spikeProb = (double) spikeCount / loopSamples;
        if (jitter.size() > 2) p.jitterStd = Stats.std(jitter.stream().mapToDouble(Double::doubleValue).toArray());
        if (spikes.size() > 2) {
            double[] a = spikes.stream().mapToDouble(Double::doubleValue).toArray();
            p.spikeMean = Stats.mean(a);
            p.spikeStd = Stats.std(a);
        }
        StringBuilder sb = new StringBuilder("  loop parts: logic " + fmt(p.logicMs) + " ms");
        for (int k = 0; k < CLASSES.length; k++) {
            if (p.has[k]) sb.append(", ").append(CLASSES[k].replace('_', ' ')).append(' ').append(fmt(p.perLoop[k])).append("/loop x ").append(fmt(p.ms[k])).append(" ms");
        }
        log.println(sb);
        return p;
    }

    private static String fmt(double v) {
        return Csv.fmt(round(v, 3));
    }

    static Session load(Path dir) throws IOException {
        Csv.Table t = Csv.read(dir.resolve("system.csv"));
        String opMode = null;
        Path summary = dir.resolve("summary.json");
        if (Files.isRegularFile(summary)) {
            try {
                Object j = Json.parse(new String(Files.readAllBytes(summary), StandardCharsets.UTF_8));
                opMode = Json.str(Json.obj(j, "session"), "opMode");
            } catch (RuntimeException e) {
                // a session cut off by a power loss can have a partial summary; the CSVs are still fine
            }
        }
        return new Session(dir.getFileName().toString(), opMode, dir, t.column("t_s"), t.column("battery_v"), t.column("total_current_a"),
                t.has("loop_ms") ? Arrays.stream(t.column("loop_ms")).filter(Double::isFinite).toArray() : new double[0]);
    }

    static double driveTopSpeed(ProfileDb db, List<Session> sessions, PrintStream log) throws IOException, SQLException {
        Double ticks = db.value("drive.encoder_ticks_per_rev"), wheel = db.value("drive.wheel_diameter_m"), gear = db.value("drive.gear_ratio");
        if (ticks == null || wheel == null) {
            log.println("  drive top speed: skipped (set drive.encoder_ticks_per_rev and drive.wheel_diameter_m)");
            return Double.NaN;
        }
        ProfileDb.ParamRow namesRow = db.param("drive.motor_names");
        List<String> names = namesRow != null && namesRow.text() != null
                ? Arrays.stream(namesRow.text().split(",")).map(String::trim).filter(s -> !s.isEmpty()).collect(Collectors.toList()) : null;
        List<Double> speeds = new ArrayList<>();
        for (Session s : sessions) {
            Path m = s.dir().resolve("motors.csv");
            if (!Files.isRegularFile(m)) continue;
            Csv.Table t = Csv.read(m);
            for (String[] row : t.rows) {
                String name = t.str(row, "motor");
                boolean drive = names != null ? names.contains(name) : DRIVE_NAME.matcher(name).find();
                double v = t.num(row, "velocity_tps");
                if (drive && Double.isFinite(v)) speeds.add(Math.abs(v));
            }
        }
        if (speeds.size() < 20) {
            log.println("  drive top speed: no drive motor velocity samples found" + (names == null ? " (set drive.motor_names)" : ""));
            return Double.NaN;
        }
        double tps = Stats.percentile(speeds.stream().mapToDouble(Double::doubleValue).toArray(), 99);
        double g = gear == null ? 1 : gear;
        double mps = tps / ticks / g * Math.PI * wheel;
        log.printf(Locale.ROOT, "  drive top speed seen: %.2f m/s (99th percentile of %d wheel samples)%n", mps, speeds.size());
        return mps;
    }

    /** Groups {x, y} points into bins of width {@code w}: {binCenter, meanY, stdY, n} (bins with >= minN points). */
    static List<double[]> bin(List<double[]> pts, double w, int minN) {
        java.util.TreeMap<Long, Stats.Running> bins = new java.util.TreeMap<>();
        for (double[] p : pts) bins.computeIfAbsent((long) Math.floor(p[0] / w), k -> new Stats.Running()).add(p[1]);
        List<double[]> out = new ArrayList<>();
        for (java.util.Map.Entry<Long, Stats.Running> e : bins.entrySet()) {
            Stats.Running r = e.getValue();
            if (r.n() < minN) continue;
            out.add(new double[]{(e.getKey() + 0.5) * w, r.mean(), r.std(), r.n()});
        }
        return out;
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

    static double[] tail(double[] v, int n) {
        double[] f = Arrays.stream(v).filter(Double::isFinite).toArray();
        return Arrays.copyOfRange(f, Math.max(0, f.length - n), f.length);
    }

    static double[] concat(List<double[]> parts) {
        return parts.stream().flatMapToDouble(Arrays::stream).filter(Double::isFinite).toArray();
    }

    static double[] subsample(double[] v, int max) {
        if (v.length <= max) return v.clone();
        double[] out = new double[max];
        for (int i = 0; i < max; i++) out[i] = v[(int) ((long) i * v.length / max)];
        return out;
    }

    static double round(double v, int sig) {
        if (!Double.isFinite(v) || v == 0) return v;
        double mag = Math.pow(10, sig - 1 - Math.floor(Math.log10(Math.abs(v))));
        return Math.round(v * mag) / mag;
    }

    static SvgChart.Series series(String name, List<double[]> pts) {
        return new SvgChart.Series(name, pts.stream().mapToDouble(p -> p[0]).toArray(), pts.stream().mapToDouble(p -> p[1]).toArray());
    }
}
