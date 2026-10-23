package org.athletech.sim;

import org.athletech.sim.config.Catalog;
import org.athletech.sim.config.ProfileDb;
import org.athletech.sim.config.ProfileExport;
import org.athletech.sim.config.ProfileReport;
import org.athletech.sim.config.RobotParams;
import org.athletech.sim.config.SourceKind;
import org.athletech.sim.field.Aperture;
import org.athletech.sim.input.MonitorImport;
import org.athletech.sim.input.ObservationImport;
import org.athletech.sim.physics.Battery;
import org.athletech.sim.predict.AimSolver;
import org.athletech.sim.predict.DriveTimePredictor;
import org.athletech.sim.predict.ShotPredictor;
import org.athletech.sim.robot.Launcher;
import org.athletech.sim.robot.MecanumDrive;
import org.athletech.sim.scenarios.DriveScenario;
import org.athletech.sim.scenarios.LauncherScenario;
import org.athletech.sim.scenarios.LocalizationScenario;
import org.athletech.sim.scenarios.SimContext;
import org.athletech.sim.utility.Args;
import org.athletech.sim.utility.Csv;
import org.athletech.sim.utility.Rng;

import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Locale;

/**
 * Command-line entry point: {@code ./gradlew :Sim:run --args="<command>"}. Run with "help" for the
 * list of commands.
 */
public final class Main {
    private Main() {
    }

    static final String HELP = "Robot sims + robot profile database.   ./gradlew :Sim:run --args=\"<command> [options]\"\n" +
            "\n" +
            "PROFILE (the database a game simulator reads: data/robot_profile.db)\n" +
            "  profile init [--example]          create/update the database (--example: fill empty inputs with\n" +
            "                                    placeholder numbers so the sims can run; use a scratch --db)\n" +
            "  profile report [--html]           what is filled, what is missing (--html: out/profile_report.html)\n" +
            "  profile list [prefix]             every parameter with value, unit and source\n" +
            "  profile get <name>                one entry (parameter, curve, surface, distribution or action)\n" +
            "  profile set <key> <value>         set a parameter   [--std S] [--source measured|manual|game_manual]\n" +
            "                                    [--unit U --description D] adds a parameter not in the catalog\n" +
            "  profile set-dist <name> <kind> <mean> <std>     set an error distribution (normal, uniform, ...)\n" +
            "  profile set-action <name> <mean_s> <std_s> <success 0-1>\n" +
            "  profile clear <name>              empty a parameter, curve, surface, distribution or action again\n" +
            "  profile set-meta <key> <value>    e.g. robot_name, team_number\n" +
            "  profile import-monitor <folder>   HardwareMonitor sessions (adb pull /sdcard/FIRST/data/HardwareMonitor)\n" +
            "  profile import-shots <file.csv>   real shot results  (template: data/templates/shots_template.csv)\n" +
            "  profile import-poses <file.csv>   real localization checks (template: data/templates/poses_template.csv)\n" +
            "  profile export [--format json|csv] [--file path]   for programs that do not read SQLite\n" +
            "  profile docs                      regenerate PROFILE.md (the data dictionary)\n" +
            "\n" +
            "SIMS (read the profile; --save writes results back into it)\n" +
            "  sim drive          drivetrain: top speed, acceleration, current, move times, battery effect\n" +
            "  sim launcher       flywheel spin-up/recovery, shooting table, Monte Carlo make probability\n" +
            "  sim localization   odometry drift over a match\n" +
            "  sim all            all of the above, in order\n" +
            "\n" +
            "PREDICTIONS (quick answers from the current profile)\n" +
            "  predict shot --distance D [--ball pollen|nectar] [--battery V] [--bearing DEG]\n" +
            "  predict drive --distance D [--strafe] [--battery V]\n" +
            "  predict turn --angle DEG [--battery V]\n" +
            "\n" +
            "OPTIONS\n" +
            "  --db PATH       profile database (default data/robot_profile.db)\n" +
            "  --out DIR       reports and CSV output (default out)\n" +
            "  --save          store sim results in the profile\n" +
            "  --force         let a write replace a higher-priority value (e.g. overwrite a measurement)\n" +
            "  --trials N      Monte Carlo trials per point (default 1000)\n" +
            "  --seed N        random seed (default 1)\n";

    public static void main(String[] argv) {
        PrintStream out;
        try {
            out = new PrintStream(System.out, true, "UTF-8");
        } catch (java.io.UnsupportedEncodingException e) {
            out = System.out; // UTF-8 is always supported
        }
        try {
            int code = run(argv, out);
            System.exit(code);
        } catch (RobotParams.MissingInputsException e) {
            out.println(e.getMessage());
            System.exit(2);
        } catch (Exception e) {
            out.println("Error: " + e.getMessage());
            if (System.getenv("SIM_DEBUG") != null) e.printStackTrace(out);
            System.exit(1);
        }
    }

    public static int run(String[] argv, PrintStream out) throws Exception {
        Args a = new Args(argv);
        String cmd = a.pos(0);
        if (cmd == null || cmd.equals("help") || a.flag("help")) {
            out.print(HELP);
            return 0;
        }
        Path dbPath = Paths.get(a.str("db", "data/robot_profile.db"));
        Path outDir = Paths.get(a.str("out", "out"));
        String sub = a.pos(1);
        boolean force = a.flag("force");
        int code;
        switch (cmd) {
            case "profile":
                code = profile(a, sub, dbPath, outDir, force, out);
                break;
            case "sim":
                code = sim(a, sub, dbPath, outDir, force, out, String.join(" ", argv));
                break;
            case "predict":
                code = predict(a, sub, dbPath, out);
                break;
            default: {
                out.println("Unknown command '" + cmd + "'.\n");
                out.print(HELP);
                code = 1;
                break;
            }
        }
        List<String> unused = a.unused();
        if (!unused.isEmpty()) out.println("Note: ignored option(s) " + String.join(", ", unused));
        return code;
    }

    // ------------------------------------------------------------------ profile

    static int profile(Args a, String sub, Path dbPath, Path outDir, boolean force, PrintStream out) throws Exception {
        if (sub == null) {
            out.print(HELP);
            return 1;
        }
        try (ProfileDb db = ProfileDb.open(dbPath)) {
            switch (sub) {
                case "init": {
                    out.println("Profile ready: " + dbPath.toAbsolutePath());
                    out.println("  " + (Catalog.PARAMS.size() + Catalog.CURVES.size() + Catalog.SURFACES.size() + Catalog.DISTS.size() + Catalog.ACTIONS.size())
                            + " catalog entries (" + Catalog.PARAMS.size() + " parameters, " + Catalog.CURVES.size() + " curves, " + Catalog.SURFACES.size()
                            + " surfaces, " + Catalog.DISTS.size() + " distributions, " + Catalog.ACTIONS.size() + " actions); game-manual values pre-filled.");
                    if (a.flag("example")) {
                        int n = db.fillExamples();
                        out.println("  filled " + n + " empty entries with PLACEHOLDER example values (not your robot).");
                        if (dbPath.normalize().endsWith(Paths.get("data/robot_profile.db"))) {
                            out.println("  Warning: that is the shared profile. Clear placeholders with 'profile clear <key>' as you add real data.");
                        }
                    }
                    return 0;
                }
                case "report": {
                    ProfileReport.text(db, out);
                    if (a.flag("html")) {
                        Path f = outDir.resolve("profile_report.html");
                        ProfileReport.html(db, f);
                        out.println("\n  HTML report: " + f.toAbsolutePath());
                    }
                    return 0;
                }
                case "list": {
                    String prefix = a.pos(2) == null ? "" : a.pos(2);
                    for (ProfileDb.ParamRow r : db.params(prefix)) {
                        String v = r.text() != null ? r.text() : r.value() == null ? "-" : Csv.fmt(r.value());
                        out.printf(Locale.ROOT, "  %-44s %14s %-10s %-12s%s%n", r.key(), v, r.unit(),
                                r.sourceKind() == null ? "" : r.sourceKind().id(), r.required() && !r.hasData() ? "  (required)" : "");
                    }
                    return 0;
                }
                case "get": {
                    return get(db, a.pos(2), out);
                }
                case "set": {
                    String key = need(a.pos(2), "profile set <key> <value>"), val = need(a.pos(3), "profile set <key> <value>");
                    SourceKind kind = SourceKind.parse(a.str("source", "manual"));
                    if (db.param(key) == null) {
                        if (!a.has("unit")) {
                            out.println("'" + key + "' is not in the catalog. To add a team-specific parameter, also pass --unit and --description.");
                            return 1;
                        }
                        db.addCustomParam(key, a.str("unit", ""), a.str("description", ""));
                    }
                    Double num = parseNum(val);
                    ProfileDb.ParamRow row = db.param(key);
                    if (num != null && row.outOfRange(num)) {
                        out.println("Warning: " + val + " is outside the usual range [" + Csv.fmt(row.min() == null ? Double.NaN : row.min()) + ", "
                                + Csv.fmt(row.max() == null ? Double.NaN : row.max()) + "] " + row.unit() + " for " + key + ". Check the unit.");
                    }
                    long src = db.addSource(kind, "profile set", key + " = " + val);
                    Double std = a.has("std") ? a.num("std", Double.NaN) : null;
                    ProfileDb.Write w = db.setParam(key, num, num == null ? val : null, std, src, force);
                    out.println(w == ProfileDb.Write.WRITTEN ? key + " = " + val + " " + row.unit() + " (" + kind.id() + ")"
                            : key + " kept: it already has a higher-priority value (" + row.sourceKind().id() + "). Use --force to replace it.");
                    return w == ProfileDb.Write.WRITTEN ? 0 : 1;
                }
                case "set-dist": {
                    String usage = "profile set-dist <name> <kind> <mean> <std>";
                    String name = need(a.pos(2), usage), kind = need(a.pos(3), usage);
                    Double mean = parseNum(need(a.pos(4), usage)), std = parseNum(need(a.pos(5), usage));
                    long src = db.addSource(SourceKind.parse(a.str("source", "manual")), "profile set-dist", String.join(" ", a.positional()));
                    ProfileDb.Write w = db.setDist(name, kind, mean, std, a.has("min") ? a.num("min", 0) : null, a.has("max") ? a.num("max", 0) : null,
                            a.has("mode") ? a.num("mode", 0) : null, null, src, force);
                    out.println(name + ": " + w);
                    return w == ProfileDb.Write.WRITTEN ? 0 : 1;
                }
                case "set-action": {
                    String usage = "profile set-action <name> <mean_s> <std_s> <success 0-1>";
                    String name = need(a.pos(2), usage);
                    long src = db.addSource(SourceKind.parse(a.str("source", "manual")), "profile set-action", String.join(" ", a.positional()));
                    ProfileDb.Write w = db.setAction(name, parseNum(need(a.pos(3), usage)), parseNum(need(a.pos(4), usage)),
                            a.has("min") ? a.num("min", 0) : null, parseNum(need(a.pos(5), usage)), src, force);
                    out.println(name + ": " + w);
                    return w == ProfileDb.Write.WRITTEN ? 0 : 1;
                }
                case "clear": {
                    String key = need(a.pos(2), "profile clear <name>");
                    String kind = db.clear(key);
                    if (kind == null) {
                        out.println("No parameter, curve, surface, distribution or action named " + key);
                        return 1;
                    }
                    out.println(kind + " " + key + " cleared");
                    return 0;
                }
                case "set-meta": {
                    db.putMeta(need(a.pos(2), "profile set-meta <key> <value>"), need(a.pos(3), "profile set-meta <key> <value>"));
                    return 0;
                }
                case "import-monitor": {
                    MonitorImport.run(db, Paths.get(need(a.pos(2), "profile import-monitor <HardwareMonitor folder>")), out, force, outDir);
                    return 0;
                }
                case "import-shots": {
                    ObservationImport.importShots(db, Paths.get(need(a.pos(2), "profile import-shots <file.csv>")), out, force);
                    return 0;
                }
                case "import-poses": {
                    ObservationImport.importPoses(db, Paths.get(need(a.pos(2), "profile import-poses <file.csv>")), out, force);
                    return 0;
                }
                case "export": {
                    String fmt = a.str("format", "json");
                    if (fmt.equals("json")) {
                        Path f = Paths.get(a.str("file", outDir.resolve("robot_profile.json").toString()));
                        ProfileExport.json(db, f);
                        out.println("Wrote " + f.toAbsolutePath());
                    } else if (fmt.equals("csv")) {
                        Path d = Paths.get(a.str("file", outDir.resolve("robot_profile_csv").toString()));
                        ProfileExport.csv(db, d);
                        out.println("Wrote CSV tables to " + d.toAbsolutePath());
                    } else {
                        out.println("--format must be json or csv");
                        return 1;
                    }
                    return 0;
                }
                case "docs": {
                    Path f = Paths.get(a.str("file", "PROFILE.md"));
                    Files.write(f, ProfileExport.docs().getBytes(StandardCharsets.UTF_8));
                    out.println("Wrote " + f.toAbsolutePath());
                    return 0;
                }
                default: {
                    out.println("Unknown profile command '" + sub + "'.");
                    return 1;
                }
            }
        }
    }

    static int get(ProfileDb db, String name, PrintStream out) throws Exception {
        if (name == null) {
            out.println("profile get <name>");
            return 1;
        }
        ProfileDb.ParamRow p = db.param(name);
        if (p != null) {
            out.println(p.key() + " = " + (p.text() != null ? p.text() : p.value() == null ? "(empty)" : Csv.fmt(p.value())) + " " + p.unit()
                    + (p.std() == null ? "" : "  +/- " + Csv.fmt(p.std())));
            out.println("  " + p.description());
            out.println("  role " + p.role() + (p.required() ? ", required" : "") + (p.sourceKind() == null ? "" : ", source " + p.sourceKind().id() + " (" + p.sourceName() + ", " + p.updated() + ")"));
            return 0;
        }
        ProfileDb.CurveRow c = db.curve(name);
        if (c != null) {
            out.println(c.name() + ": " + c.yName() + " [" + c.yUnit() + "] vs " + c.xName() + " [" + c.xUnit() + "]" + (c.sourceKind() == null ? "" : ", source " + c.sourceKind().id()));
            out.println("  " + c.description());
            for (double[] q : c.points()) out.println("  " + Csv.fmt(q[0]) + "\t" + Csv.fmt(q[1]) + (Double.isNaN(q[2]) ? "" : "\t+/- " + Csv.fmt(q[2])));
            if (c.points().isEmpty()) out.println("  (no points yet)");
            return 0;
        }
        ProfileDb.SurfaceRow s = db.surface(name);
        if (s != null) {
            out.println(s.name() + ": " + s.zName() + " [" + s.zUnit() + "] by " + s.xName() + " [" + s.xUnit() + "] and " + s.yName() + " [" + s.yUnit() + "]");
            for (double[] q : s.points()) out.println("  " + Csv.fmt(q[0]) + "\t" + Csv.fmt(q[1]) + "\t" + Csv.fmt(q[2]));
            if (s.points().isEmpty()) out.println("  (no points yet)");
            return 0;
        }
        ProfileDb.DistRow d = db.dist(name);
        if (d != null) {
            out.println(d.name() + ": " + d.toDistribution().describe() + " per " + d.per());
            out.println("  " + d.description());
            return 0;
        }
        ProfileDb.ActionRow ac = db.action(name);
        if (ac != null) {
            out.println(ac.name() + ": " + (ac.meanTimeS() == null ? "(empty)" : Csv.fmt(ac.meanTimeS()) + " s +/- " + Csv.fmt(ac.stdTimeS() == null ? Double.NaN : ac.stdTimeS())
                    + ", success " + Csv.fmt(ac.successProb() == null ? Double.NaN : ac.successProb())));
            out.println("  " + ac.description());
            return 0;
        }
        out.println("Nothing named " + name + " in the profile.");
        return 1;
    }

    // ------------------------------------------------------------------ sims

    static int sim(Args a, String sub, Path dbPath, Path outDir, boolean force, PrintStream out, String command) throws Exception {
        if (sub == null) {
            out.print(HELP);
            return 1;
        }
        try (ProfileDb db = ProfileDb.open(dbPath)) {
            SimContext ctx = new SimContext(db, outDir, a.flag("save"), force, (long) a.num("seed", 1), a.integer("trials", 1000), out, command);
            long t0 = System.nanoTime();
            switch (sub) {
                case "drive":
                    DriveScenario.run(ctx);
                    break;
                case "launcher":
                    LauncherScenario.run(ctx);
                    break;
                case "localization":
                    LocalizationScenario.run(ctx);
                    break;
                case "all": {
                    DriveScenario.run(ctx);
                    out.println();
                    LocalizationScenario.run(ctx); // before the launcher: it produces the pose error the shots use
                    out.println();
                    LauncherScenario.run(ctx);
                    break;
                }
                default: {
                    out.println("Unknown sim '" + sub + "' (drive, launcher, localization, all).");
                    return 1;
                }
            }
            out.printf(Locale.ROOT, "Done in %.1f s.%n", (System.nanoTime() - t0) / 1e9);
            return 0;
        }
    }

    // ------------------------------------------------------------------ predictions

    static int predict(Args a, String sub, Path dbPath, PrintStream out) throws Exception {
        if (sub == null) {
            out.print(HELP);
            return 1;
        }
        try (ProfileDb db = ProfileDb.open(dbPath)) {
            RobotParams rp = new RobotParams(db);
            double volts = a.num("battery", Double.NaN);
            switch (sub) {
                case "shot": {
                    String ball = a.str("ball", "pollen");
                    double dist = a.num("distance", Double.NaN), bearing = Math.toRadians(a.num("bearing", 0));
                    if (Double.isNaN(dist)) throw new IllegalArgumentException("predict shot needs --distance (meters to the CELL center)");
                    Launcher.Params lp = rp.launcher();
                    Aperture cell = rp.cell();
                    Battery probe = Double.isNaN(volts) ? null : rp.battery(volts);
                    double held = Double.isNaN(volts) ? 1 : rp.req("robot.max_" + ball + "_held");
                    ShotPredictor.Errors e = new ShotPredictor.Errors(rp.dist("launcher.rpm_error"), rp.dist("launcher.hood_error_deg"),
                            rp.dist("launcher.exit_speed_variation"), rp.dist("launcher.exit_angle_variation_deg"), rp.dist("turret.aim_error_deg"),
                            rp.dist("localization.pose_error_m"), rp.dist("localization.heading_error_deg"));
                    rp.requireAll();
                    Launcher L = new Launcher(lp);
                    AimSolver.Solution s = AimSolver.solve(L, ball, cell, dist, bearing, e.speedSigma(L, ball), e.angleSigmaDeg());
                    if (!s.reachable()) {
                        out.printf(Locale.ROOT, "%s from %.2f m: out of reach (max exit speed %.1f m/s)%n", ball.toUpperCase(), dist, L.maxExitSpeed(ball));
                        return 0;
                    }
                    ShotPredictor.Prediction p = ShotPredictor.predict(L, ball, cell, s, bearing, Double.NaN, e, a.integer("trials", 2000), new Rng((long) a.num("seed", 1)));
                    out.printf(Locale.ROOT, "%s shot from %.2f m (bearing %.0f deg):%n", ball.toUpperCase(), dist, Math.toDegrees(bearing));
                    out.printf(Locale.ROOT, "  hood %.1f deg, flywheel %.0f rpm, exit speed %.2f m/s%n", s.hoodDeg(), s.rpm(), s.exitSpeed());
                    out.printf(Locale.ROOT, "  scores with exit speed %.2f-%.2f m/s (+/-%.1f%%) and launch angle %.1f-%.1f deg%n",
                            s.speedLow(), s.speedHigh(), 50 * s.tolerance(), s.hoodDeg() - s.angleLowDeg(), s.hoodDeg() + s.angleHighDeg());
                    out.printf(Locale.ROOT, "  make probability %.1f%% (95%% interval %.1f-%.1f%%, %d trials); misses: %s%n",
                            100 * p.probability(), 100 * p.low95(), 100 * p.high95(), p.trials(), p.outcomes());
                    if (probe != null) {
                        // a burst of every ball held, fired as soon as the flywheel is ready, on that battery
                        double rInt = rp.req("battery.internal_resistance_ohm"), drop = rp.opt("battery.voltage_drop_per_ah", 0.6);
                        int count = Math.max(1, (int) Math.round(held));
                        double[] b = LauncherScenario.burstMake(L, lp, v -> new Battery(v, rInt, drop), volts, rp.electronicsCurrent(), ball, cell, s, e,
                                count, Math.max(200, a.integer("trials", 2000) / 2), new Rng((long) a.num("seed", 1) + 1), dist);
                        out.printf(Locale.ROOT, "  burst of %d on a %.2f V battery: %.1f%% per ball (+/-%.1f%%)%n", count, volts, 100 * b[1], 100 * b[2]);
                    }
                    if (!rp.assumptions().isEmpty()) out.println("  assumptions: " + String.join("; ", rp.assumptions()));
                    if (!rp.placeholderInputs().isEmpty()) out.println("  uses PLACEHOLDER inputs: " + String.join(", ", rp.placeholderInputs()));
                    return 0;
                }
                case "drive":
                case "turn": {
                    MecanumDrive.Params p = rp.drive();
                    Battery probe = rp.battery(volts);
                    double other = rp.electronicsCurrent();
                    rp.requireAll();
                    double voc = probe.openCircuitVolts();
                    boolean turn = sub.equals("turn");
                    DriveTimePredictor.Axis axis = turn ? DriveTimePredictor.Axis.TURN : a.flag("strafe") ? DriveTimePredictor.Axis.STRAFE : DriveTimePredictor.Axis.FORWARD;
                    double amount = turn ? Math.toRadians(a.num("angle", Double.NaN)) : a.num("distance", Double.NaN);
                    if (Double.isNaN(amount)) throw new IllegalArgumentException(turn ? "predict turn needs --angle DEG" : "predict drive needs --distance M");
                    DriveTimePredictor.Move m = DriveTimePredictor.timeTo(p, () -> rp.battery(voc), other, axis, amount);
                    double overhead = rp.opt("drive.path_overhead_factor", 1.0);
                    out.printf(Locale.ROOT, "%s %s on a %.2f V battery: %.2f s physics limit, about %.2f s with path overhead x%.2f%n",
                            axis.name().toLowerCase(Locale.ROOT), turn ? a.num("angle", 0) + " deg" : amount + " m", voc, m.timeS(), m.timeS() * overhead, overhead);
                    out.printf(Locale.ROOT, "  peak speed %.2f %s, peak drive current %.1f A, battery dips to %.2f V%n",
                            turn ? Math.toDegrees(m.peakSpeed()) : m.peakSpeed(), turn ? "deg/s" : "m/s", m.peakCurrentA(), m.minBusV());
                    if (!rp.placeholderInputs().isEmpty()) out.println("  uses PLACEHOLDER inputs: " + String.join(", ", rp.placeholderInputs()));
                    return 0;
                }
                default: {
                    out.println("Unknown prediction '" + sub + "' (shot, drive, turn).");
                    return 1;
                }
            }
        }
    }

    // ------------------------------------------------------------------ helpers

    private static String need(String v, String usage) {
        if (v == null) throw new IllegalArgumentException("usage: " + usage);
        return v;
    }

    private static Double parseNum(String s) {
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            return null;
        }
    }
}
