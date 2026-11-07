package org.athletech.sim.config;

import org.athletech.sim.utility.Csv;
import org.athletech.sim.visualization.HtmlReport;
import org.athletech.sim.visualization.SvgChart;

import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Path;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

public final class ProfileReport {
    private ProfileReport() {
    }

    static final class CategoryCount {
        private final int entries;
        private final int filled;
        private final int required;
        private final int requiredFilled;
        private final int placeholders;

        CategoryCount(int entries, int filled, int required, int requiredFilled, int placeholders) {

            this.entries = entries;
            this.filled = filled;
            this.required = required;
            this.requiredFilled = requiredFilled;
            this.placeholders = placeholders;
        }

        public int entries() { return entries; }
        public int filled() { return filled; }
        public int required() { return required; }
        public int requiredFilled() { return requiredFilled; }
        public int placeholders() { return placeholders; }

        @Override
        public boolean equals(Object other) {

            if (this == other) return true;
            if (!(other instanceof CategoryCount)) return false;
            CategoryCount o = (CategoryCount) other;
            return entries == o.entries && filled == o.filled && required == o.required && requiredFilled == o.requiredFilled && placeholders == o.placeholders;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(entries, filled, required, requiredFilled, placeholders);
        }

        @Override
        public String toString() {
            return "CategoryCount[entries=" + entries + ", filled=" + filled + ", required=" + required + ", requiredFilled=" + requiredFilled + ", placeholders=" + placeholders + "]";
        }
    }

    static Map<String, CategoryCount> counts(List<ProfileDb.Status> all, Set<String> disabled) {

        Map<String, int[]> m = new LinkedHashMap<>();
        for (String[] c : Catalog.CATEGORIES) m.put(c[0], new int[5]);
        for (ProfileDb.Status s : all) {
            int[] c = m.computeIfAbsent(s.category(), k -> new int[5]);
            c[0]++;
            if (s.hasData()) c[1]++;
            if (required(s.name(), s.required(), disabled)) c[2]++;
            if (required(s.name(), s.required(), disabled) && s.hasData() && s.sourceKind() != SourceKind.PLACEHOLDER) c[3]++;
            if (s.hasData() && s.sourceKind() == SourceKind.PLACEHOLDER) c[4]++;
        }
        Map<String, CategoryCount> out = new LinkedHashMap<>();
        m.forEach((k, c) -> {
            if (c[0] > 0) out.put(k, new CategoryCount(c[0], c[1], c[2], c[3], c[4]));
        });
        return out;
    }

    private static Set<String> disabledLaunchers(ProfileDb db) throws SQLException {

        Set<String> disabled = new java.util.LinkedHashSet<>();
        for (String ball : Catalog.BALLS) {
            Double enabled = db.value("launcher." + ball + ".enabled");
            if (enabled != null && enabled == 0) disabled.add(ball);
        }
        return disabled;
    }

    private static boolean required(String name, boolean required, Set<String> disabled) {

        if (!required) return false;
        for (String ball : disabled) {
            String prefix = "launcher." + ball + ".";
            if (name.startsWith(prefix) && !name.equals(prefix + "enabled")) return false;
        }
        return true;
    }

    public static void text(ProfileDb db, PrintStream out) throws SQLException {

        List<ProfileDb.Status> all = db.everything();
        Set<String> disabled = disabledLaunchers(db);
        Map<String, CategoryCount> counts = counts(all, disabled);
        int req = 0, reqOk = 0, ph = 0, filled = 0;
        out.println("Robot profile: " + db.path().toAbsolutePath());
        String name = db.meta("robot_name");
        out.println("  season " + db.meta("season") + (name == null || name.isEmpty() ? "" : ", robot " + name) + ", schema " + db.meta("schema_version") + ", " + (db.meta("updated_utc") == null ? "created " + db.meta("created_utc") + ", no robot data yet" : "updated " + db.meta("updated_utc")));
        out.printf("%n  %-14s %8s %9s %13s%n", "category", "entries", "filled", "required ok");
        for (Map.Entry<String, CategoryCount> e : counts.entrySet()) {
            CategoryCount c = e.getValue();
            out.printf("  %-14s %8d %9d %8d / %-3d%s%n", e.getKey(), c.entries(), c.filled(), c.requiredFilled(), c.required(), c.placeholders() > 0 ? "  (" + c.placeholders() + " placeholder)" : "");
            req += c.required();
            reqOk += c.requiredFilled();
            ph += c.placeholders();
            filled += c.filled();
        }
        out.printf("%n  %d of %d entries have data; %d of %d required entries have real (non-placeholder) data%s.%n", filled, all.size(), reqOk, req, ph > 0 ? "; " + ph + " placeholders" : "");
        out.println("  Observations: " + db.count("shot_observations") + " shots, " + db.count("pose_observations") + " pose checks, " + db.count("disconnect_observations") + " disconnect-log rows.");
        if (!disabled.isEmpty()) out.println("  Unsupported launcher ball types: " + String.join(", ", disabled) + " (shot entries not required).");
        List<ProfileDb.Status> missingInputs = all.stream().filter(s -> required(s.name(), s.required(), disabled) && !s.hasData() && !"derived".equals(s.role())).collect(Collectors.toList());
        List<ProfileDb.Status> missingDerived = all.stream().filter(s -> required(s.name(), s.required(), disabled) && !s.hasData() && "derived".equals(s.role())).collect(Collectors.toList());
        if (!missingInputs.isEmpty()) {
            out.println("\n  Still needed from you (inputs and measurements):");
            for (ProfileDb.Status s : missingInputs) out.println("    " + s.type() + "  " + s.name());
        }
        if (!missingDerived.isEmpty()) {
            out.println("\n  Filled by the sims once the inputs exist (sim all --save): " + missingDerived.size() + " entries.");
        }
    }

    public static void html(ProfileDb db, Path file) throws SQLException, IOException {

        List<ProfileDb.Status> all = db.everything();
        Set<String> disabled = disabledLaunchers(db);
        Map<String, CategoryCount> counts = counts(all, disabled);
        HtmlReport r = new HtmlReport("Robot profile");
        r.paragraph(db.path().toAbsolutePath() + " | season " + db.meta("season") + " | " + (db.meta("updated_utc") == null ? "created " + db.meta("created_utc") : "updated " + db.meta("updated_utc")));
        List<List<String>> rows = new ArrayList<>();
        counts.forEach((k, c) -> rows.add(Arrays.asList(k, Integer.toString(c.entries()), Integer.toString(c.filled()), c.requiredFilled() + " / " + c.required(), Integer.toString(c.placeholders()), Catalog.categoryDescription(k))));
        r.heading("Completeness");
        r.table(Arrays.asList("category", "entries", "filled", "required with real data", "placeholders", "what it is"), rows);
        if (!disabled.isEmpty()) r.note("info", "Unsupported launcher ball types: " + String.join(", ", disabled) + ". Their shot entries are not required; historical data remains visible.");
        List<ProfileDb.Status> missing = all.stream().filter(s -> required(s.name(), s.required(), disabled) && !s.hasData()).collect(Collectors.toList());
        if (!missing.isEmpty()) {
            r.note("warn", missing.size() + " required entries are still empty: " + String.join(", ", missing.stream().map(ProfileDb.Status::name).collect(Collectors.toList())));
        }
        r.heading("Parameters");
        List<List<String>> prow = new ArrayList<>();
        for (ProfileDb.ParamRow p : db.params("")) {
            prow.add(Arrays.asList(p.key(), p.text() != null ? p.text() : p.value() == null ? "" : Csv.fmt(p.value()), p.unit(), p.std() == null ? "" : Csv.fmt(p.std()), p.sourceKind() == null ? "" : p.sourceKind().id(), p.role(), required(p.key(), p.required(), disabled) ? "yes" : ""));
        }
        r.table(Arrays.asList("key", "value", "unit", "std", "source", "role", "required"), prow);
        r.heading("Curves");
        for (ProfileDb.CurveRow c : db.curves("")) {
            if (!c.hasData()) continue;
            SvgChart ch = new SvgChart(c.name() + (c.sourceKind() == null ? "" : "  [" + c.sourceKind().id() + "]"), c.xName() + " (" + c.xUnit() + ")", c.yUnit());
            double[] band = c.points().stream().mapToDouble(p -> p[2]).toArray();
            boolean hasBand = java.util.Arrays.stream(band).anyMatch(Double::isFinite);
            ch.add(new SvgChart.Series(c.yName(), c.xs(), c.ys(), hasBand ? band : null));
            r.chart(ch);
        }
        r.heading("Surfaces");
        for (ProfileDb.SurfaceRow s : db.surfaces("")) {
            if (!s.hasData()) continue;
            SvgChart ch = new SvgChart(s.name() + (s.sourceKind() == null ? "" : "  [" + s.sourceKind().id() + "]"), s.xName() + " (" + s.xUnit() + ")", s.zUnit());
            Map<Double, List<double[]>> byY = new LinkedHashMap<>();
            for (double[] p : s.points()) byY.computeIfAbsent(p[1], k -> new ArrayList<>()).add(p);
            byY.forEach((y, pts) -> ch.add(new SvgChart.Series(s.yName() + " " + Csv.fmt(y) + " " + s.yUnit(), pts.stream().mapToDouble(p -> p[0]).toArray(), pts.stream().mapToDouble(p -> p[2]).toArray())));
            r.chart(ch);
        }
        r.heading("Distributions");
        List<List<String>> drow = new ArrayList<>();
        for (ProfileDb.DistRow d : db.dists("")) {
            drow.add(Arrays.asList(d.name(), d.kind() == null ? "" : d.kind(), d.mean() == null ? "" : Csv.fmt(d.mean()), d.std() == null ? "" : Csv.fmt(d.std()), d.unit(), d.per() == null ? "" : d.per(), d.samples().length == 0 ? "" : Integer.toString(d.samples().length), d.sourceKind() == null ? "" : d.sourceKind().id()));
        }
        r.table(Arrays.asList("name", "kind", "mean", "std", "unit", "per", "samples", "source"), drow);
        r.heading("Actions");
        List<List<String>> arow = new ArrayList<>();
        for (ProfileDb.ActionRow a : db.actions("")) {
            arow.add(Arrays.asList(a.name(), a.meanTimeS() == null ? "" : Csv.fmt(a.meanTimeS()), a.stdTimeS() == null ? "" : Csv.fmt(a.stdTimeS()), a.successProb() == null ? "" : Csv.fmt(a.successProb()), a.sourceKind() == null ? "" : a.sourceKind().id(), a.description()));
        }
        r.table(Arrays.asList("action", "mean s", "std s", "success", "source", "description"), arow);
        r.write(file);
    }
}
