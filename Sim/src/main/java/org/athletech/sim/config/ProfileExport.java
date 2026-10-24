package org.athletech.sim.config;

import org.athletech.sim.utility.Csv;
import org.athletech.sim.utility.Json;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.sql.SQLException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * Exports the profile for programs that would rather not open SQLite: one JSON file (same
 * content, nested) or a folder of CSV tables. Also generates PROFILE.md, the data dictionary.
 */
public final class ProfileExport {
    private ProfileExport() {
    }

    private static String kind(SourceKind k) {
        return k == null ? null : k.id();
    }

    public static Map<String, Object> toMap(ProfileDb db) throws SQLException {
        Map<String, Object> root = new LinkedHashMap<>();
        root.put("format", ProfileSchema.FORMAT + "/" + ProfileSchema.VERSION);
        root.put("meta", db.allMeta());
        Map<String, Object> params = new LinkedHashMap<>();
        for (ProfileDb.ParamRow r : db.params("")) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("value", r.text() != null ? r.text() : r.value());
            m.put("unit", r.unit());
            if (r.std() != null) m.put("std", r.std());
            m.put("source", kind(r.sourceKind()));
            m.put("role", r.role());
            m.put("required", r.required());
            m.put("description", r.description());
            params.put(r.key(), m);
        }
        root.put("parameters", params);
        Map<String, Object> curves = new LinkedHashMap<>();
        for (ProfileDb.CurveRow r : db.curves("")) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("x", r.xName() + " [" + r.xUnit() + "]");
            m.put("y", r.yName() + " [" + r.yUnit() + "]");
            m.put("source", kind(r.sourceKind()));
            m.put("role", r.role());
            m.put("required", r.required());
            m.put("description", r.description());
            List<Object> pts = new ArrayList<>();
            for (double[] p : r.points()) pts.add(Arrays.asList(p[0], p[1], Double.isNaN(p[2]) ? Double.NaN : p[2]));
            m.put("points_x_y_ystd", pts);
            curves.put(r.name(), m);
        }
        root.put("curves", curves);
        Map<String, Object> surfaces = new LinkedHashMap<>();
        for (ProfileDb.SurfaceRow r : db.surfaces("")) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("x", r.xName() + " [" + r.xUnit() + "]");
            m.put("y", r.yName() + " [" + r.yUnit() + "]");
            m.put("z", r.zName() + " [" + r.zUnit() + "]");
            m.put("source", kind(r.sourceKind()));
            m.put("required", r.required());
            m.put("description", r.description());
            List<Object> pts = new ArrayList<>();
            for (double[] p : r.points()) pts.add(Arrays.asList(p[0], p[1], p[2], p[3]));
            m.put("points_x_y_z_zstd", pts);
            surfaces.put(r.name(), m);
        }
        root.put("surfaces", surfaces);
        Map<String, Object> dists = new LinkedHashMap<>();
        for (ProfileDb.DistRow r : db.dists("")) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("kind", r.kind());
            m.put("unit", r.unit());
            m.put("per", r.per());
            m.put("mean", r.mean());
            m.put("std", r.std());
            m.put("min", r.min());
            m.put("max", r.max());
            m.put("mode", r.mode());
            if (r.samples().length > 0) m.put("samples", r.samples());
            m.put("source", kind(r.sourceKind()));
            m.put("required", r.required());
            m.put("description", r.description());
            dists.put(r.name(), m);
        }
        root.put("distributions", dists);
        Map<String, Object> actions = new LinkedHashMap<>();
        for (ProfileDb.ActionRow r : db.actions("")) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("mean_time_s", r.meanTimeS());
            m.put("std_time_s", r.stdTimeS());
            m.put("min_time_s", r.minTimeS());
            m.put("success_prob", r.successProb());
            m.put("source", kind(r.sourceKind()));
            m.put("required", r.required());
            m.put("description", r.description());
            actions.put(r.name(), m);
        }
        root.put("actions", actions);
        Map<String, Object> obs = new LinkedHashMap<>();
        obs.put("shots", db.count("shot_observations"));
        obs.put("poses", db.count("pose_observations"));
        obs.put("disconnect_log_rows", db.count("disconnect_observations"));
        root.put("observations", obs);
        return root;
    }

    public static void json(ProfileDb db, Path file) throws SQLException, IOException {
        Files.createDirectories(file.toAbsolutePath().getParent());
        Files.write(file, (Json.write(toMap(db)) + "\n").getBytes(StandardCharsets.UTF_8));
    }

    /** One CSV per table in {@code dir}. */
    public static void csv(ProfileDb db, Path dir) throws SQLException, IOException {
        List<Object[]> rows = new ArrayList<>();
        for (ProfileDb.ParamRow r : db.params("")) {
            rows.add(new Object[]{r.key(), r.text() != null ? r.text() : r.value(), r.unit(), r.std(), kind(r.sourceKind()), r.role(), r.required() ? 1 : 0, r.description()});
        }
        Csv.write(dir.resolve("parameters.csv"), Arrays.asList("key", "value", "unit", "std", "source", "role", "required", "description"), rows);
        rows = new ArrayList<>();
        for (ProfileDb.CurveRow r : db.curves("")) {
            for (double[] p : r.points()) rows.add(new Object[]{r.name(), p[0], p[1], p[2], p[3], r.xUnit(), r.yUnit(), kind(r.sourceKind())});
        }
        Csv.write(dir.resolve("curve_points.csv"), Arrays.asList("curve", "x", "y", "y_std", "n", "x_unit", "y_unit", "source"), rows);
        rows = new ArrayList<>();
        for (ProfileDb.SurfaceRow r : db.surfaces("")) {
            for (double[] p : r.points()) rows.add(new Object[]{r.name(), p[0], p[1], p[2], p[3], r.xUnit(), r.yUnit(), r.zUnit(), kind(r.sourceKind())});
        }
        Csv.write(dir.resolve("surface_points.csv"), Arrays.asList("surface", "x", "y", "z", "z_std", "x_unit", "y_unit", "z_unit", "source"), rows);
        rows = new ArrayList<>();
        for (ProfileDb.DistRow r : db.dists("")) {
            rows.add(new Object[]{r.name(), r.kind(), r.unit(), r.per(), r.mean(), r.std(), r.min(), r.max(), r.mode(), r.samples().length, kind(r.sourceKind()), r.description()});
        }
        Csv.write(dir.resolve("distributions.csv"), Arrays.asList("name", "kind", "unit", "per", "mean", "std", "min", "max", "mode", "samples", "source", "description"), rows);
        rows = new ArrayList<>();
        for (ProfileDb.ActionRow r : db.actions("")) {
            rows.add(new Object[]{r.name(), r.meanTimeS(), r.stdTimeS(), r.minTimeS(), r.successProb(), kind(r.sourceKind()), r.description()});
        }
        Csv.write(dir.resolve("actions.csv"), Arrays.asList("name", "mean_time_s", "std_time_s", "min_time_s", "success_prob", "source", "description"), rows);
    }

    /** PROFILE.md: what every entry means, generated from the catalog so it never goes stale. */
    public static String docs() {
        StringBuilder sb = new StringBuilder();
        sb.append("# Robot profile database (`Sim/data/robot_profile.db`)\n" +
                "\n" +
                "<!-- Generated by `./gradlew :Sim:run --args=\"profile docs\"` from Catalog.java. Do not edit by hand. -->\n" +
                "\n" +
                "One SQLite file holds everything a game simulator needs to know about this robot: how fast it drives\n" +
                "and accelerates, how that changes as the battery sags under current draw, how accurate its shots are,\n" +
                "how its believed position drifts, and how long each game action takes. Any language can read it\n" +
                "(Python `sqlite3`, C# `Microsoft.Data.Sqlite`, JavaScript `better-sqlite3`/`sql.js`, Unity via a SQLite\n" +
                "plugin, or the JSON/CSV export).\n" +
                "\n" +
                "Every value records **where it came from** (`source_kind`). Higher priority wins, so a sim never\n" +
                "overwrites a measurement:\n" +
                "\n" +
                "| source_kind | meaning | priority |\n" +
                "|---|---|---|\n" +
                "| `measured` | logged on the real robot (HardwareMonitor import, shot/pose logs, scale, stopwatch) | 5 |\n" +
                "| `game_manual` | official BIOBUZZ Competition Manual | 5 |\n" +
                "| `manual` | typed in by a person (spec sheet, CAD, estimate) | 4 |\n" +
                "| `sim` | computed by the `Sim` module from real inputs | 3 |\n" +
                "| `placeholder` | example numbers, or sim output computed from any placeholder input. **Never trust.** | 1 |\n" +
                "\n" +
                "An empty value (`NULL`) means \"not known yet\". `v_missing` lists what is still needed.\n" +
                "\n" +
                "## Tables\n" +
                "\n" +
                "| table | one row per | key columns |\n" +
                "|---|---|---|\n" +
                "| `meta` | setting | `key`, `value` (format, schema_version, season, robot_name, frames, units...) |\n" +
                "| `parameters` | single number or text | `key`, `value` / `text_value`, `unit`, `std_dev`, `role`, `required`, `source_id` |\n" +
                "| `curves` + `curve_points` | 1-D lookup table y(x) | `curve`, `x`, `y`, `y_std`, `n`; interpolate linearly, hold the end values outside the range |\n" +
                "| `surfaces` + `surface_points` | 2-D table z(x, y) | `surface`, `x`, `y`, `z`, `z_std`, `n`; bilinear between grid points |\n" +
                "| `distributions` + `distribution_samples` | random error / variation | `kind` (normal, uniform, triangular, lognormal, empirical, constant), `mean`, `std_dev`, `min`, `max`, `mode`, `per` |\n" +
                "| `actions` | timed game action | `mean_time_s`, `std_time_s`, `min_time_s`, `success_prob` |\n" +
                "| `shot_observations` | real shot (make/miss) | `ball`, `distance_m`, rpm/hood/battery..., `result` |\n" +
                "| `pose_observations` | real localization check | true vs estimated pose, `distance_driven_m`, `elapsed_s` |\n" +
                "| `disconnect_observations` | logged match or disconnect | `match`, `type` (none or a disconnect kind), `match_time_s`, `outage_s`, `humidity_pct`, `cause` |\n" +
                "| `sources` | batch of values | `kind`, `name`, `detail` (command, sessions, assumptions), `created_utc` |\n" +
                "\n" +
                "Views: `v_parameters` (parameters with their source), `v_everything` (one row per entry of any type with\n" +
                "`has_data`), `v_missing` (required and still empty), `v_placeholders` (filled with placeholder data).\n" +
                "\n" +
                "`role`: `input` = a property you put in, `derived` = computed by the sims from the inputs, `measured` =\n" +
                "from real-robot logs.\n" +
                "\n" +
                "Curve notes for game sims:\n" +
                "- Where both exist, prefer a `*.measured_*` curve (real logs) over the sim curve at x values whose\n" +
                "  `n` is large (about 20+ shots or checks per bin); fall back to the sim curve elsewhere.\n" +
                "- Make-probability curves cover 0.5 m out to the field diagonal. Distances the robot cannot reach\n" +
                "  are stored as 0, so holding the end value never invents makes.\n" +
                "- `launcher.<ball>.hood_vs_distance` / `target_rpm_vs_distance` may hold two points 1 mm apart where\n" +
                "  the best arc switches (a big hood change). Interpolate between neighbours as usual; the pair keeps\n" +
                "  the two arcs from being mixed.\n" +
                "\n" +
                "## Reading it\n" +
                "\n" +
                "```python\n" +
                "import sqlite3\n" +
                "db = sqlite3.connect(\"robot_profile.db\")\n" +
                "top = db.execute(\"SELECT value FROM parameters WHERE key='drive.max_speed_mps'\").fetchone()[0]\n" +
                "pts = db.execute(\"SELECT x, y FROM curve_points WHERE curve='drive.time_to_distance' ORDER BY x\").fetchall()\n" +
                "missing = db.execute(\"SELECT type, name FROM v_missing\").fetchall()\n" +
                "```\n" +
                "\n" +
                "```csharp\n" +
                "using var c = new SqliteConnection(\"Data Source=robot_profile.db\"); c.Open();\n" +
                "var cmd = c.CreateCommand(); cmd.CommandText = \"SELECT value FROM parameters WHERE key = $k\";\n" +
                "cmd.Parameters.AddWithValue(\"$k\", \"drive.max_speed_mps\"); var top = (double?)cmd.ExecuteScalar();\n" +
                "```\n" +
                "\n" +
                "Units are SI unless the name's suffix says otherwise (`_rpm`, `_deg`, `_dps`, `_mah`, `_ms`). Angles in the\n" +
                "data are degrees. Field frame: official FTC field coordinates (origin at the field center, +z up). Robot\n" +
                "frame: +x forward, +y left, yaw counter-clockwise positive.\n" +
                "\n" +
                "How a game sim typically uses it: drive moves with `drive.time_to_distance` x `drive.path_overhead_factor`\n" +
                "(or `drive.time_to_distance_vs_battery` as the battery drains), shots with\n" +
                "`launcher.<ball>.make_prob_vs_distance` / `burst_make_prob_vs_distance_battery` and\n" +
                "`launcher.<ball>.burst_time_vs_battery_v`, intake/placing with `actions`, aim error from\n" +
                "`localization.position_error_vs_time`, and randomness from `distributions`.\n" +
                "\n" +
                "## Contents\n");
        for (String[] cat : Catalog.CATEGORIES) {
            String c = cat[0];
            List<Catalog.Param> ps = Catalog.PARAMS.stream().filter(p -> p.category().equals(c)).collect(Collectors.toList());
            List<Catalog.Curve> cs = Catalog.CURVES.stream().filter(p -> p.category().equals(c)).collect(Collectors.toList());
            List<Catalog.Surface> ss = Catalog.SURFACES.stream().filter(p -> p.category().equals(c)).collect(Collectors.toList());
            List<Catalog.Dist> ds = Catalog.DISTS.stream().filter(p -> p.category().equals(c)).collect(Collectors.toList());
            List<Catalog.Action> as = Catalog.ACTIONS.stream().filter(p -> p.category().equals(c)).collect(Collectors.toList());
            if (ps.isEmpty() && cs.isEmpty() && ss.isEmpty() && ds.isEmpty() && as.isEmpty()) continue;
            sb.append("\n### `").append(c).append("`\n\n").append(cat[1]).append("\n\n");
            sb.append("| type | name | unit | role | req | description |\n|---|---|---|---|---|---|\n");
            for (Catalog.Param p : ps) {
                String pre = p.exampleKind() == SourceKind.GAME_MANUAL ? "**" + Csv.fmt(p.example()) + "** (manual). " : "";
                sb.append(line("parameter", p.key(), p.unit(), p.role().id(), p.required(), pre + p.description()));
            }
            for (Catalog.Curve x : cs) sb.append(line("curve", x.name(), x.xUnit() + " -> " + x.yUnit(), x.role().id(), x.required(), x.description()));
            for (Catalog.Surface x : ss) sb.append(line("surface", x.name(), x.xUnit() + ", " + x.yUnit() + " -> " + x.zUnit(), x.role().id(), x.required(), x.description()));
            for (Catalog.Dist x : ds) sb.append(line("distribution", x.name(), x.unit() + " per " + x.per(), x.role().id(), x.required(), x.description()));
            for (Catalog.Action x : as) sb.append(line("action", x.name(), "s", x.role().id(), x.required(), x.description()));
        }
        return sb.toString();
    }

    private static String line(String type, String name, String unit, String role, boolean req, String desc) {
        return "| " + type + " | `" + name + "` | " + unit.replace("|", "\\|") + " | " + role + " | " + (req ? "yes" : "") + " | "
                + desc.replace("|", "\\|") + " |\n";
    }
}
