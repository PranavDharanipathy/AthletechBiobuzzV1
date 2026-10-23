package org.athletech.sim.config;

import org.athletech.sim.undeterminables.Distribution;

import java.nio.file.Files;
import java.nio.file.Path;
import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.sql.Types;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The robot profile database: one SQLite file with every number a game simulator needs. Opening
 * it creates the schema if needed and adds a row for every {@link Catalog} entry that is missing
 * (existing values are never touched).
 *
 * <p>Every value remembers its {@link SourceKind}. A write from a lower-priority source does not
 * replace a higher-priority value unless forced, so sims cannot overwrite measurements.
 */
public final class ProfileDb implements AutoCloseable {

    public enum Write { WRITTEN, KEPT_HIGHER_PRIORITY, UNKNOWN }

    private final Connection c;
    private final Path path;

    private ProfileDb(Connection c, Path path) {
        this.c = c;
        this.path = path;
    }

    /** Opens (creating if needed) the profile at {@code path}. */
    public static ProfileDb open(Path path) throws SQLException {
        try {
            Path parent = path.toAbsolutePath().getParent();
            if (parent != null) Files.createDirectories(parent);
        } catch (java.io.IOException e) {
            throw new SQLException("cannot create folder for " + path + ": " + e.getMessage(), e);
        }
        Connection c = DriverManager.getConnection("jdbc:sqlite:" + path.toAbsolutePath());
        ProfileDb db = new ProfileDb(c, path);
        db.exec("PRAGMA foreign_keys = ON");
        db.exec("PRAGMA busy_timeout = 5000"); // a game sim may have the file open
        db.migrate();
        // only write when the catalog changed, so reading the profile never needs a write lock
        if (!Catalog.fingerprint().equals(db.meta("catalog_fingerprint"))) {
            db.syncCatalog();
            db.putMeta("catalog_fingerprint", Catalog.fingerprint());
        }
        return db;
    }

    public Path path() { return path; }

    @Override
    public void close() throws SQLException {
        c.close();
    }

    static String now() {
        return Instant.now().truncatedTo(ChronoUnit.SECONDS).toString();
    }

    private void exec(String sql) throws SQLException {
        try (Statement s = c.createStatement()) {
            s.execute(sql);
        }
    }

    /** Runs {@code body} as one transaction (much faster for bulk writes, all-or-nothing). */
    public <T> T transaction(SqlBody<T> body) throws SQLException {
        boolean auto = c.getAutoCommit();
        if (!auto) return body.run(); // already inside a transaction: join it
        c.setAutoCommit(false);
        try {
            T r = body.run();
            c.commit();
            return r;
        } catch (SQLException | RuntimeException e) {
            c.rollback();
            throw e;
        } finally {
            c.setAutoCommit(auto);
        }
    }

    @FunctionalInterface
    public interface SqlBody<T> {
        T run() throws SQLException;
    }

    // ------------------------------------------------------------------ schema + catalog

    private void migrate() throws SQLException {
        int version;
        try (Statement s = c.createStatement(); ResultSet rs = s.executeQuery("PRAGMA user_version")) {
            version = rs.next() ? rs.getInt(1) : 0;
        }
        if (version > ProfileSchema.VERSION) {
            throw new SQLException(path + " was written by a newer version of the sim (schema " + version
                    + ", this code understands " + ProfileSchema.VERSION + "). Update the Sim module.");
        }
        if (version == ProfileSchema.VERSION) return; // up to date: nothing to write
        transaction(() -> {
            for (String ddl : ProfileSchema.DDL) exec(ddl);
            // future migrations: if (version < 2) { ALTER TABLE ... }
            exec("PRAGMA user_version = " + ProfileSchema.VERSION);
            putMetaIfAbsent("format", ProfileSchema.FORMAT);
            putMetaIfAbsent("created_utc", now());
            putMetaIfAbsent("season", "BIOBUZZ (FTC 2026-2027)");
            putMetaIfAbsent("robot_name", "");
            putMetaIfAbsent("team_number", "");
            putMetaIfAbsent("units", "SI unless the name's suffix says otherwise (_rpm, _deg, _mah, _ms, ...).");
            putMetaIfAbsent("frames", "Field: official FTC field coordinates, origin at the field center, +z up. "
                    + "Robot: +x forward, +y left, yaw counter-clockwise positive. Angles in the data are degrees.");
            putMeta("schema_version", Integer.toString(ProfileSchema.VERSION));
            return null;
        });
    }

    /** Adds missing catalog rows and refreshes catalog metadata (units, descriptions). Values are kept. */
    public int syncCatalog() throws SQLException {
        return transaction(() -> {
            int added = 0;
            long manualSource = -1;
            for (String table : Arrays.asList("parameters", "curves", "surfaces", "distributions", "actions")) {
                exec("UPDATE " + table + " SET in_catalog = 0");
            }
            for (Catalog.Param p : Catalog.PARAMS) {
                boolean exists = exists("parameters", "key", p.key());
                if (!exists) {
                    try (PreparedStatement ps = c.prepareStatement(
                            "INSERT INTO parameters(key, category, unit, role, required, description, min_valid, max_valid, in_catalog) VALUES (?,?,?,?,?,?,?,?,1)")) {
                        ps.setString(1, p.key());
                        ps.setString(2, p.category());
                        ps.setString(3, p.unit());
                        ps.setString(4, p.role().id());
                        ps.setInt(5, p.required() ? 1 : 0);
                        ps.setString(6, p.description());
                        setNullable(ps, 7, p.min());
                        setNullable(ps, 8, p.max());
                        ps.executeUpdate();
                    }
                    added++;
                    if (p.exampleKind() == SourceKind.GAME_MANUAL && p.example() != null) {
                        if (manualSource < 0) manualSource = addSource(SourceKind.GAME_MANUAL, "FTC BIOBUZZ Competition Manual",
                                "Sections 9 and 10, ftc-resources.firstinspires.org/ftc/game");
                        writeParam(p.key(), p.example(), null, null, manualSource);
                    }
                } else {
                    try (PreparedStatement ps = c.prepareStatement(
                            "UPDATE parameters SET category=?, unit=?, role=?, required=?, description=?, min_valid=?, max_valid=?, in_catalog=1 WHERE key=?")) {
                        ps.setString(1, p.category());
                        ps.setString(2, p.unit());
                        ps.setString(3, p.role().id());
                        ps.setInt(4, p.required() ? 1 : 0);
                        ps.setString(5, p.description());
                        setNullable(ps, 6, p.min());
                        setNullable(ps, 7, p.max());
                        ps.setString(8, p.key());
                        ps.executeUpdate();
                    }
                }
            }
            for (Catalog.Curve x : Catalog.CURVES) {
                if (!exists("curves", "name", x.name())) added++;
                upsert("INSERT INTO curves(name, category, x_name, x_unit, y_name, y_unit, role, required, description, in_catalog) VALUES (?,?,?,?,?,?,?,?,?,1) "
                                + "ON CONFLICT(name) DO UPDATE SET category=excluded.category, x_name=excluded.x_name, x_unit=excluded.x_unit, y_name=excluded.y_name, "
                                + "y_unit=excluded.y_unit, role=excluded.role, required=excluded.required, description=excluded.description, in_catalog=1",
                        x.name(), x.category(), x.xName(), x.xUnit(), x.yName(), x.yUnit(), x.role().id(), x.required() ? 1 : 0, x.description());
            }
            for (Catalog.Surface x : Catalog.SURFACES) {
                if (!exists("surfaces", "name", x.name())) added++;
                upsert("INSERT INTO surfaces(name, category, x_name, x_unit, y_name, y_unit, z_name, z_unit, role, required, description, in_catalog) VALUES (?,?,?,?,?,?,?,?,?,?,?,1) "
                                + "ON CONFLICT(name) DO UPDATE SET category=excluded.category, x_name=excluded.x_name, x_unit=excluded.x_unit, y_name=excluded.y_name, "
                                + "y_unit=excluded.y_unit, z_name=excluded.z_name, z_unit=excluded.z_unit, role=excluded.role, required=excluded.required, "
                                + "description=excluded.description, in_catalog=1",
                        x.name(), x.category(), x.xName(), x.xUnit(), x.yName(), x.yUnit(), x.zName(), x.zUnit(), x.role().id(), x.required() ? 1 : 0, x.description());
            }
            for (Catalog.Dist x : Catalog.DISTS) {
                if (!exists("distributions", "name", x.name())) added++;
                upsert("INSERT INTO distributions(name, category, unit, per, role, required, description, in_catalog) VALUES (?,?,?,?,?,?,?,1) "
                                + "ON CONFLICT(name) DO UPDATE SET category=excluded.category, unit=excluded.unit, per=excluded.per, role=excluded.role, "
                                + "required=excluded.required, description=excluded.description, in_catalog=1",
                        x.name(), x.category(), x.unit(), x.per(), x.role().id(), x.required() ? 1 : 0, x.description());
            }
            for (Catalog.Action x : Catalog.ACTIONS) {
                if (!exists("actions", "name", x.name())) added++;
                upsert("INSERT INTO actions(name, category, role, required, description, in_catalog) VALUES (?,?,?,?,?,1) "
                                + "ON CONFLICT(name) DO UPDATE SET category=excluded.category, role=excluded.role, required=excluded.required, "
                                + "description=excluded.description, in_catalog=1",
                        x.name(), x.category(), x.role().id(), x.required() ? 1 : 0, x.description());
            }
            // entries that left the catalog keep their data but are no longer required
            for (String table : Arrays.asList("parameters", "curves", "surfaces", "distributions", "actions")) {
                exec("UPDATE " + table + " SET required = 0 WHERE in_catalog = 0");
            }
            return added;
        });
    }

    /** Fills every empty catalog entry that has an example value with it, marked as a placeholder. */
    public int fillExamples() throws SQLException {
        return transaction(() -> {
            long src = addSource(SourceKind.PLACEHOLDER, "catalog example values",
                    "profile init --example: generic FTC-robot numbers so the sims can run. Replace with real data.");
            int n = 0;
            for (Catalog.Param p : Catalog.PARAMS) {
                if (p.example() == null || p.exampleKind() != SourceKind.PLACEHOLDER) continue;
                ParamRow r = param(p.key());
                if (r != null && r.value() == null && r.text() == null) {
                    writeParam(p.key(), p.example(), null, null, src);
                    n++;
                }
            }
            for (Catalog.Dist d : Catalog.DISTS) {
                if (d.exampleKind() == null) continue;
                DistRow r = dist(d.name());
                if (r != null && r.kind() == null) {
                    writeDist(d.name(), d.exampleKind(), d.exampleMean(), d.exampleStd(), null, null, null, null, src);
                    n++;
                }
            }
            for (Catalog.Action a : Catalog.ACTIONS) {
                if (a.exampleMeanS() == null) continue;
                ActionRow r = action(a.name());
                if (r != null && r.meanTimeS() == null) {
                    writeAction(a.name(), a.exampleMeanS(), a.exampleStdS(), null, a.exampleSuccess(), src);
                    n++;
                }
            }
            return n;
        });
    }

    private boolean exists(String table, String keyCol, String key) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement("SELECT 1 FROM " + table + " WHERE " + keyCol + " = ?")) {
            ps.setString(1, key);
            try (ResultSet rs = ps.executeQuery()) {
                return rs.next();
            }
        }
    }

    private void upsert(String sql, Object... args) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement(sql)) {
            for (int i = 0; i < args.length; i++) bind(ps, i + 1, args[i]);
            ps.executeUpdate();
        }
    }

    private static void bind(PreparedStatement ps, int i, Object v) throws SQLException {
        if (v == null) ps.setNull(i, Types.NULL);
        else if (v instanceof Double) setNullable(ps, i, (Double) v);
        else if (v instanceof Integer) ps.setInt(i, (Integer) v);
        else if (v instanceof Long) ps.setLong(i, (Long) v);
        else ps.setString(i, v.toString());
    }

    private static void setNullable(PreparedStatement ps, int i, Double v) throws SQLException {
        if (v == null || v.isNaN() || v.isInfinite()) ps.setNull(i, Types.REAL);
        else ps.setDouble(i, v);
    }

    private static Double getNullable(ResultSet rs, String col) throws SQLException {
        double v = rs.getDouble(col);
        return rs.wasNull() ? null : v;
    }

    // ------------------------------------------------------------------ meta + sources

    public void putMeta(String key, String value) throws SQLException {
        upsert("INSERT INTO meta(key, value) VALUES (?, ?) ON CONFLICT(key) DO UPDATE SET value = excluded.value", key, value);
    }

    private void putMetaIfAbsent(String key, String value) throws SQLException {
        upsert("INSERT OR IGNORE INTO meta(key, value) VALUES (?, ?)", key, value);
    }

    public String meta(String key) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement("SELECT value FROM meta WHERE key = ?")) {
            ps.setString(1, key);
            try (ResultSet rs = ps.executeQuery()) {
                return rs.next() ? rs.getString(1) : null;
            }
        }
    }

    public Map<String, String> allMeta() throws SQLException {
        Map<String, String> m = new LinkedHashMap<>();
        try (Statement s = c.createStatement(); ResultSet rs = s.executeQuery("SELECT key, value FROM meta ORDER BY key")) {
            while (rs.next()) m.put(rs.getString(1), rs.getString(2));
        }
        return m;
    }

    /** Records where a batch of values came from; returns its id for the write calls. */
    public long addSource(SourceKind kind, String name, String detail) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement(
                "INSERT INTO sources(kind, name, detail, created_utc) VALUES (?,?,?,?)", Statement.RETURN_GENERATED_KEYS)) {
            ps.setString(1, kind.id());
            ps.setString(2, name);
            ps.setString(3, detail);
            ps.setString(4, now());
            ps.executeUpdate();
            try (ResultSet rs = ps.getGeneratedKeys()) {
                rs.next();
                return rs.getLong(1);
            }
        }
    }

    public SourceKind sourceKind(long sourceId) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement("SELECT kind FROM sources WHERE id = ?")) {
            ps.setLong(1, sourceId);
            try (ResultSet rs = ps.executeQuery()) {
                if (!rs.next()) throw new SQLException("no source with id " + sourceId);
                return SourceKind.parse(rs.getString(1));
            }
        }
    }

    public static final class SourceRow {
        private final long id;
        private final SourceKind kind;
        private final String name;
        private final String detail;
        private final String createdUtc;

        public SourceRow(long id, SourceKind kind, String name, String detail, String createdUtc) {
            this.id = id;
            this.kind = kind;
            this.name = name;
            this.detail = detail;
            this.createdUtc = createdUtc;
        }

        public long id() { return id; }
        public SourceKind kind() { return kind; }
        public String name() { return name; }
        public String detail() { return detail; }
        public String createdUtc() { return createdUtc; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof SourceRow)) return false;
            SourceRow o = (SourceRow) other;
            return id == o.id
                    && java.util.Objects.equals(kind, o.kind)
                    && java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(detail, o.detail)
                    && java.util.Objects.equals(createdUtc, o.createdUtc);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(id, kind, name, detail, createdUtc);
        }

        @Override
        public String toString() {
            return "SourceRow[id=" + id + ", kind=" + kind + ", name=" + name + ", detail=" + detail + ", createdUtc=" + createdUtc + "]";
        }
    }

    public List<SourceRow> sources() throws SQLException {
        List<SourceRow> out = new ArrayList<>();
        try (Statement s = c.createStatement(); ResultSet rs = s.executeQuery("SELECT * FROM sources ORDER BY id")) {
            while (rs.next()) out.add(new SourceRow(rs.getLong("id"), SourceKind.parse(rs.getString("kind")),
                    rs.getString("name"), rs.getString("detail"), rs.getString("created_utc")));
        }
        return out;
    }

    /** Whether a write from {@code newSource} may replace a value currently from {@code existingSourceId}. */
    private boolean mayReplace(Long existingSourceId, long newSource, boolean force) throws SQLException {
        if (force || existingSourceId == null) return true;
        return sourceKind(newSource).priority >= sourceKind(existingSourceId).priority;
    }

    private Long currentSource(String table, String keyCol, String key) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement("SELECT source_id FROM " + table + " WHERE " + keyCol + " = ?")) {
            ps.setString(1, key);
            try (ResultSet rs = ps.executeQuery()) {
                if (!rs.next()) return null;
                long v = rs.getLong(1);
                return rs.wasNull() ? null : v;
            }
        }
    }

    private void touch() throws SQLException {
        putMeta("updated_utc", now());
    }

    // ------------------------------------------------------------------ parameters

    public static final class ParamRow {
        private final String key;
        private final String category;
        private final Double value;
        private final String text;
        private final String unit;
        private final Double std;
        private final Double min;
        private final Double max;
        private final String role;
        private final boolean required;
        private final boolean inCatalog;
        private final String description;
        private final SourceKind sourceKind;
        private final String sourceName;
        private final String updated;

        public ParamRow(String key, String category, Double value, String text, String unit, Double std, Double min, Double max, String role, boolean required, boolean inCatalog, String description, SourceKind sourceKind, String sourceName, String updated) {
            this.key = key;
            this.category = category;
            this.value = value;
            this.text = text;
            this.unit = unit;
            this.std = std;
            this.min = min;
            this.max = max;
            this.role = role;
            this.required = required;
            this.inCatalog = inCatalog;
            this.description = description;
            this.sourceKind = sourceKind;
            this.sourceName = sourceName;
            this.updated = updated;
        }

        public String key() { return key; }
        public String category() { return category; }
        public Double value() { return value; }
        public String text() { return text; }
        public String unit() { return unit; }
        public Double std() { return std; }
        public Double min() { return min; }
        public Double max() { return max; }
        public String role() { return role; }
        public boolean required() { return required; }
        public boolean inCatalog() { return inCatalog; }
        public String description() { return description; }
        public SourceKind sourceKind() { return sourceKind; }
        public String sourceName() { return sourceName; }
        public String updated() { return updated; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof ParamRow)) return false;
            ParamRow o = (ParamRow) other;
            return java.util.Objects.equals(key, o.key)
                    && java.util.Objects.equals(category, o.category)
                    && java.util.Objects.equals(value, o.value)
                    && java.util.Objects.equals(text, o.text)
                    && java.util.Objects.equals(unit, o.unit)
                    && java.util.Objects.equals(std, o.std)
                    && java.util.Objects.equals(min, o.min)
                    && java.util.Objects.equals(max, o.max)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && inCatalog == o.inCatalog
                    && java.util.Objects.equals(description, o.description)
                    && java.util.Objects.equals(sourceKind, o.sourceKind)
                    && java.util.Objects.equals(sourceName, o.sourceName)
                    && java.util.Objects.equals(updated, o.updated);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(key, category, value, text, unit, std, min, max, role, required, inCatalog, description, sourceKind, sourceName, updated);
        }

        @Override
        public String toString() {
            return "ParamRow[key=" + key + ", category=" + category + ", value=" + value + ", text=" + text + ", unit=" + unit + ", std=" + std + ", min=" + min + ", max=" + max + ", role=" + role + ", required=" + required + ", inCatalog=" + inCatalog + ", description=" + description + ", sourceKind=" + sourceKind + ", sourceName=" + sourceName + ", updated=" + updated + "]";
        }

        public boolean hasData() { return value != null || text != null; }

        public boolean outOfRange(double v) {
            return (min != null && v < min) || (max != null && v > max);
        }
    
    }

    private static final String PARAM_SELECT = "SELECT p.*, s.kind AS skind, s.name AS sname FROM parameters p LEFT JOIN sources s ON s.id = p.source_id";

    private static ParamRow paramRow(ResultSet rs) throws SQLException {
        String kind = rs.getString("skind");
        return new ParamRow(rs.getString("key"), rs.getString("category"), getNullable(rs, "value"), rs.getString("text_value"),
                rs.getString("unit"), getNullable(rs, "std_dev"), getNullable(rs, "min_valid"), getNullable(rs, "max_valid"),
                rs.getString("role"), rs.getInt("required") == 1, rs.getInt("in_catalog") == 1, rs.getString("description"),
                kind == null ? null : SourceKind.parse(kind), rs.getString("sname"), rs.getString("updated_utc"));
    }

    public ParamRow param(String key) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement(PARAM_SELECT + " WHERE p.key = ?")) {
            ps.setString(1, key);
            try (ResultSet rs = ps.executeQuery()) {
                return rs.next() ? paramRow(rs) : null;
            }
        }
    }

    public List<ParamRow> params(String prefix) throws SQLException {
        List<ParamRow> out = new ArrayList<>();
        try (PreparedStatement ps = c.prepareStatement(PARAM_SELECT + " WHERE p.key LIKE ? ORDER BY p.key")) {
            ps.setString(1, (prefix == null ? "" : prefix) + "%");
            try (ResultSet rs = ps.executeQuery()) {
                while (rs.next()) out.add(paramRow(rs));
            }
        }
        return out;
    }

    /** Number value of a parameter, or null if empty / unknown. */
    public Double value(String key) throws SQLException {
        ParamRow r = param(key);
        return r == null ? null : r.value();
    }

    public Write setParam(String key, Double value, String text, Double std, long sourceId, boolean force) throws SQLException {
        if (!exists("parameters", "key", key)) return Write.UNKNOWN;
        if (!mayReplace(currentSource("parameters", "key", key), sourceId, force)) return Write.KEPT_HIGHER_PRIORITY;
        writeParam(key, value, text, std, sourceId);
        touch();
        return Write.WRITTEN;
    }

    /** Adds a parameter that is not in the catalog (team-specific extras). */
    public void addCustomParam(String key, String unit, String description) throws SQLException {
        upsert("INSERT OR IGNORE INTO parameters(key, category, unit, role, required, description, in_catalog) VALUES (?,?,?,?,0,?,0)",
                key, Catalog.category(key), unit, "input", description == null ? "" : description);
    }

    private void writeParam(String key, Double value, String text, Double std, long sourceId) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement(
                "UPDATE parameters SET value=?, text_value=?, std_dev=?, source_id=?, updated_utc=? WHERE key=?")) {
            setNullable(ps, 1, value);
            ps.setString(2, text);
            setNullable(ps, 3, std);
            ps.setLong(4, sourceId);
            ps.setString(5, now());
            ps.setString(6, key);
            ps.executeUpdate();
        }
    }

    /** Empties a parameter (back to "unknown"). */
    public void clearParam(String key) throws SQLException {
        upsert("UPDATE parameters SET value=NULL, text_value=NULL, std_dev=NULL, source_id=NULL, updated_utc=? WHERE key=?", now(), key);
        touch();
    }

    /** Empties any entry (parameter, curve, surface, distribution or action). Returns its type, or null. */
    public String clear(String name) throws SQLException {
        return transaction(() -> {
            if (exists("parameters", "key", name)) {
                clearParam(name);
                return "parameter";
            }
            if (exists("curves", "name", name)) {
                upsert("DELETE FROM curve_points WHERE curve = ?", name);
                upsert("UPDATE curves SET source_id = NULL, updated_utc = ? WHERE name = ?", now(), name);
                touch();
                return "curve";
            }
            if (exists("surfaces", "name", name)) {
                upsert("DELETE FROM surface_points WHERE surface = ?", name);
                upsert("UPDATE surfaces SET source_id = NULL, updated_utc = ? WHERE name = ?", now(), name);
                touch();
                return "surface";
            }
            if (exists("distributions", "name", name)) {
                upsert("DELETE FROM distribution_samples WHERE distribution = ?", name);
                upsert("UPDATE distributions SET kind=NULL, mean=NULL, std_dev=NULL, min=NULL, max=NULL, mode=NULL, source_id=NULL, updated_utc=? WHERE name=?", now(), name);
                touch();
                return "distribution";
            }
            if (exists("actions", "name", name)) {
                upsert("UPDATE actions SET mean_time_s=NULL, std_time_s=NULL, min_time_s=NULL, success_prob=NULL, source_id=NULL, updated_utc=? WHERE name=?", now(), name);
                touch();
                return "action";
            }
            return null;
        });
    }

    // ------------------------------------------------------------------ curves

    public static final class CurveRow {
        private final String name;
        private final String category;
        private final String xName;
        private final String xUnit;
        private final String yName;
        private final String yUnit;
        private final String role;
        private final boolean required;
        private final String description;
        private final SourceKind sourceKind;
        private final String sourceName;
        private final String updated;
        private final List<double[]> points;

        public CurveRow(String name, String category, String xName, String xUnit, String yName, String yUnit, String role, boolean required, String description, SourceKind sourceKind, String sourceName, String updated, List<double[]> points) {
            this.name = name;
            this.category = category;
            this.xName = xName;
            this.xUnit = xUnit;
            this.yName = yName;
            this.yUnit = yUnit;
            this.role = role;
            this.required = required;
            this.description = description;
            this.sourceKind = sourceKind;
            this.sourceName = sourceName;
            this.updated = updated;
            this.points = points;
        }

        public String name() { return name; }
        public String category() { return category; }
        public String xName() { return xName; }
        public String xUnit() { return xUnit; }
        public String yName() { return yName; }
        public String yUnit() { return yUnit; }
        public String role() { return role; }
        public boolean required() { return required; }
        public String description() { return description; }
        public SourceKind sourceKind() { return sourceKind; }
        public String sourceName() { return sourceName; }
        public String updated() { return updated; }
        public List<double[]> points() { return points; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof CurveRow)) return false;
            CurveRow o = (CurveRow) other;
            return java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(category, o.category)
                    && java.util.Objects.equals(xName, o.xName)
                    && java.util.Objects.equals(xUnit, o.xUnit)
                    && java.util.Objects.equals(yName, o.yName)
                    && java.util.Objects.equals(yUnit, o.yUnit)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && java.util.Objects.equals(description, o.description)
                    && java.util.Objects.equals(sourceKind, o.sourceKind)
                    && java.util.Objects.equals(sourceName, o.sourceName)
                    && java.util.Objects.equals(updated, o.updated)
                    && java.util.Objects.equals(points, o.points);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, category, xName, xUnit, yName, yUnit, role, required, description, sourceKind, sourceName, updated, points);
        }

        @Override
        public String toString() {
            return "CurveRow[name=" + name + ", category=" + category + ", xName=" + xName + ", xUnit=" + xUnit + ", yName=" + yName + ", yUnit=" + yUnit + ", role=" + role + ", required=" + required + ", description=" + description + ", sourceKind=" + sourceKind + ", sourceName=" + sourceName + ", updated=" + updated + ", points=" + points + "]";
        }

        public boolean hasData() { return !points.isEmpty(); }

        public double[] xs() { return points.stream().mapToDouble(p -> p[0]).toArray(); }

        public double[] ys() { return points.stream().mapToDouble(p -> p[1]).toArray(); }
    
    }

    public CurveRow curve(String name) throws SQLException {
        List<CurveRow> l = curves(name, true);
        return l.isEmpty() ? null : l.get(0);
    }

    public List<CurveRow> curves(String prefix) throws SQLException {
        return curves(prefix, false);
    }

    private List<CurveRow> curves(String key, boolean exact) throws SQLException {
        List<CurveRow> out = new ArrayList<>();
        String sql = "SELECT k.*, s.kind AS skind, s.name AS sname FROM curves k LEFT JOIN sources s ON s.id = k.source_id WHERE k.name "
                + (exact ? "= ?" : "LIKE ?") + " ORDER BY k.name";
        try (PreparedStatement ps = c.prepareStatement(sql)) {
            ps.setString(1, exact ? key : (key == null ? "" : key) + "%");
            try (ResultSet rs = ps.executeQuery()) {
                while (rs.next()) {
                    String kind = rs.getString("skind");
                    out.add(new CurveRow(rs.getString("name"), rs.getString("category"), rs.getString("x_name"), rs.getString("x_unit"),
                            rs.getString("y_name"), rs.getString("y_unit"), rs.getString("role"), rs.getInt("required") == 1,
                            rs.getString("description"), kind == null ? null : SourceKind.parse(kind), rs.getString("sname"),
                            rs.getString("updated_utc"), new ArrayList<>()));
                }
            }
        }
        for (CurveRow r : out) {
            try (PreparedStatement ps = c.prepareStatement("SELECT x, y, y_std, n FROM curve_points WHERE curve = ? ORDER BY x")) {
                ps.setString(1, r.name());
                try (ResultSet rs = ps.executeQuery()) {
                    while (rs.next()) {
                        Double ys = getNullable(rs, "y_std");
                        double n = rs.getDouble("n");
                        boolean nNull = rs.wasNull();
                        r.points().add(new double[]{rs.getDouble("x"), rs.getDouble("y"), ys == null ? Double.NaN : ys, nNull ? Double.NaN : n});
                    }
                }
            }
        }
        return out;
    }

    /** Replaces a curve's points. Each point is {x, y} or {x, y, yStd} or {x, y, yStd, n}. */
    public Write setCurve(String name, List<double[]> points, long sourceId, boolean force) throws SQLException {
        if (!exists("curves", "name", name)) return Write.UNKNOWN;
        if (!mayReplace(currentSource("curves", "name", name), sourceId, force)) return Write.KEPT_HIGHER_PRIORITY;
        transaction(() -> {
            upsert("DELETE FROM curve_points WHERE curve = ?", name);
            try (PreparedStatement ps = c.prepareStatement("INSERT OR REPLACE INTO curve_points(curve, x, y, y_std, n) VALUES (?,?,?,?,?)")) {
                for (double[] p : points) {
                    if (Double.isNaN(p[0]) || Double.isNaN(p[1])) continue;
                    ps.setString(1, name);
                    ps.setDouble(2, p[0]);
                    ps.setDouble(3, p[1]);
                    setNullable(ps, 4, p.length > 2 ? p[2] : null);
                    if (p.length > 3 && !Double.isNaN(p[3])) ps.setLong(5, Math.round(p[3]));
                    else ps.setNull(5, Types.INTEGER);
                    ps.addBatch();
                }
                ps.executeBatch();
            }
            upsert("UPDATE curves SET source_id = ?, updated_utc = ? WHERE name = ?", sourceId, now(), name);
            touch();
            return null;
        });
        return Write.WRITTEN;
    }

    // ------------------------------------------------------------------ surfaces

    public static final class SurfaceRow {
        private final String name;
        private final String category;
        private final String xName;
        private final String xUnit;
        private final String yName;
        private final String yUnit;
        private final String zName;
        private final String zUnit;
        private final String role;
        private final boolean required;
        private final String description;
        private final SourceKind sourceKind;
        private final String sourceName;
        private final String updated;
        private final List<double[]> points;

        public SurfaceRow(String name, String category, String xName, String xUnit, String yName, String yUnit, String zName, String zUnit, String role, boolean required, String description, SourceKind sourceKind, String sourceName, String updated, List<double[]> points) {
            this.name = name;
            this.category = category;
            this.xName = xName;
            this.xUnit = xUnit;
            this.yName = yName;
            this.yUnit = yUnit;
            this.zName = zName;
            this.zUnit = zUnit;
            this.role = role;
            this.required = required;
            this.description = description;
            this.sourceKind = sourceKind;
            this.sourceName = sourceName;
            this.updated = updated;
            this.points = points;
        }

        public String name() { return name; }
        public String category() { return category; }
        public String xName() { return xName; }
        public String xUnit() { return xUnit; }
        public String yName() { return yName; }
        public String yUnit() { return yUnit; }
        public String zName() { return zName; }
        public String zUnit() { return zUnit; }
        public String role() { return role; }
        public boolean required() { return required; }
        public String description() { return description; }
        public SourceKind sourceKind() { return sourceKind; }
        public String sourceName() { return sourceName; }
        public String updated() { return updated; }
        public List<double[]> points() { return points; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof SurfaceRow)) return false;
            SurfaceRow o = (SurfaceRow) other;
            return java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(category, o.category)
                    && java.util.Objects.equals(xName, o.xName)
                    && java.util.Objects.equals(xUnit, o.xUnit)
                    && java.util.Objects.equals(yName, o.yName)
                    && java.util.Objects.equals(yUnit, o.yUnit)
                    && java.util.Objects.equals(zName, o.zName)
                    && java.util.Objects.equals(zUnit, o.zUnit)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && java.util.Objects.equals(description, o.description)
                    && java.util.Objects.equals(sourceKind, o.sourceKind)
                    && java.util.Objects.equals(sourceName, o.sourceName)
                    && java.util.Objects.equals(updated, o.updated)
                    && java.util.Objects.equals(points, o.points);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, category, xName, xUnit, yName, yUnit, zName, zUnit, role, required, description, sourceKind, sourceName, updated, points);
        }

        @Override
        public String toString() {
            return "SurfaceRow[name=" + name + ", category=" + category + ", xName=" + xName + ", xUnit=" + xUnit + ", yName=" + yName + ", yUnit=" + yUnit + ", zName=" + zName + ", zUnit=" + zUnit + ", role=" + role + ", required=" + required + ", description=" + description + ", sourceKind=" + sourceKind + ", sourceName=" + sourceName + ", updated=" + updated + ", points=" + points + "]";
        }

        public boolean hasData() { return !points.isEmpty(); }
    
    }

    public List<SurfaceRow> surfaces(String prefix) throws SQLException {
        List<SurfaceRow> out = new ArrayList<>();
        try (PreparedStatement ps = c.prepareStatement("SELECT f.*, s.kind AS skind, s.name AS sname FROM surfaces f LEFT JOIN sources s "
                + "ON s.id = f.source_id WHERE f.name LIKE ? ORDER BY f.name")) {
            ps.setString(1, (prefix == null ? "" : prefix) + "%");
            try (ResultSet rs = ps.executeQuery()) {
                while (rs.next()) {
                    String kind = rs.getString("skind");
                    out.add(new SurfaceRow(rs.getString("name"), rs.getString("category"), rs.getString("x_name"), rs.getString("x_unit"),
                            rs.getString("y_name"), rs.getString("y_unit"), rs.getString("z_name"), rs.getString("z_unit"),
                            rs.getString("role"), rs.getInt("required") == 1, rs.getString("description"),
                            kind == null ? null : SourceKind.parse(kind), rs.getString("sname"), rs.getString("updated_utc"), new ArrayList<>()));
                }
            }
        }
        for (SurfaceRow r : out) {
            try (PreparedStatement ps = c.prepareStatement("SELECT x, y, z, z_std, n FROM surface_points WHERE surface = ? ORDER BY y, x")) {
                ps.setString(1, r.name());
                try (ResultSet rs = ps.executeQuery()) {
                    while (rs.next()) {
                        Double zs = getNullable(rs, "z_std");
                        double n = rs.getDouble("n");
                        boolean nNull = rs.wasNull();
                        r.points().add(new double[]{rs.getDouble("x"), rs.getDouble("y"), rs.getDouble("z"),
                                zs == null ? Double.NaN : zs, nNull ? Double.NaN : n});
                    }
                }
            }
        }
        return out;
    }

    public SurfaceRow surface(String name) throws SQLException {
        for (SurfaceRow r : surfaces(name)) if (r.name().equals(name)) return r;
        return null;
    }

    /** Replaces a surface's points: {x, y, z} or {x, y, z, zStd} or {x, y, z, zStd, n}. */
    public Write setSurface(String name, List<double[]> points, long sourceId, boolean force) throws SQLException {
        if (!exists("surfaces", "name", name)) return Write.UNKNOWN;
        if (!mayReplace(currentSource("surfaces", "name", name), sourceId, force)) return Write.KEPT_HIGHER_PRIORITY;
        transaction(() -> {
            upsert("DELETE FROM surface_points WHERE surface = ?", name);
            try (PreparedStatement ps = c.prepareStatement("INSERT OR REPLACE INTO surface_points(surface, x, y, z, z_std, n) VALUES (?,?,?,?,?,?)")) {
                for (double[] p : points) {
                    if (Double.isNaN(p[2])) continue;
                    ps.setString(1, name);
                    ps.setDouble(2, p[0]);
                    ps.setDouble(3, p[1]);
                    ps.setDouble(4, p[2]);
                    setNullable(ps, 5, p.length > 3 ? p[3] : null);
                    if (p.length > 4 && !Double.isNaN(p[4])) ps.setLong(6, Math.round(p[4]));
                    else ps.setNull(6, Types.INTEGER);
                    ps.addBatch();
                }
                ps.executeBatch();
            }
            upsert("UPDATE surfaces SET source_id = ?, updated_utc = ? WHERE name = ?", sourceId, now(), name);
            touch();
            return null;
        });
        return Write.WRITTEN;
    }

    // ------------------------------------------------------------------ distributions

    public static final class DistRow {
        private final String name;
        private final String category;
        private final String kind;
        private final String unit;
        private final String per;
        private final Double mean;
        private final Double std;
        private final Double min;
        private final Double max;
        private final Double mode;
        private final double[] samples;
        private final String role;
        private final boolean required;
        private final String description;
        private final SourceKind sourceKind;
        private final String sourceName;
        private final String updated;

        public DistRow(String name, String category, String kind, String unit, String per, Double mean, Double std, Double min, Double max, Double mode, double[] samples, String role, boolean required, String description, SourceKind sourceKind, String sourceName, String updated) {
            this.name = name;
            this.category = category;
            this.kind = kind;
            this.unit = unit;
            this.per = per;
            this.mean = mean;
            this.std = std;
            this.min = min;
            this.max = max;
            this.mode = mode;
            this.samples = samples;
            this.role = role;
            this.required = required;
            this.description = description;
            this.sourceKind = sourceKind;
            this.sourceName = sourceName;
            this.updated = updated;
        }

        public String name() { return name; }
        public String category() { return category; }
        public String kind() { return kind; }
        public String unit() { return unit; }
        public String per() { return per; }
        public Double mean() { return mean; }
        public Double std() { return std; }
        public Double min() { return min; }
        public Double max() { return max; }
        public Double mode() { return mode; }
        public double[] samples() { return samples; }
        public String role() { return role; }
        public boolean required() { return required; }
        public String description() { return description; }
        public SourceKind sourceKind() { return sourceKind; }
        public String sourceName() { return sourceName; }
        public String updated() { return updated; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof DistRow)) return false;
            DistRow o = (DistRow) other;
            return java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(category, o.category)
                    && java.util.Objects.equals(kind, o.kind)
                    && java.util.Objects.equals(unit, o.unit)
                    && java.util.Objects.equals(per, o.per)
                    && java.util.Objects.equals(mean, o.mean)
                    && java.util.Objects.equals(std, o.std)
                    && java.util.Objects.equals(min, o.min)
                    && java.util.Objects.equals(max, o.max)
                    && java.util.Objects.equals(mode, o.mode)
                    && java.util.Objects.equals(samples, o.samples)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && java.util.Objects.equals(description, o.description)
                    && java.util.Objects.equals(sourceKind, o.sourceKind)
                    && java.util.Objects.equals(sourceName, o.sourceName)
                    && java.util.Objects.equals(updated, o.updated);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, category, kind, unit, per, mean, std, min, max, mode, samples, role, required, description, sourceKind, sourceName, updated);
        }

        @Override
        public String toString() {
            return "DistRow[name=" + name + ", category=" + category + ", kind=" + kind + ", unit=" + unit + ", per=" + per + ", mean=" + mean + ", std=" + std + ", min=" + min + ", max=" + max + ", mode=" + mode + ", samples=" + samples + ", role=" + role + ", required=" + required + ", description=" + description + ", sourceKind=" + sourceKind + ", sourceName=" + sourceName + ", updated=" + updated + "]";
        }

        public boolean hasData() { return kind != null; }

        public Distribution toDistribution() {
            if (kind == null) return Distribution.none(name, unit);
            return new Distribution(name, Distribution.Kind.parse(kind), nan(mean), nan(std), nan(min), nan(max), nan(mode), samples, unit);
        }

        private static double nan(Double d) {
            return d == null ? Double.NaN : d;
        }
    
    }

    public List<DistRow> dists(String prefix) throws SQLException {
        List<DistRow> out = new ArrayList<>();
        try (PreparedStatement ps = c.prepareStatement("SELECT d.*, s.kind AS skind, s.name AS sname FROM distributions d LEFT JOIN sources s "
                + "ON s.id = d.source_id WHERE d.name LIKE ? ORDER BY d.name")) {
            ps.setString(1, (prefix == null ? "" : prefix) + "%");
            try (ResultSet rs = ps.executeQuery()) {
                while (rs.next()) {
                    String kind = rs.getString("skind");
                    out.add(new DistRow(rs.getString("name"), rs.getString("category"), rs.getString("kind"), rs.getString("unit"),
                            rs.getString("per"), getNullable(rs, "mean"), getNullable(rs, "std_dev"), getNullable(rs, "min"),
                            getNullable(rs, "max"), getNullable(rs, "mode"), null, rs.getString("role"), rs.getInt("required") == 1,
                            rs.getString("description"), kind == null ? null : SourceKind.parse(kind), rs.getString("sname"),
                            rs.getString("updated_utc")));
                }
            }
        }
        List<DistRow> full = new ArrayList<>();
        for (DistRow r : out) {
            List<Double> s = new ArrayList<>();
            try (PreparedStatement ps = c.prepareStatement("SELECT value FROM distribution_samples WHERE distribution = ? ORDER BY rowid")) {
                ps.setString(1, r.name());
                try (ResultSet rs = ps.executeQuery()) {
                    while (rs.next()) s.add(rs.getDouble(1));
                }
            }
            full.add(new DistRow(r.name(), r.category(), r.kind(), r.unit(), r.per(), r.mean(), r.std(), r.min(), r.max(), r.mode(),
                    s.stream().mapToDouble(Double::doubleValue).toArray(), r.role(), r.required(), r.description(),
                    r.sourceKind(), r.sourceName(), r.updated()));
        }
        return full;
    }

    public DistRow dist(String name) throws SQLException {
        for (DistRow r : dists(name)) if (r.name().equals(name)) return r;
        return null;
    }

    public Write setDist(String name, String kind, Double mean, Double std, Double min, Double max, Double mode,
                         double[] samples, long sourceId, boolean force) throws SQLException {
        if (!exists("distributions", "name", name)) return Write.UNKNOWN;
        if (!mayReplace(currentSource("distributions", "name", name), sourceId, force)) return Write.KEPT_HIGHER_PRIORITY;
        transaction(() -> {
            writeDist(name, kind, mean, std, min, max, mode, samples, sourceId);
            return null;
        });
        touch();
        return Write.WRITTEN;
    }

    private void writeDist(String name, String kind, Double mean, Double std, Double min, Double max, Double mode,
                           double[] samples, long sourceId) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement(
                "UPDATE distributions SET kind=?, mean=?, std_dev=?, min=?, max=?, mode=?, source_id=?, updated_utc=? WHERE name=?")) {
            ps.setString(1, kind == null ? null : kind.toLowerCase(java.util.Locale.ROOT));
            setNullable(ps, 2, mean);
            setNullable(ps, 3, std);
            setNullable(ps, 4, min);
            setNullable(ps, 5, max);
            setNullable(ps, 6, mode);
            ps.setLong(7, sourceId);
            ps.setString(8, now());
            ps.setString(9, name);
            ps.executeUpdate();
        }
        upsert("DELETE FROM distribution_samples WHERE distribution = ?", name);
        if (samples != null && samples.length > 0) {
            try (PreparedStatement ps = c.prepareStatement("INSERT INTO distribution_samples(distribution, value) VALUES (?, ?)")) {
                for (double v : samples) {
                    ps.setString(1, name);
                    ps.setDouble(2, v);
                    ps.addBatch();
                }
                ps.executeBatch();
            }
        }
    }

    /** The distribution as a sampler; {@link Distribution.Kind#NONE} if it has no data. */
    public Distribution distribution(String name) throws SQLException {
        DistRow r = dist(name);
        return r == null ? Distribution.none(name, "") : r.toDistribution();
    }

    // ------------------------------------------------------------------ actions

    public static final class ActionRow {
        private final String name;
        private final String category;
        private final Double meanTimeS;
        private final Double stdTimeS;
        private final Double minTimeS;
        private final Double successProb;
        private final String role;
        private final boolean required;
        private final String description;
        private final SourceKind sourceKind;
        private final String sourceName;
        private final String updated;

        public ActionRow(String name, String category, Double meanTimeS, Double stdTimeS, Double minTimeS, Double successProb, String role, boolean required, String description, SourceKind sourceKind, String sourceName, String updated) {
            this.name = name;
            this.category = category;
            this.meanTimeS = meanTimeS;
            this.stdTimeS = stdTimeS;
            this.minTimeS = minTimeS;
            this.successProb = successProb;
            this.role = role;
            this.required = required;
            this.description = description;
            this.sourceKind = sourceKind;
            this.sourceName = sourceName;
            this.updated = updated;
        }

        public String name() { return name; }
        public String category() { return category; }
        public Double meanTimeS() { return meanTimeS; }
        public Double stdTimeS() { return stdTimeS; }
        public Double minTimeS() { return minTimeS; }
        public Double successProb() { return successProb; }
        public String role() { return role; }
        public boolean required() { return required; }
        public String description() { return description; }
        public SourceKind sourceKind() { return sourceKind; }
        public String sourceName() { return sourceName; }
        public String updated() { return updated; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof ActionRow)) return false;
            ActionRow o = (ActionRow) other;
            return java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(category, o.category)
                    && java.util.Objects.equals(meanTimeS, o.meanTimeS)
                    && java.util.Objects.equals(stdTimeS, o.stdTimeS)
                    && java.util.Objects.equals(minTimeS, o.minTimeS)
                    && java.util.Objects.equals(successProb, o.successProb)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && java.util.Objects.equals(description, o.description)
                    && java.util.Objects.equals(sourceKind, o.sourceKind)
                    && java.util.Objects.equals(sourceName, o.sourceName)
                    && java.util.Objects.equals(updated, o.updated);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, category, meanTimeS, stdTimeS, minTimeS, successProb, role, required, description, sourceKind, sourceName, updated);
        }

        @Override
        public String toString() {
            return "ActionRow[name=" + name + ", category=" + category + ", meanTimeS=" + meanTimeS + ", stdTimeS=" + stdTimeS + ", minTimeS=" + minTimeS + ", successProb=" + successProb + ", role=" + role + ", required=" + required + ", description=" + description + ", sourceKind=" + sourceKind + ", sourceName=" + sourceName + ", updated=" + updated + "]";
        }

        public boolean hasData() { return meanTimeS != null; }
    
    }

    public List<ActionRow> actions(String prefix) throws SQLException {
        List<ActionRow> out = new ArrayList<>();
        try (PreparedStatement ps = c.prepareStatement("SELECT a.*, s.kind AS skind, s.name AS sname FROM actions a LEFT JOIN sources s "
                + "ON s.id = a.source_id WHERE a.name LIKE ? ORDER BY a.name")) {
            ps.setString(1, (prefix == null ? "" : prefix) + "%");
            try (ResultSet rs = ps.executeQuery()) {
                while (rs.next()) {
                    String kind = rs.getString("skind");
                    out.add(new ActionRow(rs.getString("name"), rs.getString("category"), getNullable(rs, "mean_time_s"),
                            getNullable(rs, "std_time_s"), getNullable(rs, "min_time_s"), getNullable(rs, "success_prob"),
                            rs.getString("role"), rs.getInt("required") == 1, rs.getString("description"),
                            kind == null ? null : SourceKind.parse(kind), rs.getString("sname"), rs.getString("updated_utc")));
                }
            }
        }
        return out;
    }

    public ActionRow action(String name) throws SQLException {
        for (ActionRow r : actions(name)) if (r.name().equals(name)) return r;
        return null;
    }

    public Write setAction(String name, Double meanS, Double stdS, Double minS, Double success, long sourceId, boolean force) throws SQLException {
        if (!exists("actions", "name", name)) return Write.UNKNOWN;
        if (!mayReplace(currentSource("actions", "name", name), sourceId, force)) return Write.KEPT_HIGHER_PRIORITY;
        writeAction(name, meanS, stdS, minS, success, sourceId);
        touch();
        return Write.WRITTEN;
    }

    private void writeAction(String name, Double meanS, Double stdS, Double minS, Double success, long sourceId) throws SQLException {
        try (PreparedStatement ps = c.prepareStatement(
                "UPDATE actions SET mean_time_s=?, std_time_s=?, min_time_s=?, success_prob=?, source_id=?, updated_utc=? WHERE name=?")) {
            setNullable(ps, 1, meanS);
            setNullable(ps, 2, stdS);
            setNullable(ps, 3, minS);
            setNullable(ps, 4, success);
            ps.setLong(5, sourceId);
            ps.setString(6, now());
            ps.setString(7, name);
            ps.executeUpdate();
        }
    }

    // ------------------------------------------------------------------ observations

    public static final class ShotObs {
        private final String recorded;
        private final String ball;
        private final String target;
        private final double distanceM;
        private final Double bearingDeg;
        private final Double robotSpeedMps;
        private final Double targetRpm;
        private final Double actualRpm;
        private final Double hoodDeg;
        private final Double turretErrorDeg;
        private final Double batteryV;
        private final Integer shotInBurst;
        private final String result;
        private final String notes;

        public ShotObs(String recorded, String ball, String target, double distanceM, Double bearingDeg, Double robotSpeedMps, Double targetRpm, Double actualRpm, Double hoodDeg, Double turretErrorDeg, Double batteryV, Integer shotInBurst, String result, String notes) {
            this.recorded = recorded;
            this.ball = ball;
            this.target = target;
            this.distanceM = distanceM;
            this.bearingDeg = bearingDeg;
            this.robotSpeedMps = robotSpeedMps;
            this.targetRpm = targetRpm;
            this.actualRpm = actualRpm;
            this.hoodDeg = hoodDeg;
            this.turretErrorDeg = turretErrorDeg;
            this.batteryV = batteryV;
            this.shotInBurst = shotInBurst;
            this.result = result;
            this.notes = notes;
        }

        public String recorded() { return recorded; }
        public String ball() { return ball; }
        public String target() { return target; }
        public double distanceM() { return distanceM; }
        public Double bearingDeg() { return bearingDeg; }
        public Double robotSpeedMps() { return robotSpeedMps; }
        public Double targetRpm() { return targetRpm; }
        public Double actualRpm() { return actualRpm; }
        public Double hoodDeg() { return hoodDeg; }
        public Double turretErrorDeg() { return turretErrorDeg; }
        public Double batteryV() { return batteryV; }
        public Integer shotInBurst() { return shotInBurst; }
        public String result() { return result; }
        public String notes() { return notes; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof ShotObs)) return false;
            ShotObs o = (ShotObs) other;
            return java.util.Objects.equals(recorded, o.recorded)
                    && java.util.Objects.equals(ball, o.ball)
                    && java.util.Objects.equals(target, o.target)
                    && Double.compare(distanceM, o.distanceM) == 0
                    && java.util.Objects.equals(bearingDeg, o.bearingDeg)
                    && java.util.Objects.equals(robotSpeedMps, o.robotSpeedMps)
                    && java.util.Objects.equals(targetRpm, o.targetRpm)
                    && java.util.Objects.equals(actualRpm, o.actualRpm)
                    && java.util.Objects.equals(hoodDeg, o.hoodDeg)
                    && java.util.Objects.equals(turretErrorDeg, o.turretErrorDeg)
                    && java.util.Objects.equals(batteryV, o.batteryV)
                    && java.util.Objects.equals(shotInBurst, o.shotInBurst)
                    && java.util.Objects.equals(result, o.result)
                    && java.util.Objects.equals(notes, o.notes);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(recorded, ball, target, distanceM, bearingDeg, robotSpeedMps, targetRpm, actualRpm, hoodDeg, turretErrorDeg, batteryV, shotInBurst, result, notes);
        }

        @Override
        public String toString() {
            return "ShotObs[recorded=" + recorded + ", ball=" + ball + ", target=" + target + ", distanceM=" + distanceM + ", bearingDeg=" + bearingDeg + ", robotSpeedMps=" + robotSpeedMps + ", targetRpm=" + targetRpm + ", actualRpm=" + actualRpm + ", hoodDeg=" + hoodDeg + ", turretErrorDeg=" + turretErrorDeg + ", batteryV=" + batteryV + ", shotInBurst=" + shotInBurst + ", result=" + result + ", notes=" + notes + "]";
        }

        public boolean made() { return "make".equals(result); }
    
    }

    public static final List<String> SHOT_RESULTS = Arrays.asList("make", "miss_short", "miss_long", "miss_left", "miss_right", "miss_other", "bounce_out");

    public void addShots(List<ShotObs> shots, long sourceId) throws SQLException {
        transaction(() -> {
            try (PreparedStatement ps = c.prepareStatement("INSERT INTO shot_observations(source_id, recorded, ball, target, distance_m, bearing_deg, "
                    + "robot_speed_mps, target_rpm, actual_rpm, hood_deg, turret_error_deg, battery_v, shot_in_burst, result, notes) "
                    + "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)")) {
                for (ShotObs s : shots) {
                    ps.setLong(1, sourceId);
                    ps.setString(2, s.recorded());
                    ps.setString(3, s.ball());
                    ps.setString(4, s.target() == null ? "cell" : s.target());
                    ps.setDouble(5, s.distanceM());
                    setNullable(ps, 6, s.bearingDeg());
                    setNullable(ps, 7, s.robotSpeedMps());
                    setNullable(ps, 8, s.targetRpm());
                    setNullable(ps, 9, s.actualRpm());
                    setNullable(ps, 10, s.hoodDeg());
                    setNullable(ps, 11, s.turretErrorDeg());
                    setNullable(ps, 12, s.batteryV());
                    if (s.shotInBurst() == null) ps.setNull(13, Types.INTEGER);
                    else ps.setInt(13, s.shotInBurst());
                    ps.setString(14, s.result());
                    ps.setString(15, s.notes());
                    ps.addBatch();
                }
                ps.executeBatch();
            }
            touch();
            return null;
        });
    }

    public List<ShotObs> shots(String ball, String target) throws SQLException {
        List<ShotObs> out = new ArrayList<>();
        try (PreparedStatement ps = c.prepareStatement("SELECT * FROM shot_observations WHERE ball = ? AND target = ? ORDER BY id")) {
            ps.setString(1, ball);
            ps.setString(2, target);
            try (ResultSet rs = ps.executeQuery()) {
                while (rs.next()) {
                    int burst = rs.getInt("shot_in_burst");
                    Integer b = rs.wasNull() ? null : burst;
                    out.add(new ShotObs(rs.getString("recorded"), rs.getString("ball"), rs.getString("target"), rs.getDouble("distance_m"),
                            getNullable(rs, "bearing_deg"), getNullable(rs, "robot_speed_mps"), getNullable(rs, "target_rpm"),
                            getNullable(rs, "actual_rpm"), getNullable(rs, "hood_deg"), getNullable(rs, "turret_error_deg"),
                            getNullable(rs, "battery_v"), b, rs.getString("result"), rs.getString("notes")));
                }
            }
        }
        return out;
    }

    public static final class PoseObs {
        private final String recorded;
        private final Double elapsedS;
        private final Double distanceDrivenM;
        private final double trueX;
        private final double trueY;
        private final Double trueHeadingDeg;
        private final double estX;
        private final double estY;
        private final Double estHeadingDeg;
        private final String notes;

        public PoseObs(String recorded, Double elapsedS, Double distanceDrivenM, double trueX, double trueY, Double trueHeadingDeg, double estX, double estY, Double estHeadingDeg, String notes) {
            this.recorded = recorded;
            this.elapsedS = elapsedS;
            this.distanceDrivenM = distanceDrivenM;
            this.trueX = trueX;
            this.trueY = trueY;
            this.trueHeadingDeg = trueHeadingDeg;
            this.estX = estX;
            this.estY = estY;
            this.estHeadingDeg = estHeadingDeg;
            this.notes = notes;
        }

        public String recorded() { return recorded; }
        public Double elapsedS() { return elapsedS; }
        public Double distanceDrivenM() { return distanceDrivenM; }
        public double trueX() { return trueX; }
        public double trueY() { return trueY; }
        public Double trueHeadingDeg() { return trueHeadingDeg; }
        public double estX() { return estX; }
        public double estY() { return estY; }
        public Double estHeadingDeg() { return estHeadingDeg; }
        public String notes() { return notes; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof PoseObs)) return false;
            PoseObs o = (PoseObs) other;
            return java.util.Objects.equals(recorded, o.recorded)
                    && java.util.Objects.equals(elapsedS, o.elapsedS)
                    && java.util.Objects.equals(distanceDrivenM, o.distanceDrivenM)
                    && Double.compare(trueX, o.trueX) == 0
                    && Double.compare(trueY, o.trueY) == 0
                    && java.util.Objects.equals(trueHeadingDeg, o.trueHeadingDeg)
                    && Double.compare(estX, o.estX) == 0
                    && Double.compare(estY, o.estY) == 0
                    && java.util.Objects.equals(estHeadingDeg, o.estHeadingDeg)
                    && java.util.Objects.equals(notes, o.notes);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(recorded, elapsedS, distanceDrivenM, trueX, trueY, trueHeadingDeg, estX, estY, estHeadingDeg, notes);
        }

        @Override
        public String toString() {
            return "PoseObs[recorded=" + recorded + ", elapsedS=" + elapsedS + ", distanceDrivenM=" + distanceDrivenM + ", trueX=" + trueX + ", trueY=" + trueY + ", trueHeadingDeg=" + trueHeadingDeg + ", estX=" + estX + ", estY=" + estY + ", estHeadingDeg=" + estHeadingDeg + ", notes=" + notes + "]";
        }

        public double positionError() { return Math.hypot(estX - trueX, estY - trueY); }
    
    }

    public void addPoses(List<PoseObs> poses, long sourceId) throws SQLException {
        transaction(() -> {
            try (PreparedStatement ps = c.prepareStatement("INSERT INTO pose_observations(source_id, recorded, elapsed_s, distance_driven_m, "
                    + "true_x_m, true_y_m, true_heading_deg, est_x_m, est_y_m, est_heading_deg, notes) VALUES (?,?,?,?,?,?,?,?,?,?,?)")) {
                for (PoseObs p : poses) {
                    ps.setLong(1, sourceId);
                    ps.setString(2, p.recorded());
                    setNullable(ps, 3, p.elapsedS());
                    setNullable(ps, 4, p.distanceDrivenM());
                    ps.setDouble(5, p.trueX());
                    ps.setDouble(6, p.trueY());
                    setNullable(ps, 7, p.trueHeadingDeg());
                    ps.setDouble(8, p.estX());
                    ps.setDouble(9, p.estY());
                    setNullable(ps, 10, p.estHeadingDeg());
                    ps.setString(11, p.notes());
                    ps.addBatch();
                }
                ps.executeBatch();
            }
            touch();
            return null;
        });
    }

    public List<PoseObs> poses() throws SQLException {
        List<PoseObs> out = new ArrayList<>();
        try (Statement s = c.createStatement(); ResultSet rs = s.executeQuery("SELECT * FROM pose_observations ORDER BY id")) {
            while (rs.next()) {
                out.add(new PoseObs(rs.getString("recorded"), getNullable(rs, "elapsed_s"), getNullable(rs, "distance_driven_m"),
                        rs.getDouble("true_x_m"), rs.getDouble("true_y_m"), getNullable(rs, "true_heading_deg"),
                        rs.getDouble("est_x_m"), rs.getDouble("est_y_m"), getNullable(rs, "est_heading_deg"), rs.getString("notes")));
            }
        }
        return out;
    }

    public int count(String table) throws SQLException {
        try (Statement s = c.createStatement(); ResultSet rs = s.executeQuery("SELECT COUNT(*) FROM " + table)) {
            return rs.next() ? rs.getInt(1) : 0;
        }
    }

    // ------------------------------------------------------------------ completeness

    /** One line of the completeness report. */
    public static final class Status {
        private final String type;
        private final String name;
        private final String category;
        private final String role;
        private final boolean required;
        private final boolean hasData;
        private final SourceKind sourceKind;

        public Status(String type, String name, String category, String role, boolean required, boolean hasData, SourceKind sourceKind) {
            this.type = type;
            this.name = name;
            this.category = category;
            this.role = role;
            this.required = required;
            this.hasData = hasData;
            this.sourceKind = sourceKind;
        }

        public String type() { return type; }
        public String name() { return name; }
        public String category() { return category; }
        public String role() { return role; }
        public boolean required() { return required; }
        public boolean hasData() { return hasData; }
        public SourceKind sourceKind() { return sourceKind; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Status)) return false;
            Status o = (Status) other;
            return java.util.Objects.equals(type, o.type)
                    && java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(category, o.category)
                    && java.util.Objects.equals(role, o.role)
                    && required == o.required
                    && hasData == o.hasData
                    && java.util.Objects.equals(sourceKind, o.sourceKind);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(type, name, category, role, required, hasData, sourceKind);
        }

        @Override
        public String toString() {
            return "Status[type=" + type + ", name=" + name + ", category=" + category + ", role=" + role + ", required=" + required + ", hasData=" + hasData + ", sourceKind=" + sourceKind + "]";
        }
    }

    public List<Status> everything() throws SQLException {
        List<Status> out = new ArrayList<>();
        try (Statement s = c.createStatement(); ResultSet rs = s.executeQuery("SELECT * FROM v_everything ORDER BY category, type, name")) {
            while (rs.next()) {
                String kind = rs.getString("source_kind");
                out.add(new Status(rs.getString("type"), rs.getString("name"), rs.getString("category"), rs.getString("role"),
                        rs.getInt("required") == 1, rs.getInt("has_data") == 1, kind == null ? null : SourceKind.parse(kind)));
            }
        }
        return out;
    }
}
