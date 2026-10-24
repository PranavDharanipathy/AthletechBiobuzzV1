package org.athletech.sim.config;

import java.util.Arrays;
import java.util.List;

/**
 * SQLite schema of the robot profile (data/robot_profile.db). Any program that can open SQLite
 * reads it; PROFILE.md documents it for game-sim authors. Bump {@link #VERSION} and add a
 * migration in {@link ProfileDb} when a table changes shape.
 */
public final class ProfileSchema {
    private ProfileSchema() {
    }

    /** 2: adds disconnect_observations. */
    public static final int VERSION = 2;
    public static final String FORMAT = "athletech-robot-profile";

    private static final String KINDS = "'measured','game_manual','manual','sim','placeholder'";
    private static final String ROLES = "'input','derived','measured'";

    /** Columns shared by every catalog-style table. */
    private static final String COMMON = String.format("  role TEXT NOT NULL CHECK (role IN (%s)),\n" +
            "  required INTEGER NOT NULL DEFAULT 0,\n" +
            "  description TEXT NOT NULL DEFAULT '',\n" +
            "  in_catalog INTEGER NOT NULL DEFAULT 0,\n" +
            "  source_id INTEGER REFERENCES sources(id),\n" +
            "  updated_utc TEXT\n", ROLES);

    public static final List<String> DDL = Arrays.asList(
            "CREATE TABLE IF NOT EXISTS meta (\n" +
                    "  key TEXT PRIMARY KEY,\n" +
                    "  value TEXT\n" +
                    ")",
            String.format("CREATE TABLE IF NOT EXISTS sources (\n" +
                    "  id INTEGER PRIMARY KEY AUTOINCREMENT,\n" +
                    "  kind TEXT NOT NULL CHECK (kind IN (%s)),\n" +
                    "  name TEXT NOT NULL,\n" +
                    "  detail TEXT,\n" +
                    "  created_utc TEXT NOT NULL\n" +
                    ")", KINDS),
            String.format("CREATE TABLE IF NOT EXISTS parameters (\n" +
                    "  key TEXT PRIMARY KEY,\n" +
                    "  category TEXT NOT NULL,\n" +
                    "  value REAL,\n" +
                    "  text_value TEXT,\n" +
                    "  unit TEXT NOT NULL DEFAULT '',\n" +
                    "  std_dev REAL,\n" +
                    "  min_valid REAL,\n" +
                    "  max_valid REAL,\n" +
                    "%s)", COMMON),
            String.format("CREATE TABLE IF NOT EXISTS curves (\n" +
                    "  name TEXT PRIMARY KEY,\n" +
                    "  category TEXT NOT NULL,\n" +
                    "  x_name TEXT NOT NULL, x_unit TEXT NOT NULL,\n" +
                    "  y_name TEXT NOT NULL, y_unit TEXT NOT NULL,\n" +
                    "  interpolation TEXT NOT NULL DEFAULT 'linear',\n" +
                    "%s)", COMMON),
            "CREATE TABLE IF NOT EXISTS curve_points (\n" +
                    "  curve TEXT NOT NULL REFERENCES curves(name) ON DELETE CASCADE ON UPDATE CASCADE,\n" +
                    "  x REAL NOT NULL,\n" +
                    "  y REAL NOT NULL,\n" +
                    "  y_std REAL,\n" +
                    "  n INTEGER,\n" +
                    "  PRIMARY KEY (curve, x)\n" +
                    ")",
            String.format("CREATE TABLE IF NOT EXISTS surfaces (\n" +
                    "  name TEXT PRIMARY KEY,\n" +
                    "  category TEXT NOT NULL,\n" +
                    "  x_name TEXT NOT NULL, x_unit TEXT NOT NULL,\n" +
                    "  y_name TEXT NOT NULL, y_unit TEXT NOT NULL,\n" +
                    "  z_name TEXT NOT NULL, z_unit TEXT NOT NULL,\n" +
                    "%s)", COMMON),
            "CREATE TABLE IF NOT EXISTS surface_points (\n" +
                    "  surface TEXT NOT NULL REFERENCES surfaces(name) ON DELETE CASCADE ON UPDATE CASCADE,\n" +
                    "  x REAL NOT NULL,\n" +
                    "  y REAL NOT NULL,\n" +
                    "  z REAL NOT NULL,\n" +
                    "  z_std REAL,\n" +
                    "  n INTEGER,\n" +
                    "  PRIMARY KEY (surface, x, y)\n" +
                    ")",
            String.format("CREATE TABLE IF NOT EXISTS distributions (\n" +
                    "  name TEXT PRIMARY KEY,\n" +
                    "  category TEXT NOT NULL,\n" +
                    "  kind TEXT CHECK (kind IS NULL OR kind IN ('constant','normal','uniform','triangular','lognormal','empirical')),\n" +
                    "  unit TEXT NOT NULL DEFAULT '',\n" +
                    "  per TEXT,\n" +
                    "  mean REAL, std_dev REAL, min REAL, max REAL, mode REAL,\n" +
                    "%s)", COMMON),
            "CREATE TABLE IF NOT EXISTS distribution_samples (\n" +
                    "  distribution TEXT NOT NULL REFERENCES distributions(name) ON DELETE CASCADE ON UPDATE CASCADE,\n" +
                    "  value REAL NOT NULL\n" +
                    ")",
            "CREATE INDEX IF NOT EXISTS distribution_samples_by_name ON distribution_samples(distribution)",
            String.format("CREATE TABLE IF NOT EXISTS actions (\n" +
                    "  name TEXT PRIMARY KEY,\n" +
                    "  category TEXT NOT NULL,\n" +
                    "  mean_time_s REAL,\n" +
                    "  std_time_s REAL,\n" +
                    "  min_time_s REAL,\n" +
                    "  success_prob REAL CHECK (success_prob IS NULL OR (success_prob >= 0 AND success_prob <= 1)),\n" +
                    "%s)", COMMON),
            "CREATE TABLE IF NOT EXISTS shot_observations (\n" +
                    "  id INTEGER PRIMARY KEY AUTOINCREMENT,\n" +
                    "  source_id INTEGER REFERENCES sources(id),\n" +
                    "  recorded TEXT,\n" +
                    "  ball TEXT NOT NULL,\n" +
                    "  target TEXT NOT NULL DEFAULT 'cell',\n" +
                    "  distance_m REAL NOT NULL,\n" +
                    "  bearing_deg REAL,\n" +
                    "  robot_speed_mps REAL,\n" +
                    "  target_rpm REAL,\n" +
                    "  actual_rpm REAL,\n" +
                    "  hood_deg REAL,\n" +
                    "  turret_error_deg REAL,\n" +
                    "  battery_v REAL,\n" +
                    "  shot_in_burst INTEGER,\n" +
                    "  result TEXT NOT NULL CHECK (result IN ('make','miss_short','miss_long','miss_left','miss_right','miss_other','bounce_out')),\n" +
                    "  notes TEXT\n" +
                    ")",
            "CREATE TABLE IF NOT EXISTS pose_observations (\n" +
                    "  id INTEGER PRIMARY KEY AUTOINCREMENT,\n" +
                    "  source_id INTEGER REFERENCES sources(id),\n" +
                    "  recorded TEXT,\n" +
                    "  elapsed_s REAL,\n" +
                    "  distance_driven_m REAL,\n" +
                    "  true_x_m REAL NOT NULL, true_y_m REAL NOT NULL, true_heading_deg REAL,\n" +
                    "  est_x_m REAL NOT NULL, est_y_m REAL NOT NULL, est_heading_deg REAL,\n" +
                    "  notes TEXT\n" +
                    ")",
            "CREATE TABLE IF NOT EXISTS disconnect_observations (\n" +
                    "  id INTEGER PRIMARY KEY AUTOINCREMENT,\n" +
                    "  source_id INTEGER REFERENCES sources(id),\n" +
                    "  recorded TEXT,\n" +
                    "  match TEXT NOT NULL,\n" +
                    "  match_time_s REAL,\n" +
                    "  type TEXT NOT NULL,\n" +
                    "  outage_s REAL,\n" +
                    "  humidity_pct REAL,\n" +
                    "  cause TEXT,\n" +
                    "  notes TEXT\n" +
                    ")",
            // ------------------------------------------------------------ convenience views
            "CREATE VIEW IF NOT EXISTS v_parameters AS\n" +
                    "  SELECT p.key, p.category, p.value, p.text_value, p.unit, p.std_dev, p.role, p.required,\n" +
                    "         s.kind AS source_kind, s.name AS source_name, p.updated_utc, p.description\n" +
                    "  FROM parameters p LEFT JOIN sources s ON s.id = p.source_id",
            "CREATE VIEW IF NOT EXISTS v_everything AS\n" +
                    "  SELECT 'parameter' AS type, p.key AS name, p.category, p.role, p.required,\n" +
                    "         (p.value IS NOT NULL OR p.text_value IS NOT NULL) AS has_data, s.kind AS source_kind, p.unit AS unit\n" +
                    "    FROM parameters p LEFT JOIN sources s ON s.id = p.source_id\n" +
                    "  UNION ALL\n" +
                    "  SELECT 'curve', c.name, c.category, c.role, c.required,\n" +
                    "         EXISTS (SELECT 1 FROM curve_points q WHERE q.curve = c.name), s.kind, c.x_unit || ' -> ' || c.y_unit\n" +
                    "    FROM curves c LEFT JOIN sources s ON s.id = c.source_id\n" +
                    "  UNION ALL\n" +
                    "  SELECT 'surface', f.name, f.category, f.role, f.required,\n" +
                    "         EXISTS (SELECT 1 FROM surface_points q WHERE q.surface = f.name), s.kind, f.x_unit || ', ' || f.y_unit || ' -> ' || f.z_unit\n" +
                    "    FROM surfaces f LEFT JOIN sources s ON s.id = f.source_id\n" +
                    "  UNION ALL\n" +
                    "  SELECT 'distribution', d.name, d.category, d.role, d.required, d.kind IS NOT NULL, s.kind, d.unit\n" +
                    "    FROM distributions d LEFT JOIN sources s ON s.id = d.source_id\n" +
                    "  UNION ALL\n" +
                    "  SELECT 'action', a.name, a.category, a.role, a.required, a.mean_time_s IS NOT NULL, s.kind, 's'\n" +
                    "    FROM actions a LEFT JOIN sources s ON s.id = a.source_id",
            "CREATE VIEW IF NOT EXISTS v_missing AS SELECT * FROM v_everything WHERE required = 1 AND has_data = 0",
            "CREATE VIEW IF NOT EXISTS v_placeholders AS SELECT * FROM v_everything WHERE has_data = 1 AND source_kind = 'placeholder'");
}
