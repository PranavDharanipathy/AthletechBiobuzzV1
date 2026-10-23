package org.athletech.sim.utility;

import java.io.IOException;
import java.io.Writer;
import java.math.BigDecimal;
import java.math.MathContext;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/** Minimal CSV reading (quoted fields supported) and writing. */
public final class Csv {
    private Csv() {
    }

    /** A parsed file: header names plus rows of raw strings. Lines starting with '#' are skipped. */
    public static final class Table {
        public final List<String> header;
        public final List<String[]> rows;
        private final Map<String, Integer> index = new HashMap<>();

        Table(List<String> header, List<String[]> rows) {
            this.header = header;
            this.rows = rows;
            for (int i = 0; i < header.size(); i++) index.put(header.get(i).trim().toLowerCase(Locale.ROOT), i);
        }

        public boolean has(String column) {
            return index.containsKey(column.toLowerCase(Locale.ROOT));
        }

        public String str(String[] row, String column) {
            Integer i = index.get(column.toLowerCase(Locale.ROOT));
            if (i == null || i >= row.length) return "";
            return row[i].trim();
        }

        /** The column as a number, or NaN when blank / missing / not a number. */
        public double num(String[] row, String column) {
            String s = str(row, column);
            if (s.isEmpty()) return Double.NaN;
            try {
                return Double.parseDouble(s);
            } catch (NumberFormatException e) {
                return Double.NaN;
            }
        }

        public double[] column(String column) {
            double[] out = new double[rows.size()];
            for (int r = 0; r < rows.size(); r++) out[r] = num(rows.get(r), column);
            return out;
        }
    }

    public static Table read(Path file) throws IOException {
        List<String> lines = Files.readAllLines(file, StandardCharsets.UTF_8);
        List<String> header = null;
        List<String[]> rows = new ArrayList<>();
        for (String line : lines) {
            if (line.trim().isEmpty() || line.startsWith("#")) continue;
            if (header == null && line.startsWith("﻿")) line = line.substring(1); // Excel BOM
            List<String> cells = parseLine(line);
            if (header == null) header = cells;
            else rows.add(cells.toArray(new String[0]));
        }
        return new Table(header == null ? Collections.emptyList() : header, rows);
    }

    public static List<String> parseLine(String line) {
        List<String> out = new ArrayList<>();
        StringBuilder cur = new StringBuilder();
        boolean quoted = false;
        for (int i = 0; i < line.length(); i++) {
            char c = line.charAt(i);
            if (quoted) {
                if (c == '"') {
                    if (i + 1 < line.length() && line.charAt(i + 1) == '"') {
                        cur.append('"');
                        i++;
                    } else {
                        quoted = false;
                    }
                } else {
                    cur.append(c);
                }
            } else if (c == '"') {
                quoted = true;
            } else if (c == ',') {
                out.add(cur.toString());
                cur.setLength(0);
            } else {
                cur.append(c);
            }
        }
        out.add(cur.toString());
        return out;
    }

    public static String cell(Object v) {
        if (v == null) return "";
        if (v instanceof Double) return fmt((Double) v);
        String s = v.toString();
        if (s.contains(",") || s.contains("\"") || s.contains("\n")) return '"' + s.replace("\"", "\"\"") + '"';
        return s;
    }

    /** Up to 6 significant digits, no trailing zeros, no exponent for everyday magnitudes. */
    public static String fmt(double d) {
        if (Double.isNaN(d) || Double.isInfinite(d)) return "";
        if (d == Math.rint(d) && Math.abs(d) < 1e15) return Long.toString((long) d);
        double a = Math.abs(d);
        if (a >= 1e-6 && a < 1e12) {
            return new BigDecimal(d).round(new MathContext(6)).stripTrailingZeros().toPlainString();
        }
        return String.format(Locale.ROOT, "%.5e", d);
    }

    public static void write(Path file, List<String> header, List<Object[]> rows) throws IOException {
        Files.createDirectories(file.toAbsolutePath().getParent());
        try (Writer w = Files.newBufferedWriter(file, StandardCharsets.UTF_8)) {
            w.write(String.join(",", header));
            w.write('\n');
            for (Object[] r : rows) {
                for (int i = 0; i < r.length; i++) {
                    if (i > 0) w.write(',');
                    w.write(cell(r[i]));
                }
                w.write('\n');
            }
        }
    }
}
