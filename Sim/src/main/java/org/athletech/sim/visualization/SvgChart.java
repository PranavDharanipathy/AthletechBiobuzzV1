package org.athletech.sim.visualization;

import org.athletech.sim.utility.Csv;

import java.util.ArrayList;
import java.util.List;

/** Line charts as inline SVG (no libraries, so reports open anywhere, offline). */
public final class SvgChart {
    public static final String[] PALETTE = {"#4e79a7", "#f28e2b", "#e15759", "#59a14f", "#b07aa1", "#edc948", "#76b7b2", "#ff9da7", "#9c755f", "#79706e"};

    /** One line; {@code band} (optional, same length) draws a +/- shaded area. */
    public static final class Series {
        private final String name;
        private final double[] x;
        private final double[] y;
        private final double[] band;

        public Series(String name, double[] x, double[] y, double[] band) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.band = band;
        }

        public String name() { return name; }
        public double[] x() { return x; }
        public double[] y() { return y; }
        public double[] band() { return band; }

        @Override
        public boolean equals(Object other) {
            if (this == other) return true;
            if (!(other instanceof Series)) return false;
            Series o = (Series) other;
            return java.util.Objects.equals(name, o.name)
                    && java.util.Objects.equals(x, o.x)
                    && java.util.Objects.equals(y, o.y)
                    && java.util.Objects.equals(band, o.band);
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(name, x, y, band);
        }

        @Override
        public String toString() {
            return "Series[name=" + name + ", x=" + x + ", y=" + y + ", band=" + band + "]";
        }

        public Series(String name, double[] x, double[] y) {
            this(name, x, y, null);
        }
    
    }

    private final String title, xLabel, yLabel;
    private final List<Series> series = new ArrayList<>();
    private boolean zeroBased;
    private Double yMinFixed, yMaxFixed;

    public SvgChart(String title, String xLabel, String yLabel) {
        this.title = title;
        this.xLabel = xLabel;
        this.yLabel = yLabel;
    }

    public SvgChart add(Series s) {
        if (s != null && s.x().length > 0) series.add(s);
        return this;
    }

    public SvgChart zeroBased() {
        zeroBased = true;
        return this;
    }

    public SvgChart yRange(double min, double max) {
        yMinFixed = min;
        yMaxFixed = max;
        return this;
    }

    public boolean isEmpty() {
        return series.isEmpty();
    }

    public String svg() {
        boolean legend = series.size() > 1 || (!series.isEmpty() && !series.get(0).name().isEmpty());
        int legendRows = legend ? (series.size() + 2) / 3 : 0;
        int w = 640, h = 300 + legendRows * 15, L = 62, R = 14, T = 30 + legendRows * 15, B = 44;
        int pw = w - L - R, ph = h - T - B;
        double xmin = Double.POSITIVE_INFINITY, xmax = Double.NEGATIVE_INFINITY, ymin = Double.POSITIVE_INFINITY, ymax = Double.NEGATIVE_INFINITY;
        for (Series s : series) {
            for (int i = 0; i < s.x().length; i++) {
                if (!Double.isFinite(s.x()[i]) || !Double.isFinite(s.y()[i])) continue;
                double b = s.band() != null && Double.isFinite(s.band()[i]) ? s.band()[i] : 0;
                xmin = Math.min(xmin, s.x()[i]);
                xmax = Math.max(xmax, s.x()[i]);
                ymin = Math.min(ymin, s.y()[i] - b);
                ymax = Math.max(ymax, s.y()[i] + b);
            }
        }
        StringBuilder sb = new StringBuilder();
        sb.append("<svg class=\"chart\" viewBox=\"0 0 ").append(w).append(' ').append(h).append("\" role=\"img\" aria-label=\"")
                .append(esc(title)).append("\">");
        sb.append("<text x=\"").append(L).append("\" y=\"18\" class=\"ct\">").append(esc(title)).append("</text>");
        if (!Double.isFinite(xmin)) {
            sb.append("<text x=\"").append(w / 2).append("\" y=\"").append(h / 2).append("\" class=\"cm\" text-anchor=\"middle\">no data</text></svg>");
            return sb.toString();
        }
        if (zeroBased) ymin = Math.min(0, ymin);
        if (yMinFixed != null) ymin = yMinFixed;
        if (yMaxFixed != null) ymax = yMaxFixed;
        if (ymax - ymin <= 1e-4 * Math.max(Math.abs(ymax), Math.abs(ymin))) { // flat data: show it as a flat line, not noise
            double d = Math.abs(ymax) > 1e-12 ? Math.abs(ymax) * 0.1 : 1;
            ymax += d;
            ymin -= d;
        }
        if (xmax == xmin) {
            xmax += 1;
            xmin -= 1;
        }
        if (yMinFixed == null && !(zeroBased && ymin == 0)) ymin -= (ymax - ymin) * 0.05;
        if (yMaxFixed == null) ymax += (ymax - ymin) * 0.05;
        final double x0 = xmin, x1 = xmax, y0 = ymin, y1 = ymax;
        java.util.function.DoubleUnaryOperator X = v -> L + (v - x0) / (x1 - x0) * pw;
        java.util.function.DoubleUnaryOperator Y = v -> T + (1 - (v - y0) / (y1 - y0)) * ph;

        for (double t : ticks(y0, y1, 5)) {
            double yy = Y.applyAsDouble(t);
            sb.append(String.format(java.util.Locale.ROOT, "<line x1=\"%d\" x2=\"%d\" y1=\"%.1f\" y2=\"%.1f\" class=\"cg\"/>", L, L + pw, yy, yy));
            sb.append(String.format(java.util.Locale.ROOT, "<text x=\"%d\" y=\"%.1f\" class=\"cl\" text-anchor=\"end\">%s</text>", L - 6, yy + 4, Csv.fmt(round(t))));
        }
        for (double t : ticks(x0, x1, 7)) {
            double xx = X.applyAsDouble(t);
            sb.append(String.format(java.util.Locale.ROOT, "<line x1=\"%.1f\" x2=\"%.1f\" y1=\"%d\" y2=\"%d\" class=\"cg\"/>", xx, xx, T, T + ph));
            sb.append(String.format(java.util.Locale.ROOT, "<text x=\"%.1f\" y=\"%d\" class=\"cl\" text-anchor=\"middle\">%s</text>", xx, T + ph + 16, Csv.fmt(round(t))));
        }
        sb.append(String.format(java.util.Locale.ROOT, "<line x1=\"%d\" x2=\"%d\" y1=\"%d\" y2=\"%d\" class=\"ca\"/>", L, L + pw, T + ph, T + ph));
        sb.append(String.format(java.util.Locale.ROOT, "<line x1=\"%d\" x2=\"%d\" y1=\"%d\" y2=\"%d\" class=\"ca\"/>", L, L, T, T + ph));
        sb.append(String.format(java.util.Locale.ROOT, "<text x=\"%d\" y=\"%d\" class=\"cl\" text-anchor=\"middle\">%s</text>", L + pw / 2, h - 6, esc(xLabel)));
        sb.append(String.format(java.util.Locale.ROOT, "<text transform=\"translate(14 %d) rotate(-90)\" class=\"cl\" text-anchor=\"middle\">%s</text>", T + ph / 2, esc(yLabel)));

        for (int si = 0; si < series.size(); si++) {
            Series s = series.get(si);
            String col = PALETTE[si % PALETTE.length];
            if (s.band() != null) {
                StringBuilder up = new StringBuilder(), down = new StringBuilder();
                for (int i = 0; i < s.x().length; i++) {
                    double b = Double.isFinite(s.band()[i]) ? s.band()[i] : 0;
                    up.append(String.format(java.util.Locale.ROOT, "%.1f,%.1f ", X.applyAsDouble(s.x()[i]), Y.applyAsDouble(s.y()[i] + b)));
                }
                for (int i = s.x().length - 1; i >= 0; i--) {
                    double b = Double.isFinite(s.band()[i]) ? s.band()[i] : 0;
                    down.append(String.format(java.util.Locale.ROOT, "%.1f,%.1f ", X.applyAsDouble(s.x()[i]), Y.applyAsDouble(s.y()[i] - b)));
                }
                sb.append("<polygon points=\"").append(up).append(down).append("\" fill=\"").append(col).append("\" opacity=\"0.18\"/>");
            }
            StringBuilder pts = new StringBuilder();
            for (int i = 0; i < s.x().length; i++) {
                if (!Double.isFinite(s.y()[i])) continue;
                pts.append(String.format(java.util.Locale.ROOT, "%.1f,%.1f ", X.applyAsDouble(s.x()[i]), Y.applyAsDouble(s.y()[i])));
            }
            sb.append("<polyline points=\"").append(pts).append("\" fill=\"none\" stroke=\"").append(col).append("\" stroke-width=\"2\"/>");
            if (s.x().length <= 30) {
                for (int i = 0; i < s.x().length; i++) {
                    if (!Double.isFinite(s.y()[i])) continue;
                    sb.append(String.format(java.util.Locale.ROOT, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"2.6\" fill=\"%s\"><title>%s: %s, %s</title></circle>",
                            X.applyAsDouble(s.x()[i]), Y.applyAsDouble(s.y()[i]), col, esc(s.name()), Csv.fmt(s.x()[i]), Csv.fmt(s.y()[i])));
                }
            }
        }
        if (legend) {
            int lx = L;
            int ly = 34;
            for (int si = 0; si < series.size(); si++) {
                String col = PALETTE[si % PALETTE.length];
                int col0 = si % 3, row = si / 3;
                int xx = lx + col0 * 180, yy = ly + row * 15;
                sb.append(String.format(java.util.Locale.ROOT, "<rect x=\"%d\" y=\"%d\" width=\"10\" height=\"3\" fill=\"%s\"/>", xx, yy - 4, col));
                sb.append(String.format(java.util.Locale.ROOT, "<text x=\"%d\" y=\"%d\" class=\"cl\">%s</text>", xx + 14, yy, esc(series.get(si).name())));
            }
        }
        sb.append("</svg>");
        return sb.toString();
    }

    private static double round(double v) {
        return Math.abs(v) < 1e-12 ? 0 : v;
    }

    static List<Double> ticks(double lo, double hi, int count) {
        List<Double> out = new ArrayList<>();
        double span = hi - lo;
        if (!(span > 0)) {
            out.add(lo);
            return out;
        }
        double raw = span / count, mag = Math.pow(10, Math.floor(Math.log10(raw))), f = raw / mag;
        double step = (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * mag;
        for (double v = Math.ceil(lo / step) * step; v <= hi + step * 1e-9; v += step) out.add(v);
        return out;
    }

    static String esc(String s) {
        return s == null ? "" : s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("\"", "&quot;");
    }
}
