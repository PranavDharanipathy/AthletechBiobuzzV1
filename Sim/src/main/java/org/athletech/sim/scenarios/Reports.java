package org.athletech.sim.scenarios;

import org.athletech.sim.config.RobotParams;
import org.athletech.sim.utility.Csv;
import org.athletech.sim.visualization.HtmlReport;
import org.athletech.sim.visualization.SvgChart;

import java.util.ArrayList;
import java.util.List;

/** Shared bits of the scenario reports. */
final class Reports {
    private Reports() {
    }

    /** Warnings about placeholder inputs and assumptions, at the top of every report. */
    static void provenance(HtmlReport r, RobotParams rp, SimContext ctx) {
        if (!rp.placeholderInputs().isEmpty()) {
            r.note("bad", "These results use PLACEHOLDER inputs (example numbers, not your robot): "
                    + String.join(", ", rp.placeholderInputs())
                    + ". Saved results are marked 'placeholder' until you set real values and rerun.");
        }
        if (!rp.assumptions().isEmpty()) {
            r.note("warn", "Assumptions:\n" + String.join("\n", rp.assumptions()));
        }
        r.note("info", "Command: " + ctx.command + "   |   seed " + ctx.seed + ", " + ctx.trials + " Monte Carlo trials"
                + (ctx.save ? "   |   results saved to " + ctx.db.path() : "   |   not saved (add --save)"));
    }

    static SvgChart.Series series(String name, List<double[]> pts) {
        return series(name, pts, false);
    }

    /** Series from {x, y, yStd?} points; {@code band} shades +/- 1 std. */
    static SvgChart.Series series(String name, List<double[]> pts, boolean band) {
        double[] x = new double[pts.size()], y = new double[pts.size()], s = new double[pts.size()];
        for (int i = 0; i < pts.size(); i++) {
            x[i] = pts.get(i)[0];
            y[i] = pts.get(i)[1];
            s[i] = pts.get(i).length > 2 ? pts.get(i)[2] : Double.NaN;
        }
        return new SvgChart.Series(name, x, y, band ? s : null);
    }

    static List<String> row(Object... cells) {
        List<String> out = new ArrayList<>();
        for (Object c : cells) out.add(c instanceof Double ? Csv.fmt(round((Double) c)) : String.valueOf(c));
        return out;
    }

    /** Rounds for display: 4 significant digits. */
    static double round(double v) {
        if (!Double.isFinite(v) || v == 0) return v;
        double mag = Math.pow(10, 3 - Math.floor(Math.log10(Math.abs(v))));
        return Math.round(v * mag) / mag;
    }
}
