package org.athletech.sim.visualization;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/** A self-contained HTML report page: headings, notes, tables and SVG charts. */
public final class HtmlReport {
    private final String title;
    private final StringBuilder body = new StringBuilder();
    private boolean inGrid;

    public HtmlReport(String title) {
        this.title = title;
    }

    public HtmlReport heading(String text) {
        closeGrid();
        body.append("<h2>").append(SvgChart.esc(text)).append("</h2>\n");
        return this;
    }

    public HtmlReport paragraph(String text) {
        closeGrid();
        body.append("<p>").append(SvgChart.esc(text)).append("</p>\n");
        return this;
    }

    /** A highlighted box; {@code level} is "info", "warn" or "bad". */
    public HtmlReport note(String level, String text) {
        closeGrid();
        body.append("<div class=\"note ").append(level).append("\">").append(SvgChart.esc(text).replace("\n", "<br>")).append("</div>\n");
        return this;
    }

    public HtmlReport bullets(List<String> items) {
        if (items.isEmpty()) return this;
        closeGrid();
        body.append("<ul>");
        for (String s : items) body.append("<li>").append(SvgChart.esc(s)).append("</li>");
        body.append("</ul>\n");
        return this;
    }

    public HtmlReport table(List<String> header, List<List<String>> rows) {
        closeGrid();
        body.append("<div class=\"tw\"><table><thead><tr>");
        for (String h : header) body.append("<th>").append(SvgChart.esc(h)).append("</th>");
        body.append("</tr></thead><tbody>");
        for (List<String> r : rows) {
            body.append("<tr>");
            for (String c : r) {
                boolean num = c != null && c.matches("-?[0-9.,eE+-]+( .*)?") && !c.isEmpty();
                body.append(num ? "<td class=\"n\">" : "<td>").append(SvgChart.esc(c)).append("</td>");
            }
            body.append("</tr>");
        }
        body.append("</tbody></table></div>\n");
        return this;
    }

    /** Charts added in a row are laid out in a responsive grid. */
    public HtmlReport chart(SvgChart chart) {
        if (!inGrid) {
            body.append("<div class=\"grid\">");
            inGrid = true;
        }
        body.append("<div class=\"card\">").append(chart.svg()).append("</div>");
        return this;
    }

    private void closeGrid() {
        if (inGrid) {
            body.append("</div>\n");
            inGrid = false;
        }
    }

    public String html() {
        closeGrid();
        return String.format("<!doctype html>\n" +
                "<html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n" +
                "<title>%s</title>\n" +
                "<style>\n" +
                ":root{--bg:#f4f6f9;--card:#fff;--text:#1b2230;--muted:#667085;--line:#e3e7ee;--grid:#eef1f5;\n" +
                "  --info:#e8f0fe;--infob:#2f6fed;--warn:#fdf3e2;--warnb:#b45309;--bad:#fdecec;--badb:#dc2626}\n" +
                "@media (prefers-color-scheme: dark){:root{--bg:#0d1117;--card:#151b23;--text:#e6ebf2;--muted:#8c96a8;--line:#273141;--grid:#1f2733;\n" +
                "  --info:#16233c;--infob:#5b8dff;--warn:#2d230e;--warnb:#e3a008;--bad:#331617;--badb:#f85149}}\n" +
                "*{box-sizing:border-box}\n" +
                "body{margin:0;padding:20px 24px 48px;background:var(--bg);color:var(--text);font:14px/1.45 system-ui,-apple-system,\"Segoe UI\",Roboto,sans-serif}\n" +
                "h1{font-size:22px;margin:0 0 4px} h2{font-size:16px;margin:26px 0 10px}\n" +
                "p{max-width:900px;color:var(--muted)}\n" +
                ".note{max-width:1000px;padding:8px 12px;border-left:4px solid var(--infob);background:var(--info);border-radius:6px;margin:8px 0}\n" +
                ".note.warn{border-color:var(--warnb);background:var(--warn)} .note.bad{border-color:var(--badb);background:var(--bad)}\n" +
                ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(420px,1fr));gap:14px}\n" +
                ".card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:8px}\n" +
                "svg.chart{width:100%%;height:auto;display:block}\n" +
                ".ct{font-weight:600;font-size:13px;fill:var(--text)} .cl{font-size:11px;fill:var(--muted)} .cm{font-size:13px;fill:var(--muted)}\n" +
                ".cg{stroke:var(--grid)} .ca{stroke:var(--muted)}\n" +
                ".tw{overflow:auto;max-width:100%%;background:var(--card);border:1px solid var(--line);border-radius:10px}\n" +
                "table{border-collapse:collapse;font-size:12.5px;font-variant-numeric:tabular-nums;width:100%%}\n" +
                "th,td{padding:4px 10px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}\n" +
                "th{background:var(--grid);position:sticky;top:0} td.n{text-align:right}\n" +
                "ul{color:var(--muted)}\n" +
                "@media (max-width:600px){body{padding:12px 16px}.grid{grid-template-columns:1fr}}\n" +
                "</style></head><body>\n" +
                "<h1>%s</h1>\n" +
                "%s\n" +
                "</body></html>\n", SvgChart.esc(title), SvgChart.esc(title), body);
    }

    public void write(Path file) throws IOException {
        Files.createDirectories(file.toAbsolutePath().getParent());
        Files.write(file, html().getBytes(StandardCharsets.UTF_8));
    }
}
