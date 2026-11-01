package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * One recording segment: a single activity (passive, one controller run, or one open-loop run).
 *
 * <p>Rows live in one preallocated primitive array, so recording does no file I/O and no
 * per-loop allocation. A full buffer refuses rows instead of growing; the OpMode decides whether
 * that ends an active test. Unset cells are NaN.
 */
public final class SegmentBuffer {

    public final DiagnosticActivity activity;
    public final int sequence;
    public final int runId;
    private final List<String> metadata;
    private final double[] data;
    private final int capacity;
    private int rows;
    private boolean rowOpen;

    public SegmentBuffer(DiagnosticActivity activity, int sequence, int runId, int capacity, List<String> metadata) {
        if (capacity <= 0) throw new IllegalArgumentException("capacity");
        this.activity = activity;
        this.sequence = sequence;
        this.runId = runId;
        this.capacity = capacity;
        this.metadata = Collections.unmodifiableList(new ArrayList<>(metadata));
        this.data = new double[capacity * Col.COUNT];
    }

    /** Opens the next row (all NaN). @return false if the buffer is full. */
    public boolean beginRow() {
        if (rows >= capacity) return false;
        Arrays.fill(data, rows * Col.COUNT, (rows + 1) * Col.COUNT, Double.NaN);
        rowOpen = true;
        return true;
    }

    public void set(Col col, double value) {
        if (rowOpen) data[rows * Col.COUNT + col.ordinal()] = value;
    }

    public void set(Col col, boolean value) {
        set(col, value ? 1 : 0);
    }

    public void commitRow() {
        if (rowOpen) {
            rows++;
            rowOpen = false;
        }
    }

    public double get(int row, Col col) {
        if (row < 0 || row >= rows) throw new IndexOutOfBoundsException("row " + row);
        return data[row * Col.COUNT + col.ordinal()];
    }

    public int rows() { return rows; }
    public int capacity() { return capacity; }
    public boolean isFull() { return rows >= capacity; }
    public List<String> metadata() { return metadata; }

    /** Writes "# key=value" metadata lines, the header row, then every row. */
    public void writeCsv(Writer out) throws IOException {
        out.write("# activity=" + activity.label + "\n");
        out.write("# segment=" + sequence + "\n");
        out.write("# run_id=" + runId + "\n");
        out.write("# rows=" + rows + "\n");
        for (String line : metadata) out.write("# " + line + "\n");
        Col[] cols = Col.values();
        for (int c = 0; c < cols.length; c++) {
            if (c > 0) out.write(',');
            out.write(cols[c].header);
        }
        out.write('\n');
        StringBuilder sb = new StringBuilder(512);
        for (int r = 0; r < rows; r++) {
            sb.setLength(0);
            for (int c = 0; c < cols.length; c++) {
                if (c > 0) sb.append(',');
                appendValue(sb, data[r * Col.COUNT + c]);
            }
            sb.append('\n');
            out.write(sb.toString());
        }
    }

    static void appendValue(StringBuilder sb, double v) {
        if (Double.isNaN(v)) return; // empty cell
        if (v == Math.rint(v) && Math.abs(v) < 1e15) sb.append((long) v);
        else sb.append(v);
    }
}
