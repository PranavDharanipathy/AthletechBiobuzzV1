package org.athletech.sim.telemetry;

import org.athletech.sim.utility.Csv;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Collects a time series during a sim run (one row per sample) and writes it as CSV. */
public final class Recorder {
    private final List<String> columns;
    private final List<double[]> rows = new ArrayList<>();
    private final double period;
    private double nextT = Double.NEGATIVE_INFINITY;

    /** @param periodS minimum time between recorded rows (0 = record every call) */
    public Recorder(double periodS, String... columns) {
        this.columns = Arrays.asList(columns);
        this.period = periodS;
    }

    /** Records {@code values} (first value must be time) if at least one period has passed. */
    public void sample(double... values) {
        if (values.length != columns.size()) throw new IllegalArgumentException("expected " + columns.size() + " values");
        if (values[0] < nextT) return;
        rows.add(values.clone());
        nextT = values[0] + period;
    }

    public double[] column(String name) {
        int i = columns.indexOf(name);
        if (i < 0) throw new IllegalArgumentException("no column " + name);
        return rows.stream().mapToDouble(r -> r[i]).toArray();
    }

    public int size() {
        return rows.size();
    }

    public void write(Path file) throws IOException {
        List<Object[]> out = new ArrayList<>();
        for (double[] r : rows) out.add(Arrays.stream(r).boxed().toArray());
        Csv.write(file, columns, out);
    }
}
