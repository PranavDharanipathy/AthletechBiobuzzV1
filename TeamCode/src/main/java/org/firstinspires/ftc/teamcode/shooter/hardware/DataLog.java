package org.firstinspires.ftc.teamcode.shooter.hardware;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Preallocated column log. Recording is allocation-free (no GC pauses that would distort timing data);
 * the CSV is written once at the end to /sdcard/FIRST/shooter_logs/.
 */
public final class DataLog {

    private final String[] columns;
    private final double[][] data;
    private int rows;

    public DataLog(int capacity, String... columns) {
        this.columns = columns;
        this.data = new double[columns.length][capacity];
    }

    /** Sets a value in the row being recorded. */
    public void set(int col, double v) {
        if (rows < data[0].length) data[col][rows] = v;
    }

    /** Finishes the current row. Returns false when full. */
    public boolean commit() {
        if (rows >= data[0].length) return false;
        rows++;
        return true;
    }

    public int rows() { return rows; }
    public double[] column(int c) { return data[c]; }

    /** Writes the CSV; returns the file path (or an error message). */
    public String save(String name) {
        File dir = new File(AppUtil.FIRST_FOLDER, "shooter_logs");
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();
        File f = new File(dir, name + "_" + System.currentTimeMillis() + ".csv");
        try (BufferedWriter w = new BufferedWriter(new FileWriter(f))) {
            StringBuilder sb = new StringBuilder();
            for (int c = 0; c < columns.length; c++) {  // (String.join needs API 26; the Control Hub is API 25)
                if (c > 0) sb.append(',');
                sb.append(columns[c]);
            }
            w.write(sb.toString());
            w.newLine();
            for (int r = 0; r < rows; r++) {
                sb.setLength(0);
                for (int c = 0; c < columns.length; c++) {
                    if (c > 0) sb.append(',');
                    sb.append(data[c][r]);
                }
                w.write(sb.toString());
                w.newLine();
            }
            return f.getAbsolutePath();
        } catch (IOException e) {
            return "log save failed: " + e.getMessage();
        }
    }
}
