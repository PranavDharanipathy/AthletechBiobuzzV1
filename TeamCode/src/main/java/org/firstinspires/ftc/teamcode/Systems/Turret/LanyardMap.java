package org.firstinspires.ftc.teamcode.Systems.Turret;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

//the volts the lanyards take at each turret angle, learned from what the feedback had to add.
//Lanyards that overlap just add up in the same bins, so it doesn't care how many there are or where they start.
//One table per direction: a lanyard drags differently going out than coming back
public class LanyardMap {

    public final double minDegrees, maxDegrees, binDegrees;
    private final double[] forward, backward;
    private int lastIndex;
    private double lastFraction;

    public LanyardMap(double minDegrees, double maxDegrees, double binDegrees) {

        this.binDegrees = Math.max(binDegrees, 0.5);
        this.minDegrees = Math.min(minDegrees, maxDegrees);
        this.maxDegrees = Math.max(minDegrees, maxDegrees);
        int n = Math.max(2, (int) Math.ceil((this.maxDegrees - this.minDegrees) / this.binDegrees) + 1);
        forward = new double[n];
        backward = new double[n];
    }

    public int size() {
        return forward.length;
    }

    public double binDegrees(int k) {
        return minDegrees + k * binDegrees;
    }

    public double forwardVolts(int k) {
        return forward[k];
    }

    public double backwardVolts(int k) {
        return backward[k];
    }

    private boolean locate(double degrees) {

        if (!(degrees == degrees)) return false;
        double x = (Math.max(minDegrees, Math.min(maxDegrees, degrees)) - minDegrees) / binDegrees;
        int k = (int) Math.floor(x);
        if (k >= forward.length - 1) k = forward.length - 2;
        lastIndex = k;
        lastFraction = Math.max(0, Math.min(1, x - k));
        return true;
    }

    /// direction > 0 turning up, < 0 turning down, 0 holding (both tables averaged)
    public double volts(double degrees, double direction) {

        if (!locate(degrees)) return 0;
        int k = lastIndex;
        double s = lastFraction;
        double up = forward[k] + s * (forward[k + 1] - forward[k]);
        double down = backward[k] + s * (backward[k + 1] - backward[k]);
        if (direction > 0) return up;
        if (direction < 0) return down;
        return 0.5 * (up + down);
    }

    /// moves amount volts into the two bins around degrees (split the same way volts() reads them back), direction 0 = both tables
    public void learn(double degrees, double direction, double amount, double maxVolts) {

        if (!(amount == amount) || degrees < minDegrees || degrees > maxDegrees || !locate(degrees)) return;
        if (direction >= 0) add(forward, amount, maxVolts);
        if (direction <= 0) add(backward, amount, maxVolts);
    }

    private void add(double[] t, double amount, double maxVolts) {

        int k = lastIndex;
        double s = lastFraction;
        t[k] = clamp(t[k] + (1 - s) * amount, maxVolts);
        t[k + 1] = clamp(t[k + 1] + s * amount, maxVolts);
    }

    public void clear() {

        java.util.Arrays.fill(forward, 0);
        java.util.Arrays.fill(backward, 0);
    }

    public double largestVolts() {

        double m = 0;
        for (int k = 0; k < forward.length; k++) m = Math.max(m, Math.max(Math.abs(forward[k]), Math.abs(backward[k])));
        return m;
    }

    public String toCsv() {

        StringBuilder b = new StringBuilder("degrees,forward_v,backward_v\n");
        for (int k = 0; k < forward.length; k++) b.append(String.format(Locale.US, "%.2f,%.4f,%.4f%n", binDegrees(k), forward[k], backward[k]));
        return b.toString();
    }

    /// reads a map saved with any bin size, resampled onto these bins; false (and left alone) if it's junk
    public boolean fromCsv(String text) {

        List<double[]> rows = new ArrayList<>();
        for (String line : text.split("\r?\n")) {
            String[] p = line.trim().split(",");
            if (p.length < 3) continue;
            try {
                double[] r = {Double.parseDouble(p[0]), Double.parseDouble(p[1]), Double.parseDouble(p[2])};
                if (r[0] == r[0] && r[1] == r[1] && r[2] == r[2] && !Double.isInfinite(r[1]) && !Double.isInfinite(r[2])) rows.add(r);
            }
            catch (NumberFormatException e) {
                //header
            }
        }
        if (rows.isEmpty()) return false;
        java.util.Collections.sort(rows, new java.util.Comparator<double[]>() {
            public int compare(double[] a, double[] b) {
                return Double.compare(a[0], b[0]);
            }
        });
        for (int k = 0; k < forward.length; k++) {
            double d = binDegrees(k);
            forward[k] = sample(rows, d, 1);
            backward[k] = sample(rows, d, 2);
        }
        return true;
    }

    private static double sample(List<double[]> rows, double d, int column) {

        if (d <= rows.get(0)[0]) return rows.get(0)[column];
        for (int k = 1; k < rows.size(); k++) {
            double[] a = rows.get(k - 1), b = rows.get(k);
            if (d <= b[0]) {
                double span = b[0] - a[0];
                return span > 0 ? a[column] + (d - a[0]) / span * (b[column] - a[column]) : b[column];
            }
        }
        return rows.get(rows.size() - 1)[column];
    }

    public boolean load(File file) {

        if (file == null || !file.isFile()) return false;
        InputStream in = null;
        try {
            in = new FileInputStream(file);
            byte[] bytes = new byte[(int) Math.min(file.length(), 1 << 20)];
            int read = 0;
            while (read < bytes.length) {
                int n = in.read(bytes, read, bytes.length - read);
                if (n < 0) break;
                read += n;
            }
            return fromCsv(new String(bytes, 0, read, Charset.forName("UTF-8")));
        }
        catch (IOException e) {
            return false;
        }
        finally {
            close(in);
        }
    }

    /// writes a temp file then renames it, so a pulled battery can't leave half a map
    public boolean save(File file) {

        if (file == null) return false;
        File dir = file.getParentFile();
        if (dir != null && !dir.isDirectory() && !dir.mkdirs()) return false;
        File temp = new File(file.getPath() + ".tmp");
        OutputStream out = null;
        try {
            out = new FileOutputStream(temp);
            out.write(toCsv().getBytes(Charset.forName("UTF-8")));
            out.close();
            out = null;
            return (!file.exists() || file.delete()) && temp.renameTo(file);
        }
        catch (IOException e) {
            return false;
        }
        finally {
            close(out);
        }
    }

    private static void close(java.io.Closeable c) {

        if (c == null) return;
        try {
            c.close();
        }
        catch (IOException e) {
            //nothing to do
        }
    }

    private static double clamp(double x, double limit) {
        return Math.max(-limit, Math.min(limit, x));
    }
}
