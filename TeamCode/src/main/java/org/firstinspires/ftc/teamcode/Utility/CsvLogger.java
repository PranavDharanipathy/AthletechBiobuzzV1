package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.Charset;

//one low priority writer thread, the loop thread only appends to a reused buffer (no allocation per row)
public class CsvLogger {

    private static final int MAX_PENDING_CHARS = 4 << 20;

    private final File file;
    private final Object lock = new Object(), writeLock = new Object();
    private StringBuilder pending = new StringBuilder(1 << 16), spare = new StringBuilder(1 << 16);
    private final char[] digits = new char[9];
    private final Thread writerThread;
    private volatile boolean running = true;
    private Writer out;
    private long rows = 0, dropped = 0;

    public CsvLogger(File file, String... columns) {

        this.file = file;
        File parent = file.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) RobotLog.ee("CsvLogger", "can't make " + parent);

        for (int k = 0; k < columns.length; k++) {
            if (k > 0) pending.append(',');
            pending.append(columns[k]);
        }
        pending.append('\n');

        writerThread = new Thread(new Runnable() {
            @Override
            public void run() {

                while (running) {
                    try {
                        Thread.sleep(250);
                    } catch (InterruptedException e) {
                        break;
                    }
                    flush();
                }
            }
        }, "CsvLogger");
        writerThread.setPriority(Thread.MIN_PRIORITY);
        writerThread.setDaemon(true);
        writerThread.start();
    }

    public void row(double... values) {

        synchronized (lock) {
            if (pending.length() > MAX_PENDING_CHARS) {
                dropped++;
                return;
            }
            for (int k = 0; k < values.length; k++) {
                if (k > 0) pending.append(',');
                appendNumber(pending, values[k]);
            }
            pending.append('\n');
            rows++;
        }
    }

    private void appendNumber(StringBuilder sb, double v) {

        if (v != v) {
            sb.append("NaN");
            return;
        }
        double a = Math.abs(v);
        if (a >= 1e12) {
            sb.append(v);
            return;
        }

        int places = a < 1 ? 9 : 6;
        long scaled = Math.round(a * (places == 9 ? 1e9 : 1e6));
        long scale = places == 9 ? 1_000_000_000L : 1_000_000L;
        long whole = scaled / scale, fraction = scaled % scale;

        if (v < 0 && scaled != 0) sb.append('-');
        sb.append(whole);
        if (fraction == 0) return;

        int length = places;
        for (int k = places - 1; k >= 0; k--) {
            digits[k] = (char) ('0' + fraction % 10);
            fraction /= 10;
        }
        while (digits[length - 1] == '0') length--;
        sb.append('.').append(digits, 0, length);
    }

    public long getRows() {
        return rows;
    }

    public long getDropped() {
        return dropped;
    }

    public File getFile() {
        return file;
    }

    private void flush() {

        synchronized (writeLock) {
            flushLocked();
        }
    }

    private void flushLocked() {

        StringBuilder toWrite;
        synchronized (lock) {
            if (pending.length() == 0) return;
            toWrite = pending;
            pending = spare;
            spare = toWrite;
        }

        try {
            if (out == null) out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file, true), Charset.forName("UTF-8")), 1 << 16);
            out.append(toWrite);
            out.flush();
        } catch (IOException e) {
            RobotLog.ee("CsvLogger", e, "write failed: " + file);
        }

        synchronized (lock) {
            toWrite.setLength(0);
        }
    }

    public void close() {

        running = false;
        writerThread.interrupt();
        try {
            writerThread.join(1000);
        } catch (InterruptedException ignored) {
        }
        flush();
        try {
            if (out != null) out.close();
        } catch (IOException ignored) {
        }
    }
}
