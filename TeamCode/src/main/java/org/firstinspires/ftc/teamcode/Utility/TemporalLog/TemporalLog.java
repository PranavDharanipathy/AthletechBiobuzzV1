package org.firstinspires.ftc.teamcode.Utility.TemporalLog;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Utility.Hardware.DeviceTracker;
import org.firstinspires.ftc.teamcode.Utility.Hardware.HwOp;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.Charset;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

public final class TemporalLog {

    public static volatile boolean active;

    public static String root = "/sdcard/FIRST/data/TemporalLog";
    static final int RECORD = 24, BUFFER = 64 << 10, BUFFERS = 48, MAX_KEYS = 1 << 16;
    static final int FLAG = 1 << 30;

    private static ByteBuffer current;
    private static ArrayBlockingQueue<ByteBuffer> free, full;
    private static Thread writer;
    private static volatile boolean writing;
    private static OutputStream events, keys;
    private static final StringBuilder pendingKeys = new StringBuilder(1 << 14);
    private static final Object keyLock = new Object();
    private static long base, lastHandoff, loops, lastLoopNs;
    private static int keyCount, session;
    private static double[] lastValue = new double[1024];
    private static File dir;
    public static long records, dropped;
    private static VariableWatcher watcher;
    private static int loopKey, watchKey, droppedKey;

    private TemporalLog() {}

    public static synchronized File open(String opMode) {

        close();
        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        dir = new File(root, stamp);
        if (!dir.exists() && !dir.mkdirs()) {
            RobotLog.ee("TemporalLog", "can't make " + dir);
            return null;
        }
        try {
            events = new FileOutputStream(new File(dir, "events.bin"));
            keys = new FileOutputStream(new File(dir, "keys.tsv"));
            OutputStream meta = new FileOutputStream(new File(dir, "meta.txt"));
            String m = "opmode\t" + opMode + "\nstarted\t" + stamp + "\nwall_ms\t" + System.currentTimeMillis() + "\nformat\tevents.bin = records of 24 bytes LE: int64 ns since start, int32 key, int32 aux, float64 value\naux\tread/write: microseconds (bit 30 = cached read / redundant write); loop: loop index\n";
            meta.write(m.getBytes(Charset.forName("UTF-8")));
            meta.close();
        } catch (IOException e) {
            RobotLog.ee("TemporalLog", e, "can't open files in " + dir);
            return null;
        }
        free = new ArrayBlockingQueue<>(BUFFERS);
        full = new ArrayBlockingQueue<>(BUFFERS);
        for (int k = 0; k < BUFFERS; k++) free.add(ByteBuffer.allocate(BUFFER).order(ByteOrder.LITTLE_ENDIAN));
        current = free.poll();
        keyCount = 0;
        session++;
        records = dropped = loops = 0;
        base = System.nanoTime();
        lastHandoff = lastLoopNs = base;
        pendingKeys.setLength(0);
        keys("id\tkind\tname\tlabels");
        loopKey = key("loop", "loop", "");
        watchKey = key("meta", "temporal.watch_us", "");
        droppedKey = key("meta", "temporal.dropped_records", "");
        writing = true;
        writer = new Thread(TemporalLog::write, "TemporalLog");
        writer.setPriority(Thread.MIN_PRIORITY);
        writer.setDaemon(true);
        writer.start();
        active = true;
        return dir;
    }

    public static synchronized void watch(Object root, String rootName, Class<?>... statics) {

        if (!active) return;
        watcher = new VariableWatcher();
        watcher.addRoot(rootName, root);
        for (Class<?> c : statics) watcher.addStatics(c);
        watcher.sample(System.nanoTime());
    }

    public static void mark(String name, double value) {
        if (active) record(System.nanoTime(), key("phase", name, ""), 0, value);
    }

    public static void beginLoop() {

        if (!active) return;
        long now = System.nanoTime();
        record(now, loopKey, (int) loops, (now - lastLoopNs) * 1e-9);
        lastLoopNs = now;
        loops++;
    }

    public static void endLoop() {

        if (!active) return;
        long t0 = System.nanoTime();
        if (watcher != null) watcher.sample(t0);
        long t1 = System.nanoTime();
        record(t1, watchKey, 0, (t1 - t0) * 1e-3);
        if (dropped > 0) record(t1, droppedKey, 0, dropped);
        if (t1 - lastHandoff > 200_000_000L) handoff();
    }

    public static void io(DeviceTracker tracker, HwOp op, long t0, long t1, boolean flag, double value, Class<?> enumClass) {

        int[] k = tracker.temporalKeys;
        if (k == null || tracker.temporalSession != session) {
            if (k == null) k = new int[HwOp.count()];
            java.util.Arrays.fill(k, -1);
            tracker.temporalKeys = k;
            tracker.temporalSession = session;
        }
        int id = k[op.ordinal()];
        if (id < 0) {
            id = key(op.isRead() ? "read" : "write", tracker.getName() + "." + op.label, labels(enumClass));
            k[op.ordinal()] = id;
        }
        if (value != value && flag && op.isRead() && id < lastValue.length) value = lastValue[id];
        long us = Math.max(0, Math.min(FLAG - 1, (t1 - t0) / 1000));
        record(t1, id, (int) us | (flag ? FLAG : 0), value);
    }

    public static boolean stale(int keySession) {
        return keySession != session;
    }

    public static int session() {
        return session;
    }

    public static int newKey(String kind, String name, String labels) {
        return key(kind, name, labels);
    }

    public static void value(long t1, int key, long t0, double v) {
        if (active) record(t1, key, (int) Math.max(0, Math.min(FLAG - 1, (t1 - t0) / 1000)), v);
    }

    static String labels(Class<?> enumClass) {

        if (enumClass == null || !enumClass.isEnum()) return "";
        Object[] all = enumClass.getEnumConstants();
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < all.length; i++) {
            if (i > 0) sb.append('|');
            sb.append(i).append('=').append(all[i]);
        }
        return sb.toString();
    }

    static synchronized int key(String kind, String name, String labels) {

        if (keyCount >= MAX_KEYS) return MAX_KEYS - 1;
        int id = keyCount++;
        if (id >= lastValue.length) lastValue = java.util.Arrays.copyOf(lastValue, lastValue.length * 2);
        lastValue[id] = Double.NaN;
        keys(id + "\t" + kind + "\t" + name.replace('\t', ' ') + "\t" + labels);
        return id;
    }

    private static void keys(String line) {

        synchronized (keyLock) {
            pendingKeys.append(line).append('\n');
        }
    }

    //Blaze's callbacks read and write hardware on its own threads, and every call is recorded
    static synchronized void record(long tNs, int key, int aux, double value) {

        ByteBuffer b = current;
        if (b == null || b.remaining() < RECORD) {
            handoff();
            b = current;
            if (b == null) {
                dropped++;
                return;
            }
        }
        b.putLong(tNs - base);
        b.putInt(key);
        b.putInt(aux);
        b.putDouble(value);
        if (key < lastValue.length) lastValue[key] = value;
        records++;
    }

    private static synchronized void handoff() {

        ByteBuffer b = current;
        if (b != null && b.position() > 0) {
            full.offer(b);
            current = free.poll();
        }
        else if (b == null) current = free.poll();
        lastHandoff = System.nanoTime();
    }

    private static void write() {

        while (writing || !full.isEmpty()) {
            ByteBuffer b;
            try {
                b = full.poll(250, TimeUnit.MILLISECONDS);
            } catch (InterruptedException e) {
                b = full.poll();
                if (b == null && !writing) break;
            }
            flushKeys();
            if (b == null) continue;
            try {
                events.write(b.array(), 0, b.position());
            } catch (IOException e) {
                RobotLog.ee("TemporalLog", e, "write failed");
            }
            b.clear();
            free.offer(b);
        }
        flushKeys();
    }

    private static void flushKeys() {

        String s;
        synchronized (keyLock) {
            if (pendingKeys.length() == 0) return;
            s = pendingKeys.toString();
            pendingKeys.setLength(0);
        }
        try {
            keys.write(s.getBytes(Charset.forName("UTF-8")));
        } catch (IOException e) {
            RobotLog.ee("TemporalLog", e, "key write failed");
        }
    }

    public static File directory() {
        return dir;
    }

    public static int keyCount() {
        return keyCount;
    }

    public static int variableCount() {
        return watcher == null ? 0 : watcher.leafCount();
    }

    public static synchronized void close() {

        if (!active && writer == null) return;
        active = false;
        if (current != null && current.position() > 0) full.offer(current);
        current = null;
        writing = false;
        if (writer != null) {
            writer.interrupt();
            try {
                writer.join(3000);
            } catch (InterruptedException ignored) {
            }
        }
        writer = null;
        watcher = null;
        try {
            if (events != null) events.close();
            if (keys != null) keys.close();
        } catch (IOException ignored) {
        }
        events = keys = null;
    }
}
