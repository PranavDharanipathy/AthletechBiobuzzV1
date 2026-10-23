package org.firstinspires.ftc.teamcode.Utility.Monitoring;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Utility.Hardware.HwOp;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * The monitor's on-robot database: one folder per session under
 * {@code /sdcard/FIRST/data/HardwareMonitor/sessions/}, readable after the OpMode stops and
 * across robot restarts.
 *
 * <pre>
 * HardwareMonitor/
 *   index.csv                 one row per finished session
 *   sessions/&lt;id&gt;/
 *     summary.json            all aggregate statistics (rewritten every few seconds)
 *     devices.csv             device id, name, type, caching, connection
 *     events.csv              every read/write: time, device, op, cached/redundant, latency
 *     system.csv              time series: loop time, I/O time, battery, current, energy, ...
 *     motors.csv              time series per motor: current, velocity, back-EMF, power
 * </pre>
 *
 * Copy it to a laptop with {@code adb pull /sdcard/FIRST/data/HardwareMonitor}, browse it on the
 * robot with the "Hardware Monitor Viewer" OpMode, and delete it with
 * {@link HardwareMonitor#clearDatabase()}.
 *
 * <p>All file I/O for a running session happens on one background thread; the robot loop only
 * enqueues rows.
 */
public final class MonitorStore {

    public static final String ROOT_NAME = "HardwareMonitor";
    static final String INDEX_HEADER = "session,opmode,start,duration_s,loops,avg_loop_ms,"
            + "p95_loop_ms,min_battery_v,energy_wh,charge_mah,devices,complete";

    private static final String TAG = HardwareMonitor.TAG;
    private static final Object FS_LOCK = new Object();
    private static final long FLUSH_PERIOD_MS = 250;

    public static File rootDir() {
        return new File(AppUtil.ROBOT_DATA_DIR, ROOT_NAME);
    }

    public static File sessionsDir() {
        return new File(rootDir(), "sessions");
    }

    public static File indexFile() {
        return new File(rootDir(), "index.csv");
    }

    /** Supplies session data to the writer thread. */
    interface Provider {
        String summaryJson(boolean complete);

        String indexRow();
    }

    // ================================================================== session writer

    private final File dir;
    private final EventBuffer events;
    private EventBuffer.Batch spare;
    private final long maxEventBytes;
    private final long summaryPeriodNs;
    private final Provider provider;
    private final ScheduledExecutorService exec;
    private final AtomicBoolean finishing = new AtomicBoolean();

    private final ConcurrentLinkedQueue<String> deviceRows = new ConcurrentLinkedQueue<>();
    private final ConcurrentLinkedQueue<String> systemRows = new ConcurrentLinkedQueue<>();
    private final ConcurrentLinkedQueue<String> motorRows = new ConcurrentLinkedQueue<>();
    private volatile String systemHeader;
    private final ArrayList<String> deviceNames = new ArrayList<>();
    private final StringBuilder line = new StringBuilder(128);

    private Writer devicesOut;
    private Writer systemOut;
    private Writer motorsOut;
    private Writer eventsOut;
    private boolean systemHeaderWritten;
    private volatile long eventBytes;
    private volatile long eventsWritten;
    private volatile boolean eventLimitReached;
    private long lastSummaryNs;
    private volatile boolean closed;
    private volatile String lastError;

    MonitorStore(File dir, EventBuffer events, long maxEventBytes, long summaryPeriodMs, Provider provider) {
        this.dir = dir;
        this.events = events;
        this.spare = events == null ? null : new EventBuffer.Batch(events.capacity());
        this.maxEventBytes = maxEventBytes;
        this.summaryPeriodNs = summaryPeriodMs * 1_000_000L;
        this.provider = provider;
        this.lastSummaryNs = System.nanoTime();
        this.exec = Executors.newSingleThreadScheduledExecutor(new ThreadFactory() {
            @Override
            public Thread newThread(Runnable r) {
                Thread t = new Thread(r, "HardwareMonitor-writer");
                t.setDaemon(true);
                t.setPriority(Thread.MIN_PRIORITY);
                return t;
            }
        });
        exec.scheduleWithFixedDelay(new Runnable() {
            @Override
            public void run() {
                flush(false);
            }
        }, FLUSH_PERIOD_MS, FLUSH_PERIOD_MS, TimeUnit.MILLISECONDS);
    }

    File dir() {
        return dir;
    }

    void addDevice(int id, String name, String csvRow) {
        synchronized (deviceNames) {
            while (deviceNames.size() <= id) deviceNames.add("?");
            deviceNames.set(id, csv(name));
        }
        deviceRows.add(csvRow);
    }

    /** Sets the system.csv header (first call wins - columns depend on the hubs present). */
    void setSystemHeader(String header) {
        if (systemHeader == null) systemHeader = header;
    }

    boolean hasSystemHeader() {
        return systemHeader != null;
    }

    void addSystemRow(String row) {
        if (!closed) systemRows.add(row);
    }

    void addMotorRow(String row) {
        if (!closed) motorRows.add(row);
    }

    long eventBytes() {
        return eventBytes;
    }

    long eventsWritten() {
        return eventsWritten;
    }

    boolean eventLimitReached() {
        return eventLimitReached;
    }

    String lastError() {
        return lastError;
    }

    /** Writes everything still queued plus the final summary, then stops the writer thread. */
    void finish() {
        if (!finishing.compareAndSet(false, true)) return;
        Runnable last = new Runnable() {
            @Override
            public void run() {
                finishNow();
            }
        };
        try {
            exec.execute(last);
        } catch (RejectedExecutionException e) {
            last.run();
        }
        exec.shutdown();
    }

    private void finishNow() {
        flush(true);
        String row = null;
        try {
            row = provider.indexRow();
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "HardwareMonitor: building index row failed");
        }
        synchronized (FS_LOCK) {
            try {
                if (row != null) appendIndexRow(row);
            } catch (IOException e) {
                lastError = e.toString();
            }
            closeQuietly(devicesOut);
            closeQuietly(systemOut);
            closeQuietly(motorsOut);
            closeQuietly(eventsOut);
            closed = true;
        }
    }

    private void flush(boolean finalFlush) {
        if (closed) return;
        long now = System.nanoTime();
        String summary = null;
        if (finalFlush || now - lastSummaryNs >= summaryPeriodNs) {
            try {
                summary = provider.summaryJson(finalFlush);
            } catch (RuntimeException e) {
                RobotLog.ee(TAG, e, "HardwareMonitor: building summary failed");
            }
            lastSummaryNs = now;
        }
        synchronized (FS_LOCK) {
            if (closed) return;
            try {
                openIfNeeded();
                drain(deviceRows, devicesOut);
                String header = systemHeader;
                if (!systemHeaderWritten && header != null) {
                    systemOut.write(header);
                    systemOut.write('\n');
                    systemHeaderWritten = true;
                }
                if (systemHeaderWritten) drain(systemRows, systemOut);
                drain(motorRows, motorsOut);
                drainEvents();
                devicesOut.flush();
                systemOut.flush();
                motorsOut.flush();
                if (eventsOut != null) eventsOut.flush();
                if (summary != null) writeAtomically(new File(dir, "summary.json"), summary);
            } catch (IOException | RuntimeException e) {
                lastError = e.toString();
                RobotLog.ee(TAG, e, "HardwareMonitor: writing session data failed");
            }
        }
    }

    private void openIfNeeded() throws IOException {
        if (devicesOut != null) return;
        if (!dir.isDirectory() && !dir.mkdirs()) throw new IOException("cannot create " + dir);
        devicesOut = open("devices.csv", "id,name,type,device_class,cache_reads,connection");
        systemOut = open("system.csv", null);
        motorsOut = open("motors.csv", "t_s,motor,commanded_power,current_a,velocity_tps,"
                + "bemf_v,bemf_electrical_v,bemf_to_battery,input_power_w,energy_wh");
        if (events != null) {
            eventsOut = open("events.csv", "t_s,device_id,device,op,kind,flag,latency_us");
        }
    }

    private Writer open(String name, String header) throws IOException {
        File f = new File(dir, name);
        boolean fresh = !f.exists() || f.length() == 0;
        Writer w = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(f, true), "UTF-8"), 1 << 16);
        if (fresh && header != null) {
            w.write(header);
            w.write('\n');
        }
        return w;
    }

    private static void drain(ConcurrentLinkedQueue<String> q, Writer out) throws IOException {
        String row;
        while ((row = q.poll()) != null) {
            out.write(row);
            out.write('\n');
        }
    }

    private void drainEvents() throws IOException {
        if (events == null || eventsOut == null) return;
        EventBuffer.Batch full = events.swap(spare);
        spare = full;
        if (eventLimitReached) return;
        String[] names;
        synchronized (deviceNames) {
            names = deviceNames.toArray(new String[0]);
        }
        StringBuilder sb = line;
        for (int i = 0; i < full.size; i++) {
            sb.setLength(0);
            appendFixed(sb, full.timeNs[i] / 1000, 6);            // seconds, microsecond resolution
            int id = full.deviceId[i];
            int op = full.op[i];
            sb.append(',').append(id).append(',')
                    .append(id >= 0 && id < names.length ? names[id] : "?").append(',')
                    .append(HwOp.fromOrdinal(op).label)
                    .append(',')
                    .append(HwOp.fromOrdinal(op).isRead() ? 'R' : 'W')
                    .append(',').append(full.flag[i] ? '1' : '0').append(',');
            appendFixed(sb, full.latencyNs[i] / 100, 1);          // microseconds, 0.1 us resolution
            sb.append('\n');
            eventsOut.append(sb);
            eventBytes += sb.length();
            eventsWritten++;
            if (eventBytes >= maxEventBytes) {
                eventLimitReached = true;
                eventsOut.write("# event log size limit reached; later events were not recorded\n");
                break;
            }
        }
    }

    /** Appends {@code value / 10^decimals} with exactly {@code decimals} digits after the point. */
    static void appendFixed(StringBuilder sb, long value, int decimals) {
        if (value < 0) {
            sb.append('-');
            value = -value;
        }
        long scale = 1;
        for (int i = 0; i < decimals; i++) scale *= 10;
        sb.append(value / scale);
        if (decimals == 0) return;
        sb.append('.');
        String frac = Long.toString(value % scale);
        for (int i = frac.length(); i < decimals; i++) sb.append('0');
        sb.append(frac);
    }

    private static void writeAtomically(File target, String content) throws IOException {
        File tmp = new File(target.getPath() + ".tmp");
        Writer w = new OutputStreamWriter(new FileOutputStream(tmp, false), "UTF-8");
        try {
            w.write(content);
        } finally {
            w.close();
        }
        if (!tmp.renameTo(target)) {
            // some filesystems refuse to rename over an existing file
            //noinspection ResultOfMethodCallIgnored
            target.delete();
            if (!tmp.renameTo(target)) throw new IOException("cannot replace " + target);
        }
    }

    private static void appendIndexRow(String row) throws IOException {
        File root = rootDir();
        if (!root.isDirectory() && !root.mkdirs()) throw new IOException("cannot create " + root);
        File index = indexFile();
        boolean fresh = !index.exists() || index.length() == 0;
        Writer w = new OutputStreamWriter(new FileOutputStream(index, true), "UTF-8");
        try {
            if (fresh) w.write(INDEX_HEADER + "\n");
            w.write(row);
            w.write('\n');
        } finally {
            w.close();
        }
    }

    private static void closeQuietly(Writer w) {
        if (w == null) return;
        try {
            w.close();
        } catch (IOException ignored) {
            // nothing useful to do
        }
    }

    /** Makes a value safe for a CSV cell. */
    static String csv(String s) {
        if (s == null) return "";
        return s.replace(',', ';').replace('\n', ' ').replace('\r', ' ');
    }

    // ================================================================== reading / clearing

    /** One stored session as listed by {@link #listSessions()}. */
    public static final class SessionInfo {
        public final String id;
        public final File dir;
        public final String opMode;
        public final String start;
        public final double durationS;
        public final long loops;
        public final double avgLoopMs;
        public final double p95LoopMs;
        public final double minBatteryV;
        public final double energyWh;
        public final int devices;
        /** False while recording, or if the robot lost power before the session ended. */
        public final boolean complete;
        public final long sizeBytes;

        SessionInfo(String id, File dir, String[] row, long sizeBytes) {
            this.id = id;
            this.dir = dir;
            this.sizeBytes = sizeBytes;
            if (row != null && row.length >= 12) {
                opMode = row[1];
                start = row[2];
                durationS = parse(row[3]);
                loops = (long) parse(row[4]);
                avgLoopMs = parse(row[5]);
                p95LoopMs = parse(row[6]);
                minBatteryV = parse(row[7]);
                energyWh = parse(row[8]);
                devices = (int) parse(row[10]);
                complete = "true".equals(row[11]);
            } else {
                int us = id.indexOf('_');
                opMode = us >= 0 ? id.substring(us + 1) : id;
                start = us >= 0 ? id.substring(0, us) : "";
                durationS = loops = 0;
                avgLoopMs = p95LoopMs = minBatteryV = energyWh = Double.NaN;
                devices = 0;
                complete = false;
            }
        }

        private static double parse(String s) {
            try {
                return Double.parseDouble(s);
            } catch (NumberFormatException e) {
                return Double.NaN;
            }
        }
    }

    /** All stored sessions, newest first. */
    public static List<SessionInfo> listSessions() {
        synchronized (FS_LOCK) {
            Map<String, String[]> index = readIndex();
            File[] dirs = sessionsDir().listFiles();
            List<SessionInfo> out = new ArrayList<>();
            if (dirs == null) return out;
            Arrays.sort(dirs, Collections.reverseOrder());
            for (File d : dirs) {
                if (!d.isDirectory()) continue;
                out.add(new SessionInfo(d.getName(), d, index.get(d.getName()), sizeOf(d)));
            }
            return out;
        }
    }

    /** Parsed summary.json of a session. */
    public static JSONObject readSummary(String sessionId) throws IOException, JSONException {
        File f = new File(new File(sessionsDir(), sessionId), "summary.json");
        String text;
        synchronized (FS_LOCK) {
            text = readText(f);
        }
        return new JSONObject(text);
    }

    /**
     * Deletes every stored session except {@code keepSessionId} (the one being recorded, may be
     * null) and the index. Returns how many sessions were deleted.
     */
    public static int clearAll(String keepSessionId) {
        synchronized (FS_LOCK) {
            int deleted = 0;
            File[] dirs = sessionsDir().listFiles();
            if (dirs != null) {
                for (File d : dirs) {
                    if (keepSessionId != null && keepSessionId.equals(d.getName())) continue;
                    if (deleteRecursively(d)) deleted++;
                }
            }
            //noinspection ResultOfMethodCallIgnored
            indexFile().delete();
            RobotLog.ii(TAG, "HardwareMonitor database cleared (%d sessions)", deleted);
            return deleted;
        }
    }

    /** Deletes one stored session and its index row. */
    public static boolean deleteSession(String sessionId) {
        synchronized (FS_LOCK) {
            boolean ok = deleteRecursively(new File(sessionsDir(), sessionId));
            Map<String, String[]> index = readIndex();
            if (index.remove(sessionId) != null) rewriteIndex(index);
            return ok;
        }
    }

    /** Total bytes used by the database. */
    public static long totalSizeBytes() {
        synchronized (FS_LOCK) {
            return sizeOf(rootDir());
        }
    }

    private static Map<String, String[]> readIndex() {
        Map<String, String[]> map = new HashMap<>();
        File f = indexFile();
        if (!f.exists()) return map;
        try {
            String[] lines = readText(f).split("\n");
            for (int i = 1; i < lines.length; i++) {
                String[] cells = lines[i].split(",", -1);
                if (cells.length > 0 && !cells[0].isEmpty()) map.put(cells[0], cells);
            }
        } catch (IOException e) {
            RobotLog.ww(TAG, "HardwareMonitor: cannot read index: " + e);
        }
        return map;
    }

    private static void rewriteIndex(Map<String, String[]> index) {
        List<String> ids = new ArrayList<>(index.keySet());
        Collections.sort(ids);
        StringBuilder sb = new StringBuilder(INDEX_HEADER).append('\n');
        for (String id : ids) {
            String[] cells = index.get(id);
            for (int i = 0; i < cells.length; i++) {
                if (i > 0) sb.append(',');
                sb.append(cells[i]);
            }
            sb.append('\n');
        }
        try {
            writeAtomically(indexFile(), sb.toString());
        } catch (IOException e) {
            RobotLog.ww(TAG, "HardwareMonitor: cannot rewrite index: " + e);
        }
    }

    private static String readText(File f) throws IOException {
        StringBuilder sb = new StringBuilder((int) Math.min(Integer.MAX_VALUE, Math.max(16, f.length())));
        BufferedReader r = new BufferedReader(new InputStreamReader(new FileInputStream(f), "UTF-8"));
        try {
            char[] buf = new char[8192];
            int n;
            while ((n = r.read(buf)) > 0) sb.append(buf, 0, n);
        } finally {
            r.close();
        }
        return sb.toString();
    }

    private static boolean deleteRecursively(File f) {
        File[] children = f.listFiles();
        if (children != null) {
            for (File c : children) deleteRecursively(c);
        }
        return f.delete() || !f.exists();
    }

    private static long sizeOf(File f) {
        if (f.isFile()) return f.length();
        long total = 0;
        File[] children = f.listFiles();
        if (children != null) {
            for (File c : children) total += sizeOf(c);
        }
        return total;
    }
}
