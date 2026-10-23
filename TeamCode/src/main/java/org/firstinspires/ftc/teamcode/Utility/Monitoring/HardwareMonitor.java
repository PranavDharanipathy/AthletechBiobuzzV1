package org.firstinspires.ftc.teamcode.Utility.Monitoring;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Utility.Hardware.DeviceStats;
import org.firstinspires.ftc.teamcode.Utility.Hardware.DeviceTracker;
import org.firstinspires.ftc.teamcode.Utility.Hardware.HwOp;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotor;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.lang.ref.WeakReference;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Watches the robot's "undeterminable" variables - the ones you can't know until it runs - and
 * records them in an on-robot database you can review after the OpMode stops.
 *
 * <h3>What it measures</h3>
 * <ul>
 *   <li><b>Loop timing</b>: loop time avg/min/max/std-dev, p50/p90/p95/p99, frequency.</li>
 *   <li><b>Hardware I/O per loop</b>: time blocked in hardware calls, share of the loop,
 *       reads/writes/cache hits per loop.</li>
 *   <li><b>Every device</b> (via the {@code Monitored*} wrappers): reads, writes, cache hits,
 *       redundant writes, read/write frequency, latency, and a timestamp for every call.</li>
 *   <li><b>Power</b>: battery voltage (start/min/avg/end, brown-out dips), total current from all
 *       hubs, total power, energy used (Wh) and charge drawn (mAh).</li>
 *   <li><b>Each REV hub</b>: current, input voltage, 5 V rail, I2C and GPIO bus current,
 *       temperature, "not responding" events.</li>
 *   <li><b>Each motor</b>: current, velocity, back-EMF (from velocity, and from current if a
 *       winding resistance is set), back-EMF as a fraction of battery voltage, electrical power
 *       and energy.</li>
 *   <li><b>The JVM and the monitor itself</b>: heap use, CPU use, and how much time the monitor
 *       spent on its own sampling.</li>
 * </ul>
 *
 * <h3>Using it</h3>
 * <pre>
 * DcMotorEx lift = new MonitoredDcMotorEx(hardwareMap, "lift", true);
 * IMU imu       = new MonitoredIMU(hardwareMap, "imu", true);
 * waitForStart();
 * while (opModeIsActive()) {
 *     HardwareMonitor.get().update();   // once per loop: timing, cache refresh, sampling
 *     ...
 *     HardwareMonitor.get().addTelemetry(telemetry);
 *     telemetry.update();
 * }
 * </pre>
 * Sessions start and are saved automatically when OpModes init and stop ({@link
 * MonitorOpModeHook}). {@link #start(OpMode)} / {@link #stop()} are there for manual control.
 * Review results with the "Hardware Monitor Viewer" OpMode or
 * {@code adb pull /sdcard/FIRST/data/HardwareMonitor}; wipe them with {@link #clearDatabase()}.
 *
 * <h3>Cost</h3>
 * Counting a call costs well under a microsecond. The monitor's own sampling (battery, hubs,
 * motor current) is limited to {@link Config#maxSampleReadsPerUpdate} hardware reads per
 * {@link #update()} and re-uses any value your code already read through a wrapper. All file
 * writing happens on a background thread.
 */
public final class HardwareMonitor {

    public static final String TAG = "HardwareMonitor";

    /** Tunables. Change them before the OpMode starts; most take effect on the next session. */
    public static final class Config {
        /** Master switch. When false nothing is sampled or saved; wrappers still count and cache. */
        public volatile boolean enabled = true;
        /** Save sessions to the on-robot database. */
        public volatile boolean persist = true;
        /** Log every individual read/write with its timestamp (events.csv). */
        public volatile boolean logEvents = true;
        /** Stop logging individual events once events.csv reaches this size (bytes). */
        public volatile long maxEventLogBytes = 64L * 1024 * 1024;
        /** Safety net for read caching: a cached value older than this is re-read anyway. */
        public volatile long cacheMaxAgeMs = 50;
        /** Hardware reads the monitor may spend on its own sampling per update(). 0 = none. */
        public volatile int maxSampleReadsPerUpdate = 1;
        public volatile long batterySamplePeriodMs = 100;
        public volatile long hubCurrentSamplePeriodMs = 200;
        /** Hub input voltage, 5 V rail, I2C/GPIO current and temperature. */
        public volatile long hubDetailSamplePeriodMs = 1000;
        public volatile long motorSamplePeriodMs = 200;
        /** How often a row is added to system.csv. */
        public volatile long timeSeriesPeriodMs = 50;
        /** How often summary.json is refreshed while recording. */
        public volatile long summaryPeriodMs = 5000;
        /** Battery dips below this count as low-voltage events. */
        public volatile double lowVoltageThreshold = 9.0;
        /**
         * If set, applied to every REV hub when the hardware map is seen. MANUAL caches are
         * cleared at the start of every {@link #update()}. Null leaves the hubs alone.
         */
        public volatile LynxModule.BulkCachingMode bulkCachingMode = null;
    }

    public static final Config CONFIG = new Config();

    private static final HardwareMonitor INSTANCE = new HardwareMonitor();
    private static volatile long cycle;

    private static final int EVENT_BUFFER_CAPACITY = 32768;
    private static final int RECENT_LOOPS = 256;
    private static final int SUMMARY_RECENT_EVENTS = 64;

    public static HardwareMonitor get() {
        return INSTANCE;
    }

    /** Internal: the read-cache generation. Advances on every {@link #update()}. */
    public static long currentCycle() {
        return cycle;
    }

    /** Internal: max age of a cached read, in nanoseconds. */
    public static long cacheMaxAgeNs() {
        return CONFIG.cacheMaxAgeMs * 1_000_000L;
    }

    // ------------------------------------------------------------------ state

    private final AtomicLong ioNs = new AtomicLong();
    private final AtomicLong ioReads = new AtomicLong();
    private final AtomicLong ioWrites = new AtomicLong();
    private final AtomicLong ioCacheHits = new AtomicLong();

    private volatile Session session;
    private volatile boolean hooked;

    // guarded by "this"
    private final List<DeviceTracker> devices = new ArrayList<>();
    private final List<MonitoredDcMotor> motors = new ArrayList<>();
    private HardwareMap hardwareMap;
    private String pendingName;
    private WeakReference<OpMode> pendingOpMode = new WeakReference<>(null);
    private long pendingRunStartNs = -1;
    private boolean stoppedByUser;

    private HardwareMonitor() {
    }

    // ================================================================== public API

    /**
     * Starts (or joins) monitoring for {@code opMode}. Optional: when the lifecycle hook is active
     * sessions begin on their own. If you call it, call it first in init, before creating
     * {@code Monitored*} devices.
     */
    public static HardwareMonitor start(OpMode opMode) {
        HardwareMonitor m = INSTANCE;
        synchronized (m) {
            if (opMode != null && m.pendingOpMode.get() != opMode) {
                Session s = m.session;
                if (s != null && s.opMode.get() != null && s.opMode.get() != opMode) {
                    m.endSessionLocked("a new OpMode called start()");
                }
                m.pendingName = displayName(opMode);
                m.pendingOpMode = new WeakReference<>(opMode);
            }
            m.stoppedByUser = false;
            if (opMode != null && opMode.hardwareMap != null) m.attachHardwareMapLocked(opMode.hardwareMap);
            m.ensureSessionLocked();
        }
        return m;
    }

    /** Ends the session now and saves it. Monitoring stays off until the next OpMode (or start()). */
    public void stop() {
        synchronized (this) {
            stoppedByUser = true;
            endSessionLocked("stop() called");
        }
    }

    /**
     * Call once per loop. Measures loop time, refreshes the read cache of every
     * {@code Monitored*} device, takes a few budgeted samples (battery, hubs, motors) and queues
     * a time-series row.
     */
    public void update() {
        long now = System.nanoTime();
        cycle++;
        if (!CONFIG.enabled) return;
        Session s = session;
        if (s == null || !s.systemTasksBuilt) {
            synchronized (this) {
                if (stoppedByUser) return;
                s = ensureSessionLocked();
                if (s != null && !s.systemTasksBuilt) {
                    if (hardwareMap == null) {
                        OpMode op = pendingOpMode.get();
                        if (op != null && op.hardwareMap != null) hardwareMap = op.hardwareMap;
                    }
                    if (hardwareMap != null) buildSystemTasks(s, hardwareMap);
                }
            }
            if (s == null) return;
        }
        long reads = ioReads.getAndSet(0);
        long writes = ioWrites.getAndSet(0);
        long hits = ioCacheHits.getAndSet(0);
        long io = ioNs.getAndSet(0);
        synchronized (s) {
            onUpdate(s, now, reads, writes, hits, io);
            s.overheadNs += System.nanoTime() - now;
        }
    }

    /** Deletes every stored session except the one currently recording. Returns how many. */
    public static int clearDatabase() {
        Session s = INSTANCE.session;
        return MonitorStore.clearAll(s != null ? s.id : null);
    }

    /** Whether a session is currently being recorded. */
    public boolean isRecording() {
        return session != null;
    }

    /** Id (folder name) of the current session, or null. */
    public String getSessionId() {
        Session s = session;
        return s == null ? null : s.id;
    }

    /** Folder of the current session, or null when not persisting. */
    public File getSessionDir() {
        Session s = session;
        return s == null || s.store == null ? null : s.store.dir();
    }

    /** Whether OpMode init/stop are detected automatically (see {@link MonitorOpModeHook}). */
    public boolean isLifecycleHookActive() {
        return hooked;
    }

    /** Devices registered in the current OpMode. */
    public synchronized List<DeviceTracker> getDevices() {
        return new ArrayList<>(devices);
    }

    public double getLastLoopMs() {
        Session s = session;
        return s == null ? Double.NaN : s.lastLoopMs;
    }

    /** Average loop time over the last {@value #RECENT_LOOPS} loops. */
    public double getAverageLoopMs() {
        Session s = session;
        if (s == null) return Double.NaN;
        synchronized (s) {
            return s.recentMean();
        }
    }

    public double getBatteryVoltage() {
        Session s = session;
        return s == null ? Double.NaN : s.batteryLatest;
    }

    public double getTotalCurrentAmps() {
        Session s = session;
        if (s == null) return Double.NaN;
        synchronized (s) {
            return s.totalCurrentLatest();
        }
    }

    public double getEnergyWh() {
        Session s = session;
        return s == null ? 0 : s.energyWh;
    }

    /**
     * The current session's statistics as JSON (same format as summary.json), computed now.
     * Null when nothing is recording. Used by the web dashboard's live view.
     */
    public String liveSummaryJson() {
        Session s = session;
        return s == null ? null : buildSummary(s, false);
    }

    // ================================================================== telemetry

    /** Adds a compact live summary (including one line per device) to telemetry. */
    public void addTelemetry(Telemetry telemetry) {
        addTelemetry(telemetry, true);
    }

    public void addTelemetry(Telemetry telemetry, boolean perDevice) {
        Session s = session;
        if (s == null) {
            telemetry.addData("HW Monitor", CONFIG.enabled ? "idle (call update() each loop)" : "disabled");
            return;
        }
        List<DeviceStats.Snapshot> snaps = null;
        if (perDevice) {
            List<DeviceTracker> list;
            synchronized (s) {
                list = new ArrayList<>(s.devices);
            }
            snaps = new ArrayList<>(list.size());
            for (DeviceTracker t : list) snaps.add(t.getStats().snapshot());
        }
        synchronized (s) {
            double avg = s.recentMean();
            telemetry.addData("HW Monitor", "%s  %.1f s%s", s.opModeName,
                    (System.nanoTime() - s.startNs) / 1e9, hooked ? "" : "  (no lifecycle hook)");
            telemetry.addData("Loop", "%.1f ms (%.0f Hz)  p95 %.1f  max %.1f",
                    avg, avg > 0 ? 1000.0 / avg : 0, s.recentPercentile(95), s.loopMs.max());
            double hw = s.hwMsPerLoop.mean();
            telemetry.addData("HW I/O", "%.2f ms/loop (%.0f%%)  %.1f R  %.1f W  %.1f cached",
                    hw, avg > 0 ? 100.0 * hw / avg : 0,
                    s.readsPerLoop.mean(), s.writesPerLoop.mean(), s.hitsPerLoop.mean());
            telemetry.addData("Battery", "%.2f V (min %.2f)  %.2f A  %.3f Wh",
                    s.batteryLatest, s.battery.min(), s.totalCurrentLatest(), s.energyWh);
            for (HubState h : s.hubs) {
                telemetry.addData(h.name, "%.2f A  in %.2f V  5V %.2f  %.0f C",
                        h.current.last(), h.inputV.last(), h.auxV.last(), h.tempC.last());
            }
            for (MotorState m : s.motors) {
                telemetry.addData(m.motor.getName(), "%.2f A  %.0f t/s  bemf %.2f V",
                        m.latestCurrent, m.latestVelocity, m.latestBemf);
            }
        }
        if (snaps != null) {
            for (DeviceStats.Snapshot d : snaps) {
                telemetry.addData(d.name, "%.0f r/s  %.0f w/s  cache %.0f%%  rd %.2f ms",
                        d.recentReadHz, d.recentWriteHz, d.cacheHitPercent(), d.avgReadLatencyUs / 1000.0);
            }
        }
    }

    // ================================================================== called by wrappers

    /** Internal: a {@code Monitored*} device was created. */
    public void register(DeviceTracker tracker) {
        synchronized (this) {
            devices.add(tracker);
            Session s = session;
            if (s != null) {
                synchronized (s) {
                    addDevice(s, tracker);
                }
            } else if (pendingName != null && !stoppedByUser && CONFIG.enabled) {
                // We already know which OpMode is initialising (lifecycle hook or start()), so
                // open the session now: init-time I/O (IMU init, setMode, ...) gets logged too.
                ensureSessionLocked();
            }
        }
    }

    /** Internal: a monitored motor was created (enables current / back-EMF sampling). */
    public void registerMotor(MonitoredDcMotor motor) {
        synchronized (this) {
            motors.add(motor);
            Session s = session;
            if (s != null) {
                synchronized (s) {
                    addMotor(s, motor);
                }
            }
        }
    }

    /** Internal: gives the monitor access to hubs and the battery sensor. */
    public void attachHardwareMap(HardwareMap hw) {
        synchronized (this) {
            attachHardwareMapLocked(hw);
        }
    }

    /**
     * Internal: one hardware call finished. {@code flag} is "served from cache" for reads and
     * "re-sent the same value" for writes. Hot path: no locks except the event buffer's.
     */
    public void onIo(DeviceTracker tracker, HwOp op, long t0, long t1, boolean flag) {
        boolean read = op.isRead();
        if (read && flag) {
            ioCacheHits.incrementAndGet();
        } else {
            ioNs.addAndGet(t1 - t0);
        }
        if (read) ioReads.incrementAndGet();
        else ioWrites.incrementAndGet();
        Session s = session;
        if (s != null && s.events != null) {
            int id = tracker.getMonitorId();
            if (id >= 0) {
                int latency = (read && flag) ? 0 : (int) Math.min(Integer.MAX_VALUE, t1 - t0);
                s.events.add(t0 - s.startNs, id, op.ordinal(), latency, flag);
            }
        }
    }

    // ================================================================== OpMode lifecycle

    void setHooked(boolean hooked) {
        this.hooked = hooked;
    }

    synchronized void onOpModePreInit(OpMode opMode) {
        endSessionLocked("next OpMode initialising");
        if (isDefaultOpMode(opMode)) return;
        resetPendingLocked();
        pendingName = displayName(opMode);
        pendingOpMode = new WeakReference<>(opMode);
        cycle++;
    }

    synchronized void onOpModePreStart(OpMode opMode) {
        if (isDefaultOpMode(opMode)) return;
        long now = System.nanoTime();
        pendingRunStartNs = now;
        Session s = session;
        if (s == null && !stoppedByUser && CONFIG.enabled && !devices.isEmpty()) s = ensureSessionLocked();
        if (s != null) {
            synchronized (s) {
                if (s.runStartNs < 0) s.runStartNs = now;
            }
        }
    }

    synchronized void onOpModePostStop(OpMode opMode) {
        if (isDefaultOpMode(opMode)) return;
        endSessionLocked("OpMode stopped");
        resetPendingLocked();
    }

    private static boolean isDefaultOpMode(OpMode opMode) {
        // The SDK's idle "$Stop$Robot$" OpMode runs between user OpModes.
        return opMode == null || opMode.getClass().getSimpleName().contains("DefaultOpMode");
    }

    // ================================================================== session management

    private void resetPendingLocked() {
        devices.clear();
        motors.clear();
        hardwareMap = null;
        pendingName = null;
        pendingOpMode = new WeakReference<>(null);
        pendingRunStartNs = -1;
        stoppedByUser = false;
        ioNs.set(0);
        ioReads.set(0);
        ioWrites.set(0);
        ioCacheHits.set(0);
    }

    private Session ensureSessionLocked() {
        Session s = session;
        if (s != null || !CONFIG.enabled) return s;
        String name = pendingName != null ? pendingName : "UnknownOpMode";
        long wall = System.currentTimeMillis();
        String id = uniqueSessionId(wall, name);
        s = new Session(id, name, wall, System.nanoTime(), pendingOpMode.get());
        s.runStartNs = pendingRunStartNs;
        if (CONFIG.persist) {
            s.events = CONFIG.logEvents ? new EventBuffer(EVENT_BUFFER_CAPACITY) : null;
            final Session fs = s;
            s.store = new MonitorStore(new File(MonitorStore.sessionsDir(), id), s.events,
                    CONFIG.maxEventLogBytes, CONFIG.summaryPeriodMs, new MonitorStore.Provider() {
                @Override
                public String summaryJson(boolean complete) {
                    return buildSummary(fs, complete);
                }

                @Override
                public String indexRow() {
                    return buildIndexRow(fs);
                }
            });
        }
        synchronized (s) {
            for (DeviceTracker t : devices) addDevice(s, t);
            for (MonitoredDcMotor m : motors) addMotor(s, m);
            if (hardwareMap == null) {
                OpMode op = pendingOpMode.get();
                if (op != null && op.hardwareMap != null) hardwareMap = op.hardwareMap;
            }
            if (hardwareMap != null) buildSystemTasks(s, hardwareMap);
        }
        ioNs.set(0);
        ioReads.set(0);
        ioWrites.set(0);
        ioCacheHits.set(0);
        session = s;
        RobotLog.ii(TAG, "HardwareMonitor recording session %s", id);
        return s;
    }

    private void endSessionLocked(String reason) {
        Session s = session;
        if (s == null) return;
        session = null;
        synchronized (s) {
            s.endNs = System.nanoTime();
        }
        if (s.store != null) s.store.finish();
        RobotLog.ii(TAG, "HardwareMonitor session %s ended (%s)", s.id, reason);
    }

    private void attachHardwareMapLocked(HardwareMap hw) {
        if (hw == null) return;
        if (hardwareMap == null) hardwareMap = hw;
        LynxModule.BulkCachingMode mode = CONFIG.bulkCachingMode;
        if (mode != null) {
            try {
                for (LynxModule hub : hw.getAll(LynxModule.class)) hub.setBulkCachingMode(mode);
            } catch (RuntimeException e) {
                RobotLog.ee(TAG, e, "HardwareMonitor: setting bulk caching mode failed");
            }
        }
        Session s = session;
        if (s != null) {
            synchronized (s) {
                buildSystemTasks(s, hardwareMap);
            }
        }
    }

    private static void addDevice(Session s, DeviceTracker t) {
        if (s.devices.contains(t)) return;
        int id = s.devices.size();
        s.devices.add(t);
        t.assignMonitorId(id);
        if (s.store != null) {
            String row = id + "," + MonitorStore.csv(t.getName()) + "," + t.getType() + ","
                    + t.getDeviceClass() + "," + t.isCachingReads() + "," + MonitorStore.csv(t.getConnectionInfo());
            s.store.addDevice(id, t.getName(), row);
        }
    }

    private static void addMotor(Session s, MonitoredDcMotor motor) {
        for (MotorState m : s.motors) if (m.motor == motor) return;
        MotorState state = new MotorState(motor);
        s.motors.add(state);
        if (motor.supportsElectricalSampling()) {
            long period = CONFIG.motorSamplePeriodMs;
            s.tasks.add(new MotorTask(motor.getName() + ".current", period, state, MotorTask.CURRENT));
            s.tasks.add(new MotorTask(motor.getName() + ".velocity", period, state, MotorTask.VELOCITY));
        }
    }

    private static void buildSystemTasks(Session s, HardwareMap hw) {
        if (s.systemTasksBuilt || hw == null) return;
        s.systemTasksBuilt = true;
        try {
            Iterator<VoltageSensor> it = hw.voltageSensor.iterator();
            if (it.hasNext()) s.tasks.add(new BatteryTask(it.next(), CONFIG.batterySamplePeriodMs));
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "HardwareMonitor: no battery voltage sensor");
        }
        try {
            List<LynxModule> hubs = hw.getAll(LynxModule.class);
            for (int i = 0; i < hubs.size(); i++) {
                LynxModule module = hubs.get(i);
                HubState h = new HubState(module, hubName(hw, module, i));
                s.hubs.add(h);
                s.tasks.add(new HubTask(h, HubTask.CURRENT, CONFIG.hubCurrentSamplePeriodMs));
                long detail = CONFIG.hubDetailSamplePeriodMs;
                s.tasks.add(new HubTask(h, HubTask.INPUT_V, detail));
                s.tasks.add(new HubTask(h, HubTask.AUX_V, detail));
                s.tasks.add(new HubTask(h, HubTask.I2C, detail));
                s.tasks.add(new HubTask(h, HubTask.GPIO, detail));
                s.tasks.add(new HubTask(h, HubTask.TEMP, detail));
            }
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "HardwareMonitor: reading hub list failed");
        }
        if (s.store != null) s.store.setSystemHeader(systemHeader(s));
    }

    private static String hubName(HardwareMap hw, LynxModule module, int index) {
        try {
            Set<String> names = hw.getNamesOf(module);
            if (names != null && !names.isEmpty()) return names.iterator().next();
        } catch (RuntimeException ignored) {
            // fall through
        }
        return "Hub" + index;
    }

    // ================================================================== per-loop work

    private void onUpdate(Session s, long now, long reads, long writes, long hits, long io) {
        if (s.lastUpdateNs > 0) {
            double loopMs = (now - s.lastUpdateNs) / 1e6;
            double hwMs = io / 1e6;
            s.loopMs.add(loopMs);
            s.loopHist.add(loopMs);
            s.lastLoopMs = loopMs;
            s.pushRecent(loopMs);
            s.hwMsPerLoop.add(hwMs);
            s.readsPerLoop.add(reads);
            s.writesPerLoop.add(writes);
            s.hitsPerLoop.add(hits);
            s.loops++;
            s.serLoopMs += loopMs;
            s.serHwMs += hwMs;
            s.serReads += reads;
            s.serWrites += writes;
            s.serHits += hits;
            s.serLoops++;
            integrate(s, loopMs / 1000.0);
        }
        s.lastUpdateNs = now;
        s.updates++;

        if (CONFIG.bulkCachingMode == LynxModule.BulkCachingMode.MANUAL) {
            for (HubState h : s.hubs) {
                try {
                    h.module.clearBulkCache();
                } catch (RuntimeException ignored) {
                    // hub went away; sampling will record it as not responding
                }
            }
        }

        runSampler(s, now);

        if (now - s.lastSeriesNs >= CONFIG.timeSeriesPeriodMs * 1_000_000L) {
            sampleJvm(s, now);
            if (s.store != null) emitSystemRow(s, now);
            s.lastSeriesNs = now;
            s.serLoopMs = s.serHwMs = 0;
            s.serReads = s.serWrites = s.serHits = 0;
            s.serLoops = 0;
        }
    }

    /** Runs the most overdue sample tasks, spending at most the configured number of reads. */
    private static void runSampler(Session s, long now) {
        if (s.tasks.isEmpty()) return;
        int budget = Math.max(0, CONFIG.maxSampleReadsPerUpdate);
        long stamp = s.updates;
        for (int pass = 0; pass < s.tasks.size(); pass++) {
            SampleTask best = null;
            long bestOverdue = -1;
            for (int i = 0; i < s.tasks.size(); i++) {
                SampleTask t = s.tasks.get(i);
                if (t.visited == stamp) continue;
                long overdue = now - t.lastNs - t.periodNs;
                if (overdue >= 0 && overdue > bestOverdue) {
                    best = t;
                    bestOverdue = overdue;
                }
            }
            if (best == null) return;
            best.visited = stamp;
            int result;
            try {
                result = best.run(s, now, budget > 0);
            } catch (RuntimeException e) {
                s.sampleErrors++;
                result = SampleTask.DID_IO;
            }
            if (result == SampleTask.DID_IO) {
                budget--;
                best.lastNs = now;
                best.runs++;
            } else if (result == SampleTask.NO_IO) {
                best.lastNs = now;
                best.runs++;
            }
        }
    }

    private static void integrate(Session s, double dtSeconds) {
        double v = s.batteryLatest;
        double amps = s.totalCurrentLatest();
        if (isFinite(v) && isFinite(amps)) {
            double watts = v * amps;
            s.energyWh += watts * dtSeconds / 3600.0;
            s.chargeMah += amps * dtSeconds * 1000.0 / 3600.0;
        }
        for (MotorState m : s.motors) {
            if (isFinite(m.latestPowerW)) m.energyWh += m.latestPowerW * dtSeconds / 3600.0;
        }
    }

    private static void sampleJvm(Session s, long now) {
        Runtime rt = Runtime.getRuntime();
        double heapMb = (rt.totalMemory() - rt.freeMemory()) / 1048576.0;
        s.heapMb.add(heapMb);
        s.latestHeapMb = heapMb;
        long cpuMs = android.os.Process.getElapsedCpuTime();
        if (s.lastCpuMs >= 0 && now > s.lastCpuWallNs) {
            double pct = 100.0 * (cpuMs - s.lastCpuMs) / ((now - s.lastCpuWallNs) / 1e6);
            s.cpuPct.add(pct);
            s.latestCpuPct = pct;
        }
        s.lastCpuMs = cpuMs;
        s.lastCpuWallNs = now;
    }

    private static String systemHeader(Session s) {
        StringBuilder sb = new StringBuilder("t_s,loop_ms,loop_avg_ms,loop_hz,hw_ms_per_loop,hw_pct,"
                + "reads_per_loop,writes_per_loop,cache_hits_per_loop,battery_v,total_current_a,"
                + "total_power_w,energy_wh,charge_mah,heap_mb,cpu_pct");
        for (HubState h : s.hubs) {
            String n = MonitorStore.csv(h.name).replace(' ', '_');
            sb.append(',').append(n).append("_current_a,").append(n).append("_input_v,")
                    .append(n).append("_aux_v,").append(n).append("_i2c_a,")
                    .append(n).append("_gpio_a,").append(n).append("_temp_c");
        }
        return sb.toString();
    }

    private static void emitSystemRow(Session s, long now) {
        if (!s.store.hasSystemHeader()) s.store.setSystemHeader(systemHeader(s));
        int n = Math.max(1, s.serLoops);
        double avg = s.serLoops > 0 ? s.serLoopMs / n : Double.NaN;
        double hw = s.serLoops > 0 ? s.serHwMs / n : Double.NaN;
        double amps = s.totalCurrentLatest();
        StringBuilder sb = new StringBuilder(160);
        sb.append(f((now - s.startNs) / 1e9, 4)).append(',')
                .append(f(s.lastLoopMs, 3)).append(',')
                .append(f(avg, 3)).append(',')
                .append(f(avg > 0 ? 1000.0 / avg : Double.NaN, 1)).append(',')
                .append(f(hw, 3)).append(',')
                .append(f(avg > 0 ? 100.0 * hw / avg : Double.NaN, 1)).append(',')
                .append(f(s.serReads / (double) n, 2)).append(',')
                .append(f(s.serWrites / (double) n, 2)).append(',')
                .append(f(s.serHits / (double) n, 2)).append(',')
                .append(f(s.batteryLatest, 3)).append(',')
                .append(f(amps, 3)).append(',')
                .append(f(s.batteryLatest * amps, 2)).append(',')
                .append(f(s.energyWh, 5)).append(',')
                .append(f(s.chargeMah, 2)).append(',')
                .append(f(s.latestHeapMb, 1)).append(',')
                .append(f(s.latestCpuPct, 1));
        for (HubState h : s.hubs) {
            sb.append(',').append(f(h.current.last(), 3))
                    .append(',').append(f(h.inputV.last(), 3))
                    .append(',').append(f(h.auxV.last(), 3))
                    .append(',').append(f(h.i2cA.last(), 3))
                    .append(',').append(f(h.gpioA.last(), 3))
                    .append(',').append(f(h.tempC.last(), 1));
        }
        s.store.addSystemRow(sb.toString());
    }

    // ================================================================== summary / index

    private static String buildIndexRow(Session s) {
        synchronized (s) {
            long end = s.endNs > 0 ? s.endNs : System.nanoTime();
            return MonitorStore.csv(s.id) + "," + MonitorStore.csv(s.opModeName) + ","
                    + formatWall(s.startWallMs) + "," + f((end - s.startNs) / 1e9, 1) + ","
                    + s.loops + "," + f(s.loopMs.mean(), 2) + "," + f(s.loopHist.percentile(95), 2) + ","
                    + f(s.battery.min(), 2) + "," + f(s.energyWh, 4) + "," + f(s.chargeMah, 1) + ","
                    + s.devices.size() + ",true";
        }
    }

    private String buildSummary(Session s, boolean complete) {
        try {
            JSONObject root = new JSONObject();
            root.put("format", "HardwareMonitor/1");
            synchronized (s) {
                long now = s.endNs > 0 ? s.endNs : System.nanoTime();

                JSONObject sess = new JSONObject();
                sess.put("id", s.id);
                sess.put("opMode", s.opModeName);
                sess.put("start", formatWall(s.startWallMs));
                sess.put("startWallMs", s.startWallMs);
                num(sess, "durationS", (now - s.startNs) / 1e9);
                if (s.runStartNs > 0) {
                    num(sess, "initS", (s.runStartNs - s.startNs) / 1e9);
                    num(sess, "runS", (now - s.runStartNs) / 1e9);
                }
                sess.put("complete", complete);
                sess.put("lifecycleHook", hooked);
                sess.put("updates", s.updates);
                root.put("session", sess);

                JSONObject loop = new JSONObject();
                loop.put("count", s.loops);
                num(loop, "avgMs", s.loopMs.mean());
                num(loop, "minMs", s.loopMs.min());
                num(loop, "maxMs", s.loopMs.max());
                num(loop, "stdMs", s.loopMs.stddev());
                num(loop, "p50Ms", s.loopHist.percentile(50));
                num(loop, "p90Ms", s.loopHist.percentile(90));
                num(loop, "p95Ms", s.loopHist.percentile(95));
                num(loop, "p99Ms", s.loopHist.percentile(99));
                double avg = s.loopMs.mean();
                num(loop, "avgHz", avg > 0 ? 1000.0 / avg : Double.NaN);
                num(loop, "hwMsPerLoop", s.hwMsPerLoop.mean());
                num(loop, "hwMsPerLoopMax", s.hwMsPerLoop.max());
                num(loop, "hwPctOfLoop", avg > 0 ? 100.0 * s.hwMsPerLoop.mean() / avg : Double.NaN);
                num(loop, "readsPerLoop", s.readsPerLoop.mean());
                num(loop, "writesPerLoop", s.writesPerLoop.mean());
                num(loop, "cacheHitsPerLoop", s.hitsPerLoop.mean());
                root.put("loop", loop);

                JSONObject power = new JSONObject();
                JSONObject batt = stats(s.battery);
                num(batt, "start", s.batteryStart);
                num(batt, "end", s.batteryLatest);
                power.put("batteryV", batt);
                num(power, "lowVoltageThresholdV", CONFIG.lowVoltageThreshold);
                power.put("lowVoltageEvents", s.lowVoltageEvents);
                power.put("totalCurrentA", stats(s.totalCurrent));
                power.put("totalPowerW", stats(s.totalPower));
                num(power, "energyWh", s.energyWh);
                num(power, "chargeMah", s.chargeMah);
                root.put("power", power);

                JSONArray hubs = new JSONArray();
                for (HubState h : s.hubs) {
                    JSONObject o = new JSONObject();
                    o.put("name", h.name);
                    o.put("currentA", stats(h.current));
                    o.put("inputV", stats(h.inputV));
                    o.put("auxV", stats(h.auxV));
                    o.put("i2cA", stats(h.i2cA));
                    o.put("gpioA", stats(h.gpioA));
                    o.put("tempC", stats(h.tempC));
                    o.put("notRespondingSamples", h.notRespondingSamples);
                    o.put("bulkCachingMode", h.bulkCachingMode());
                    hubs.put(o);
                }
                root.put("hubs", hubs);

                JSONArray motorArr = new JSONArray();
                for (MotorState m : s.motors) {
                    JSONObject o = new JSONObject();
                    o.put("name", m.motor.getName());
                    o.put("motorType", m.typeName());
                    o.put("currentA", stats(m.current));
                    o.put("velocityTps", stats(m.velocity));
                    o.put("bemfV", stats(m.bemf));
                    o.put("bemfElectricalV", stats(m.bemfElectrical));
                    o.put("bemfToBattery", stats(m.bemfRatio));
                    o.put("inputPowerW", stats(m.power));
                    num(o, "energyWh", m.energyWh);
                    num(o, "windingResistanceOhms", m.motor.getWindingResistance());
                    o.put("samples", m.samples);
                    motorArr.put(o);
                }
                root.put("motors", motorArr);

                double totalIo = 0;
                List<DeviceStats.Snapshot> snaps = new ArrayList<>();
                for (DeviceTracker t : s.devices) {
                    DeviceStats.Snapshot d = t.getStats().snapshot();
                    snaps.add(d);
                    totalIo += d.totalIoMs;
                }
                JSONArray devs = new JSONArray();
                for (int i = 0; i < snaps.size(); i++) {
                    devs.put(deviceJson(s, s.devices.get(i), snaps.get(i), totalIo));
                }
                root.put("devices", devs);

                JSONObject jvm = new JSONObject();
                jvm.put("heapUsedMb", stats(s.heapMb));
                num(jvm, "heapMaxMb", Runtime.getRuntime().maxMemory() / 1048576.0);
                jvm.put("cpuPctOfOneCore", stats(s.cpuPct));
                jvm.put("threads", Thread.activeCount());
                jvm.put("processors", Runtime.getRuntime().availableProcessors());
                root.put("jvm", jvm);

                JSONObject mon = new JSONObject();
                num(mon, "overheadMs", s.overheadNs / 1e6);
                num(mon, "overheadMsPerLoop", s.updates > 0 ? s.overheadNs / 1e6 / s.updates : Double.NaN);
                mon.put("sampleReads", s.sampleReads);
                num(mon, "sampleReadMs", s.sampleReadNs / 1e6);
                mon.put("sampleErrors", s.sampleErrors);
                if (s.events != null) {
                    mon.put("eventsLogged", s.events.accepted());
                    mon.put("eventsDropped", s.events.dropped());
                }
                if (s.store != null) {
                    mon.put("eventLogBytes", s.store.eventBytes());
                    mon.put("eventLogLimitReached", s.store.eventLimitReached());
                    String err = s.store.lastError();
                    if (err != null) mon.put("lastWriteError", err);
                }
                JSONObject cfg = new JSONObject();
                cfg.put("cacheMaxAgeMs", CONFIG.cacheMaxAgeMs);
                cfg.put("maxSampleReadsPerUpdate", CONFIG.maxSampleReadsPerUpdate);
                cfg.put("timeSeriesPeriodMs", CONFIG.timeSeriesPeriodMs);
                cfg.put("bulkCachingMode", String.valueOf(CONFIG.bulkCachingMode));
                mon.put("config", cfg);
                root.put("monitor", mon);
            }
            return root.toString(1);
        } catch (JSONException e) {
            return "{\"error\":\"" + e.toString().replace('"', '\'') + "\"}";
        }
    }

    private static JSONObject deviceJson(Session s, DeviceTracker t, DeviceStats.Snapshot d, double totalIoMs)
            throws JSONException {
        JSONObject o = new JSONObject();
        o.put("id", t.getMonitorId());
        o.put("name", d.name);
        o.put("type", d.type);
        o.put("deviceClass", t.getDeviceClass());
        o.put("connection", t.getConnectionInfo());
        o.put("cacheReads", t.isCachingReads());
        o.put("reads", d.reads);
        o.put("writes", d.writes);
        o.put("cacheHits", d.cacheHits);
        num(o, "cacheHitPct", d.cacheHitPercent());
        o.put("hardwareReads", d.hardwareReads);
        o.put("redundantWrites", d.redundantWrites);
        num(o, "recentReadHz", d.recentReadHz);
        num(o, "recentWriteHz", d.recentWriteHz);
        num(o, "avgReadHz", d.avgReadHz);
        num(o, "avgWriteHz", d.avgWriteHz);
        num(o, "avgReadLatencyUs", d.avgReadLatencyUs);
        num(o, "maxReadLatencyUs", d.maxReadLatencyUs);
        num(o, "avgWriteLatencyUs", d.avgWriteLatencyUs);
        num(o, "maxWriteLatencyUs", d.maxWriteLatencyUs);
        num(o, "totalIoMs", d.totalIoMs);
        num(o, "ioSharePct", totalIoMs > 0 ? 100.0 * d.totalIoMs / totalIoMs : Double.NaN);
        num(o, "firstReadS", rel(s, d.firstReadNs));
        num(o, "lastReadS", rel(s, d.lastReadNs));
        num(o, "firstWriteS", rel(s, d.firstWriteNs));
        num(o, "lastWriteS", rel(s, d.lastWriteNs));
        o.put("monitorReads", d.monitorReads);
        num(o, "monitorIoMs", d.monitorIoMs);

        JSONArray ops = new JSONArray();
        for (DeviceStats.OpSnapshot op : d.ops) {
            JSONObject x = new JSONObject();
            x.put("op", op.op.label);
            x.put("kind", op.op.isRead() ? "R" : "W");
            x.put("count", op.count);
            x.put(op.op.isRead() ? "cacheHits" : "redundant", op.flagged);
            num(x, "avgLatencyUs", op.avgLatencyUs);
            num(x, "minLatencyUs", op.minLatencyUs);
            num(x, "maxLatencyUs", op.maxLatencyUs);
            num(x, "avgHz", op.avgHz);
            num(x, "firstS", rel(s, op.firstNs));
            num(x, "lastS", rel(s, op.lastNs));
            ops.put(x);
        }
        o.put("ops", ops);

        JSONArray recent = new JSONArray();
        int from = Math.max(0, d.recentTimeNs.length - SUMMARY_RECENT_EVENTS);
        for (int i = from; i < d.recentTimeNs.length; i++) {
            JSONArray e = new JSONArray();
            e.put(round(rel(s, d.recentTimeNs[i]), 6));
            e.put(d.recentOp[i].label);
            e.put(round(d.recentLatencyNs[i] / 1000.0, 1));
            e.put(d.recentFlag[i]);
            recent.put(e);
        }
        o.put("recentEvents", recent);
        return o;
    }

    private static JSONObject stats(RunningStats r) throws JSONException {
        JSONObject o = new JSONObject();
        o.put("n", r.count());
        num(o, "avg", r.mean());
        num(o, "min", r.min());
        num(o, "max", r.max());
        num(o, "std", r.stddev());
        num(o, "last", r.last());
        return o;
    }

    /** Puts a number, writing null for NaN/infinity (which JSON cannot represent). */
    private static void num(JSONObject o, String key, double v) throws JSONException {
        if (isFinite(v)) o.put(key, round(v, 4));
        else o.put(key, JSONObject.NULL);
    }

    private static double round(double v, int decimals) {
        if (!isFinite(v)) return 0;
        double scale = Math.pow(10, decimals);
        return Math.round(v * scale) / scale;
    }

    /** Seconds since session start for a nanoTime value, NaN for "never". */
    private static double rel(Session s, long ns) {
        return ns < 0 ? Double.NaN : (ns - s.startNs) / 1e9;
    }

    // ================================================================== helpers

    static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    /** Fixed-decimal CSV number; empty for NaN. */
    static String f(double v, int decimals) {
        if (!isFinite(v)) return "";
        return String.format(Locale.US, "%." + decimals + "f", v);
    }

    static String formatWall(long wallMs) {
        return new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date(wallMs));
    }

    private static String uniqueSessionId(long wallMs, String name) {
        String base = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date(wallMs))
                + "_" + sanitize(name);
        File dir = MonitorStore.sessionsDir();
        String id = base;
        for (int i = 2; new File(dir, id).exists(); i++) id = base + "-" + i;
        return id;
    }

    private static String sanitize(String name) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < name.length() && sb.length() < 40; i++) {
            char c = name.charAt(i);
            sb.append(Character.isLetterOrDigit(c) || c == '-' || c == '_' ? c : '-');
        }
        return sb.length() == 0 ? "OpMode" : sb.toString();
    }

    /** The name shown on the Driver Station, falling back to the class name. */
    static String displayName(OpMode opMode) {
        if (opMode == null) return "UnknownOpMode";
        Class<?> cls = opMode.getClass();
        TeleOp tele = cls.getAnnotation(TeleOp.class);
        if (tele != null && !tele.name().isEmpty()) return tele.name();
        Autonomous auto = cls.getAnnotation(Autonomous.class);
        if (auto != null && !auto.name().isEmpty()) return auto.name();
        return cls.getSimpleName();
    }

    // ================================================================== session data

    private static final class Session {
        final String id;
        final String opModeName;
        final long startWallMs;
        final long startNs;
        final WeakReference<OpMode> opMode;
        MonitorStore store;
        EventBuffer events;
        long runStartNs = -1;
        long endNs = -1;

        final List<DeviceTracker> devices = new ArrayList<>();
        final List<MotorState> motors = new ArrayList<>();
        final List<HubState> hubs = new ArrayList<>();
        final List<SampleTask> tasks = new ArrayList<>();
        boolean systemTasksBuilt;

        // loop
        final RunningStats loopMs = new RunningStats();
        final Histogram loopHist = new Histogram(0.25, 1200); // 0..300 ms
        final RunningStats hwMsPerLoop = new RunningStats();
        final RunningStats readsPerLoop = new RunningStats();
        final RunningStats writesPerLoop = new RunningStats();
        final RunningStats hitsPerLoop = new RunningStats();
        final double[] recent = new double[RECENT_LOOPS];
        final double[] scratch = new double[RECENT_LOOPS];
        int recentHead;
        int recentSize;
        long lastUpdateNs;
        long updates;
        long loops;
        double lastLoopMs = Double.NaN;

        // time-series accumulators since the last system.csv row
        long lastSeriesNs;
        double serLoopMs;
        double serHwMs;
        long serReads;
        long serWrites;
        long serHits;
        int serLoops;

        // power
        final RunningStats battery = new RunningStats();
        final RunningStats totalCurrent = new RunningStats();
        final RunningStats totalPower = new RunningStats();
        double batteryLatest = Double.NaN;
        double batteryStart = Double.NaN;
        long lowVoltageEvents;
        boolean belowThreshold;
        double energyWh;
        double chargeMah;

        // jvm
        final RunningStats heapMb = new RunningStats();
        final RunningStats cpuPct = new RunningStats();
        double latestHeapMb = Double.NaN;
        double latestCpuPct = Double.NaN;
        long lastCpuMs = -1;
        long lastCpuWallNs;

        // monitor
        long overheadNs;
        long sampleReads;
        long sampleReadNs;
        long sampleErrors;

        Session(String id, String opModeName, long startWallMs, long startNs, OpMode opMode) {
            this.id = id;
            this.opModeName = opModeName;
            this.startWallMs = startWallMs;
            this.startNs = startNs;
            this.opMode = new WeakReference<>(opMode);
        }

        void pushRecent(double v) {
            recent[recentHead] = v;
            recentHead = (recentHead + 1) % recent.length;
            if (recentSize < recent.length) recentSize++;
        }

        double recentMean() {
            if (recentSize == 0) return Double.NaN;
            double sum = 0;
            for (int i = 0; i < recentSize; i++) sum += recent[i];
            return sum / recentSize;
        }

        double recentPercentile(double p) {
            if (recentSize == 0) return Double.NaN;
            System.arraycopy(recent, 0, scratch, 0, recentSize);
            Arrays.sort(scratch, 0, recentSize);
            int idx = (int) Math.ceil(p / 100.0 * recentSize) - 1;
            return scratch[Math.max(0, Math.min(recentSize - 1, idx))];
        }

        void monitorRead(long ns) {
            sampleReads++;
            sampleReadNs += Math.max(0, ns);
        }

        double totalCurrentLatest() {
            double sum = 0;
            boolean any = false;
            for (HubState h : hubs) {
                double a = h.current.last();
                if (isFinite(a)) {
                    sum += a;
                    any = true;
                }
            }
            return any ? sum : Double.NaN;
        }

        void onBattery(double volts) {
            if (!isFinite(volts) || volts <= 0) return;
            battery.add(volts);
            batteryLatest = volts;
            if (Double.isNaN(batteryStart)) batteryStart = volts;
            double threshold = CONFIG.lowVoltageThreshold;
            if (!belowThreshold && volts < threshold) {
                belowThreshold = true;
                lowVoltageEvents++;
            } else if (belowThreshold && volts > threshold + 0.2) {
                belowThreshold = false;
            }
        }

        void onHubCurrent() {
            double amps = totalCurrentLatest();
            if (!isFinite(amps)) return;
            totalCurrent.add(amps);
            if (isFinite(batteryLatest)) totalPower.add(amps * batteryLatest);
        }

        void onMotorSample(MotorState m, long now) {
            MonitoredDcMotor motor = m.motor;
            double duty = motor.getCommandedPower();
            m.latestBemf = MotorElectricalModel.backEmfFromVelocity(m.latestVelocity, m.type);
            m.latestBemfElectrical = MotorElectricalModel.backEmfFromCurrent(
                    duty, batteryLatest, m.latestCurrent, motor.getWindingResistance());
            m.latestPowerW = MotorElectricalModel.inputPowerWatts(duty, batteryLatest, m.latestCurrent);
            double ratio = isFinite(batteryLatest) && batteryLatest > 0
                    ? Math.abs(m.latestBemf) / batteryLatest : Double.NaN;
            m.bemf.add(m.latestBemf);
            m.bemfElectrical.add(m.latestBemfElectrical);
            m.bemfRatio.add(ratio);
            m.power.add(m.latestPowerW);
            m.samples++;
            if (store != null) {
                store.addMotorRow(f((now - startNs) / 1e9, 4) + "," + MonitorStore.csv(motor.getName()) + ","
                        + f(duty, 3) + "," + f(m.latestCurrent, 3) + "," + f(m.latestVelocity, 1) + ","
                        + f(m.latestBemf, 3) + "," + f(m.latestBemfElectrical, 3) + "," + f(ratio, 3) + ","
                        + f(m.latestPowerW, 2) + "," + f(m.energyWh, 5));
            }
        }
    }

    private static final class HubState {
        final LynxModule module;
        final String name;
        final RunningStats current = new RunningStats();
        final RunningStats inputV = new RunningStats();
        final RunningStats auxV = new RunningStats();
        final RunningStats i2cA = new RunningStats();
        final RunningStats gpioA = new RunningStats();
        final RunningStats tempC = new RunningStats();
        long notRespondingSamples;

        HubState(LynxModule module, String name) {
            this.module = module;
            this.name = name;
        }

        String bulkCachingMode() {
            try {
                return String.valueOf(module.getBulkCachingMode());
            } catch (RuntimeException e) {
                return "unknown";
            }
        }
    }

    private static final class MotorState {
        final MonitoredDcMotor motor;
        final MotorConfigurationType type;
        final RunningStats current = new RunningStats();
        final RunningStats velocity = new RunningStats();
        final RunningStats bemf = new RunningStats();
        final RunningStats bemfElectrical = new RunningStats();
        final RunningStats bemfRatio = new RunningStats();
        final RunningStats power = new RunningStats();
        double latestCurrent = Double.NaN;
        double latestVelocity = Double.NaN;
        double latestBemf = Double.NaN;
        double latestBemfElectrical = Double.NaN;
        double latestPowerW = Double.NaN;
        double energyWh;
        long samples;

        MotorState(MonitoredDcMotor motor) {
            this.motor = motor;
            MotorConfigurationType t = null;
            try {
                t = motor.peekMotorType();
            } catch (RuntimeException ignored) {
                // leave null: back-EMF from velocity will be NaN
            }
            this.type = t;
        }

        String typeName() {
            try {
                return type == null ? "unknown" : type.getName();
            } catch (RuntimeException e) {
                return "unknown";
            }
        }
    }

    // ================================================================== sample tasks

    private abstract static class SampleTask {
        static final int DID_IO = 0;
        static final int NO_IO = 1;
        static final int SKIPPED = 2;

        final String name;
        final long periodNs;
        long lastNs;
        long visited = -1;
        long runs;

        SampleTask(String name, long periodMs) {
            this.name = name;
            this.periodNs = Math.max(1, periodMs) * 1_000_000L;
        }

        /** @return DID_IO, NO_IO (served without touching hardware) or SKIPPED (needed I/O, none allowed) */
        abstract int run(Session s, long now, boolean allowIo);
    }

    private static final class BatteryTask extends SampleTask {
        final VoltageSensor sensor;

        BatteryTask(VoltageSensor sensor, long periodMs) {
            super("battery", periodMs);
            this.sensor = sensor;
        }

        @Override
        int run(Session s, long now, boolean allowIo) {
            if (!allowIo) return SKIPPED;
            long t0 = System.nanoTime();
            double v = sensor.getVoltage();
            s.monitorRead(System.nanoTime() - t0);
            s.onBattery(v);
            return DID_IO;
        }
    }

    private static final class HubTask extends SampleTask {
        static final int CURRENT = 0;
        static final int INPUT_V = 1;
        static final int AUX_V = 2;
        static final int I2C = 3;
        static final int GPIO = 4;
        static final int TEMP = 5;

        final HubState hub;
        final int metric;

        HubTask(HubState hub, int metric, long periodMs) {
            super(hub.name + "#" + metric, periodMs);
            this.hub = hub;
            this.metric = metric;
        }

        @Override
        int run(Session s, long now, boolean allowIo) {
            if (!allowIo) return SKIPPED;
            LynxModule m = hub.module;
            long t0 = System.nanoTime();
            switch (metric) {
                case CURRENT:
                    hub.current.add(m.getCurrent(CurrentUnit.AMPS));
                    break;
                case INPUT_V:
                    hub.inputV.add(m.getInputVoltage(VoltageUnit.VOLTS));
                    break;
                case AUX_V:
                    hub.auxV.add(m.getAuxiliaryVoltage(VoltageUnit.VOLTS));
                    break;
                case I2C:
                    hub.i2cA.add(m.getI2cBusCurrent(CurrentUnit.AMPS));
                    break;
                case GPIO:
                    hub.gpioA.add(m.getGpioBusCurrent(CurrentUnit.AMPS));
                    break;
                default:
                    hub.tempC.add(m.getTemperature(TempUnit.CELSIUS));
                    break;
            }
            s.monitorRead(System.nanoTime() - t0);
            if (metric == CURRENT) {
                if (m.isNotResponding()) hub.notRespondingSamples++;
                s.onHubCurrent();
            }
            return DID_IO;
        }
    }

    private static final class MotorTask extends SampleTask {
        static final int CURRENT = 0;
        static final int VELOCITY = 1;

        final MotorState state;
        final int kind;

        MotorTask(String name, long periodMs, MotorState state, int kind) {
            super(name, periodMs);
            this.state = state;
            this.kind = kind;
        }

        @Override
        int run(Session s, long now, boolean allowIo) {
            MonitoredDcMotor m = state.motor;
            long seenNs = kind == CURRENT ? m.getObservedCurrentNs() : m.getObservedVelocityNs();
            int result;
            double value;
            if (seenNs > 0 && now - seenNs < periodNs) {
                // your code already read it this period: reuse instead of another bus transaction
                value = kind == CURRENT ? m.getObservedCurrentAmps() : m.getObservedVelocity();
                result = NO_IO;
            } else if (!allowIo) {
                return SKIPPED;
            } else {
                long t0 = System.nanoTime();
                value = kind == CURRENT ? m.sampleCurrentAmps() : m.sampleVelocity();
                s.monitorRead(System.nanoTime() - t0);
                result = DID_IO;
            }
            if (kind == CURRENT) {
                state.latestCurrent = value;
                state.current.add(value);
            } else {
                state.latestVelocity = value;
                state.velocity.add(value);
            }
            s.onMotorSample(state, now);
            return result;
        }
    }
}
