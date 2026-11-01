package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Writes finished segments and the event log on one background thread, so the control loop never
 * waits on flash storage.
 *
 * <p>Session layout:
 * <ul>
 *   <li>{@code NNN_passive.csv}, {@code NNN_controller.csv}, {@code NNN_open_loop.csv}: one file
 *       per segment, with the activity in the name, in the header, and in every row.</li>
 *   <li>{@code NNN_open_loop_steps.csv}: per-step summary for each open-loop segment.</li>
 *   <li>{@code events.csv}: confirmations, arming, every stop with its reason, and segment
 *       boundaries.</li>
 * </ul>
 */
public final class DiagnosticRecorder {

    private static final Charset UTF8 = Charset.forName("UTF-8");

    private final File dir;
    private final ExecutorService writer;
    private final ConcurrentLinkedQueue<String> pendingEvents = new ConcurrentLinkedQueue<>();
    private final AtomicInteger segmentsWritten = new AtomicInteger();
    private final AtomicInteger writeFailures = new AtomicInteger();
    private final AtomicInteger pending = new AtomicInteger();
    private volatile String lastError = "";
    private final boolean usable;

    public DiagnosticRecorder(File dir) {
        this.dir = dir;
        boolean ok = dir.isDirectory() || dir.mkdirs();
        if (!ok) lastError = "cannot create " + dir;
        usable = ok;
        writer = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "TurretDiagnosticWriter");
            t.setDaemon(true);
            return t;
        });
        if (ok) {
            pendingEvents.add("t_s,event,activity,run_id,detail\n");
        }
    }

    /** False if the session directory could not be created. Active tests are refused then. */
    public boolean isUsable() { return usable; }
    public File directory() { return dir; }
    public int segmentsWritten() { return segmentsWritten.get(); }
    public int writeFailures() { return writeFailures.get(); }
    public int pendingWrites() { return pending.get(); }
    public String lastError() { return lastError; }

    /** Queues one event line (not written until {@link #flushEvents} or a segment write). */
    public void event(double tS, String type, DiagnosticActivity activity, int runId, String detail) {
        String safe = detail == null ? "" : detail.replace('"', '\'').replace('\n', ' ');
        pendingEvents.add(String.format(Locale.US, "%.4f,%s,%s,%d,\"%s\"\n", tS, type, activity.label, runId, safe));
    }

    /** Hands a finished segment to the writer thread. The buffer must not be touched afterwards. */
    public void submit(final SegmentBuffer segment, final double motionThresholdDeg) {
        if (!usable) return;
        pending.incrementAndGet();
        writer.execute(() -> {
            try {
                String base = String.format(Locale.US, "%03d_%s", segment.sequence, segment.activity.label);
                try (Writer out = open(new File(dir, base + ".csv"), false)) {
                    segment.writeCsv(out);
                }
                if (segment.activity == DiagnosticActivity.OPEN_LOOP && segment.rows() > 0) {
                    List<OpenLoopStepAnalyzer.StepResult> steps = OpenLoopStepAnalyzer.analyze(segment, motionThresholdDeg);
                    try (Writer out = open(new File(dir, base + "_steps.csv"), false)) {
                        OpenLoopStepAnalyzer.writeCsv(steps, out);
                    }
                }
                segmentsWritten.incrementAndGet();
            } catch (IOException | RuntimeException e) {
                writeFailures.incrementAndGet();
                lastError = e.toString();
            } finally {
                writeEventsNow();
                pending.decrementAndGet();
            }
        });
    }

    public void flushEvents() {
        if (!usable) return;
        pending.incrementAndGet();
        writer.execute(() -> {
            try {
                writeEventsNow();
            } finally {
                pending.decrementAndGet();
            }
        });
    }

    /** Stops accepting work and waits up to {@code waitMs} for queued writes. */
    public boolean shutdown(long waitMs) {
        flushEvents();
        writer.shutdown();
        try {
            return writer.awaitTermination(waitMs, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return false;
        }
    }

    private void writeEventsNow() {
        if (pendingEvents.isEmpty()) return;
        try (Writer out = open(new File(dir, "events.csv"), true)) {
            String line;
            while ((line = pendingEvents.poll()) != null) out.write(line);
        } catch (IOException | RuntimeException e) {
            writeFailures.incrementAndGet();
            lastError = e.toString();
        }
    }

    private static Writer open(File file, boolean append) throws IOException {
        return new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file, append), UTF8), 1 << 16);
    }
}
