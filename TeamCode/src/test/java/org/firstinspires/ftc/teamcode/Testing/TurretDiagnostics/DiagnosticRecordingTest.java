package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.StringWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.*;

public class DiagnosticRecordingTest {

    @Rule public TemporaryFolder tmp = new TemporaryFolder();

    private static final int NEUTRAL = OpenLoopSchedule.Kind.NEUTRAL.code;
    private static final int PULSE = OpenLoopSchedule.Kind.PULSE.code;

    /** Appends rows of one step at 100 Hz, with a position function of time-in-step. */
    private static double[] add(SegmentBuffer seg, double t, int index, int kind, double cmd, int rows,
                                double startDeg, double velDegS, int deadRows) {
        double pos = startDeg;
        for (int i = 0; i < rows; i++) {
            double inStep = i * 0.01;
            double v = i >= deadRows ? velDegS : 0;
            if (i > 0) pos += v * 0.01;
            assertTrue(seg.beginRow());
            seg.set(Col.T_S, t + inStep);
            seg.set(Col.LOOP_DT_MS, 10);
            seg.set(Col.ACTIVITY, DiagnosticActivity.OPEN_LOOP.code);
            seg.set(Col.STEP_INDEX, index);
            seg.set(Col.STEP_KIND, kind);
            seg.set(Col.CMD_REQUESTED, cmd);
            seg.set(Col.POS_DEG, pos);
            seg.set(Col.VEL_DEG_S, v);
            seg.commitRow();
        }
        return new double[] {t + rows * 0.01, pos};
    }

    private static SegmentBuffer syntheticRun() {
        SegmentBuffer seg = new SegmentBuffer(DiagnosticActivity.OPEN_LOOP, 2, 1, 1000, Collections.singletonList("note=test"));
        double[] s = add(seg, 0, 0, NEUTRAL, 0, 20, 0, 0, 0);
        s = add(seg, s[0], 1, PULSE, 0.1, 20, s[1], 50, 5);         // + from rest, 50 ms dead time
        s = add(seg, s[0], 2, NEUTRAL, 0, 20, s[1], 0, 0);
        s = add(seg, s[0], 3, PULSE, 0.1, 20, s[1], 50, 2);         // + same direction, 20 ms
        s = add(seg, s[0], 4, NEUTRAL, 0, 20, s[1], 0, 0);
        s = add(seg, s[0], 5, PULSE, -0.1, 20, s[1], -50, 9);       // reversal, 90 ms (play)
        add(seg, s[0], 6, NEUTRAL, 0, 20, s[1], 0, 0);
        return seg;
    }

    @Test public void analyzerSeparatesStartReversalAndRestart() {
        List<OpenLoopStepAnalyzer.StepResult> r = OpenLoopStepAnalyzer.analyze(syntheticRun(), 0.1);
        assertEquals(3, r.size());
        assertEquals(-1, r.get(0).reversal);
        assertEquals(0, r.get(1).reversal);
        assertEquals(1, r.get(2).reversal);
        // Dead time = first sample at least 0.1 deg from the step's start position.
        assertEquals(0.05, r.get(0).deadTimeS, 1e-9);
        assertEquals(0.02, r.get(1).deadTimeS, 1e-9);
        assertEquals(0.09, r.get(2).deadTimeS, 1e-9);
        assertTrue(r.get(2).deadTimeS - r.get(1).deadTimeS > 0.05);
        for (OpenLoopStepAnalyzer.StepResult s : r) {
            assertTrue(s.moved);
            assertTrue(s.complete);
            assertEquals(1, s.polarityAgrees, 0);
            assertEquals(50, s.peakSpeedDegS, 1e-9);
            assertEquals(10, s.loopDtMeanMs, 1e-9);
        }
    }

    @Test public void analyzerFlagsPolarityMismatchAndNoMotion() {
        SegmentBuffer seg = new SegmentBuffer(DiagnosticActivity.OPEN_LOOP, 1, 1, 500, Collections.<String>emptyList());
        double[] s = add(seg, 0, 1, PULSE, 0.1, 20, 0, -40, 0);   // + command, moves negative
        s = add(seg, s[0], 2, NEUTRAL, 0, 10, s[1], 0, 0);
        add(seg, s[0], 3, PULSE, 0.02, 20, s[1], 0, 0);           // below breakaway
        List<OpenLoopStepAnalyzer.StepResult> r = OpenLoopStepAnalyzer.analyze(seg, 0.1);
        assertEquals(0, r.get(0).polarityAgrees, 0);
        assertFalse(r.get(1).moved);
        assertTrue(Double.isNaN(r.get(1).deadTimeS));
        assertFalse("segment ended during the step", r.get(1).complete);
    }

    @Test public void fullBufferRefusesRowsAndCsvLabelsActivity() throws Exception {
        SegmentBuffer seg = new SegmentBuffer(DiagnosticActivity.CONTROLLER, 7, 3, 2, Collections.singletonList("k=v"));
        assertTrue(seg.beginRow()); seg.set(Col.T_S, 0.5); seg.commitRow();
        assertTrue(seg.beginRow()); seg.set(Col.TICKS, 12); seg.commitRow();
        assertFalse(seg.beginRow());
        assertTrue(seg.isFull());
        StringWriter w = new StringWriter();
        seg.writeCsv(w);
        String csv = w.toString();
        assertTrue(csv.startsWith("# activity=controller\n"));
        assertTrue(csv.contains("# k=v\n"));
        String[] lines = csv.split("\n");
        String header = lines[5];
        assertEquals(Col.COUNT, header.split(",", -1).length);
        assertTrue(header.startsWith("t_s,loop_dt_ms,activity"));
        assertEquals("unset cells are empty", Col.COUNT, lines[6].split(",", -1).length);
        assertTrue(lines[6].startsWith("0.5,,"));
    }

    @Test public void recorderWritesSegmentStepsAndEvents() throws Exception {
        File dir = new File(tmp.getRoot(), "session");
        DiagnosticRecorder rec = new DiagnosticRecorder(dir);
        assertTrue(rec.isUsable());
        rec.event(1.5, "armed", DiagnosticActivity.OPEN_LOOP, 1, "schedule");
        rec.submit(syntheticRun(), 0.1);
        assertTrue(rec.shutdown(5000));
        assertEquals(0, rec.writeFailures());
        File data = new File(dir, "002_open_loop.csv");
        File steps = new File(dir, "002_open_loop_steps.csv");
        File events = new File(dir, "events.csv");
        assertTrue(data.isFile());
        assertTrue(steps.isFile());
        String eventText = new String(Files.readAllBytes(events.toPath()), StandardCharsets.UTF_8);
        assertTrue(eventText.startsWith("t_s,event,activity,run_id,detail\n"));
        assertTrue(eventText.contains("armed,open_loop,1,\"schedule\""));
        List<String> stepLines = Files.readAllLines(steps.toPath(), StandardCharsets.UTF_8);
        assertEquals(4, stepLines.size());
        assertTrue(stepLines.get(3).startsWith("5,pulse,-0.1,1,1,"));
    }

    @Test public void unusableDirectoryIsReported() throws Exception {
        File blocker = tmp.newFile("not_a_dir");
        DiagnosticRecorder rec = new DiagnosticRecorder(new File(blocker, "child"));
        assertFalse(rec.isUsable());
        rec.shutdown(1000);
    }
}
