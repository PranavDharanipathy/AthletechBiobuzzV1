package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;

/**
 * Summarizes each open-loop motion step of a recorded segment, to help separate the causes of
 * turret shake:
 * <ul>
 *   <li><b>Static friction:</b> {@code dead_time_s} and whether a level moved at all
 *       (breakaway command).</li>
 *   <li><b>Mechanical play:</b> {@code dead_time_s} on a reversal compared with a same-direction
 *       restart at the same level. The servo-side gear train is not encoder-visible, so play shows
 *       only as extra reversal delay.</li>
 *   <li><b>Friction / viscous loss:</b> {@code late_speed_deg_s} versus command on holds.</li>
 *   <li><b>Inertia vs. drag:</b> {@code coast_disp_deg} and {@code coast_decel_deg_s2} after the
 *       command returns to neutral.</li>
 *   <li><b>Loop timing:</b> {@code loop_dt_mean_ms} and {@code loop_dt_max_ms} during the step.</li>
 * </ul>
 * These are descriptive measurements from one run, not fitted model parameters. Compare repeated
 * runs before drawing conclusions.
 */
public final class OpenLoopStepAnalyzer {

    /** Speed below which the turret counts as stopped after a step, deg/s. */
    static final double STOPPED_SPEED_DEG_PER_S = 2.0;

    public static final class StepResult {
        public int stepIndex;
        public int kindCode;
        public double command;
        /** 1 = opposite direction to the previous motion step, 0 = same, -1 = no previous step. */
        public int reversal = -1;
        public boolean complete;
        public double startT;
        public double durationS;
        public double startDeg;
        public double endDeg;
        public double dispDeg;
        public boolean moved;
        public double deadTimeS = Double.NaN;
        public double peakSpeedDegS = Double.NaN;
        public double lateSpeedDegS = Double.NaN;
        /** 1 = moved the commanded way, 0 = moved against it, NaN = did not move. */
        public double polarityAgrees = Double.NaN;
        public double coastDispDeg = Double.NaN;
        public double coastStopS = Double.NaN;
        public double coastDecelDegS2 = Double.NaN;
        public double loopDtMeanMs = Double.NaN;
        public double loopDtMaxMs = Double.NaN;
    }

    private OpenLoopStepAnalyzer() {}

    public static List<StepResult> analyze(SegmentBuffer seg, double motionThresholdDeg) {
        List<StepResult> results = new ArrayList<>();
        int n = seg.rows();
        int prevDirection = 0;
        int r = 0;
        while (r < n) {
            int start = r;
            double index = seg.get(r, Col.STEP_INDEX);
            double kind = seg.get(r, Col.STEP_KIND);
            while (r < n && same(seg.get(r, Col.STEP_INDEX), index) && same(seg.get(r, Col.STEP_KIND), kind)) r++;
            int end = r; // exclusive

            int kindCode = (int) kind;
            if (kindCode != OpenLoopSchedule.Kind.PULSE.code && kindCode != OpenLoopSchedule.Kind.HOLD.code) continue;

            StepResult res = new StepResult();
            res.stepIndex = (int) index;
            res.kindCode = kindCode;
            res.command = seg.get(start, Col.CMD_REQUESTED);
            int direction = (int) Math.signum(res.command);
            if (prevDirection != 0) res.reversal = direction != prevDirection ? 1 : 0;
            if (direction != 0) prevDirection = direction;

            res.complete = end < n;
            res.startT = seg.get(start, Col.T_S);
            double lastT = seg.get(end - 1, Col.T_S);
            double nextT = end < n ? seg.get(end, Col.T_S) : lastT;
            res.durationS = nextT - res.startT;
            res.startDeg = seg.get(start, Col.POS_DEG);
            // End of the step = first sample after the command went neutral, if there is one.
            res.endDeg = seg.get(end < n ? end : end - 1, Col.POS_DEG);
            res.dispDeg = res.endDeg - res.startDeg;

            double dtSum = 0, dtMax = 0;
            int dtCount = 0;
            double peak = Double.NEGATIVE_INFINITY;
            for (int i = start; i < end; i++) {
                double dt = seg.get(i, Col.LOOP_DT_MS);
                if (!Double.isNaN(dt)) { dtSum += dt; dtMax = Math.max(dtMax, dt); dtCount++; }
                double v = seg.get(i, Col.VEL_DEG_S);
                if (!Double.isNaN(v)) peak = Math.max(peak, v * direction);
                double moved = Math.abs(seg.get(i, Col.POS_DEG) - res.startDeg);
                if (!res.moved && moved >= motionThresholdDeg) {
                    res.moved = true;
                    res.deadTimeS = seg.get(i, Col.T_S) - res.startT;
                }
            }
            if (dtCount > 0) { res.loopDtMeanMs = dtSum / dtCount; res.loopDtMaxMs = dtMax; }
            if (peak != Double.NEGATIVE_INFINITY) res.peakSpeedDegS = peak;

            int lateStart = start + (end - start) / 2;
            double lateSum = 0;
            int lateCount = 0;
            for (int i = lateStart; i < end; i++) {
                double v = seg.get(i, Col.VEL_DEG_S);
                if (!Double.isNaN(v)) { lateSum += v * direction; lateCount++; }
            }
            if (lateCount > 0) res.lateSpeedDegS = lateSum / lateCount;

            if (Math.abs(res.dispDeg) >= motionThresholdDeg) {
                res.polarityAgrees = Math.signum(res.dispDeg) == direction ? 1 : 0;
            }

            // Coast: the neutral run that immediately follows.
            if (end < n && (int) seg.get(end, Col.STEP_KIND) == OpenLoopSchedule.Kind.NEUTRAL.code) {
                int c = end;
                double neutralIndex = seg.get(end, Col.STEP_INDEX);
                double coastStart = seg.get(end, Col.POS_DEG);
                double speedAtNeutral = Math.abs(seg.get(end, Col.VEL_DEG_S));
                double t0 = seg.get(end, Col.T_S);
                double stoppedAt = Double.NaN;
                double lastPos = coastStart;
                while (c < n && same(seg.get(c, Col.STEP_INDEX), neutralIndex)
                        && (int) seg.get(c, Col.STEP_KIND) == OpenLoopSchedule.Kind.NEUTRAL.code) {
                    lastPos = seg.get(c, Col.POS_DEG);
                    if (Double.isNaN(stoppedAt) && Math.abs(seg.get(c, Col.VEL_DEG_S)) < STOPPED_SPEED_DEG_PER_S) {
                        stoppedAt = seg.get(c, Col.T_S) - t0;
                    }
                    c++;
                }
                res.coastDispDeg = (lastPos - coastStart) * direction;
                res.coastStopS = stoppedAt;
                if (stoppedAt > 0 && speedAtNeutral >= STOPPED_SPEED_DEG_PER_S) {
                    res.coastDecelDegS2 = speedAtNeutral / stoppedAt;
                }
            }
            results.add(res);
        }
        return results;
    }

    public static void writeCsv(List<StepResult> results, Writer out) throws IOException {
        out.write("step_index,kind,command,reversal,complete,start_t_s,duration_s,start_deg,end_deg,disp_deg,"
                + "moved,dead_time_s,peak_speed_deg_s,late_speed_deg_s,polarity_agrees,"
                + "coast_disp_deg,coast_stop_s,coast_decel_deg_s2,loop_dt_mean_ms,loop_dt_max_ms\n");
        StringBuilder sb = new StringBuilder(256);
        for (StepResult s : results) {
            sb.setLength(0);
            sb.append(s.stepIndex).append(',')
                    .append(s.kindCode == OpenLoopSchedule.Kind.PULSE.code ? "pulse" : "hold").append(',');
            SegmentBuffer.appendValue(sb, s.command); sb.append(',');
            sb.append(s.reversal).append(',').append(s.complete ? 1 : 0).append(',');
            double[] values = {s.startT, s.durationS, s.startDeg, s.endDeg, s.dispDeg};
            for (double v : values) { SegmentBuffer.appendValue(sb, v); sb.append(','); }
            sb.append(s.moved ? 1 : 0).append(',');
            double[] rest = {s.deadTimeS, s.peakSpeedDegS, s.lateSpeedDegS, s.polarityAgrees,
                    s.coastDispDeg, s.coastStopS, s.coastDecelDegS2, s.loopDtMeanMs, s.loopDtMaxMs};
            for (int i = 0; i < rest.length; i++) {
                if (i > 0) sb.append(',');
                SegmentBuffer.appendValue(sb, rest[i]);
            }
            sb.append('\n');
            out.write(sb.toString());
        }
    }

    private static boolean same(double a, double b) {
        return Double.compare(a, b) == 0;
    }
}
