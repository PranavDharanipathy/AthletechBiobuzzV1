package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Fixed sequence of bounded open-loop commands with a neutral interval between motion steps.
 *
 * <p>Per command level (ascending), the pulse pattern is {@code +, +, -, -}, each followed by
 * neutral. That gives a start from rest, a same-direction restart, a reversal, and another restart,
 * so dead time on reversal (play) can be compared with dead time on a same-direction restart
 * (static friction). Holds ({@code +, -} per level) reach steady speed for the speed-vs-command
 * curve. Net displacement is roughly zero.
 *
 * <p>Time only advances while running. After an abort, the interrupted step restarts from its
 * beginning, preceded by one neutral interval, so no step resumes mid-motion.
 */
public final class OpenLoopSchedule {

    public enum Kind {
        NEUTRAL(0), PULSE(1), HOLD(2), JOG(3);
        public final int code;
        Kind(int code) { this.code = code; }
    }

    public static final class Step {
        public final Kind kind;
        public final double command;
        public final double durationS;
        public final int levelIndex;

        Step(Kind kind, double command, double durationS, int levelIndex) {
            this.kind = kind;
            this.command = command;
            this.durationS = durationS;
            this.levelIndex = levelIndex;
        }
    }

    private final List<Step> steps;
    private final double neutralS;
    private int index;
    private long stepStartNanos;
    private boolean running;
    private boolean complete;
    private boolean resumeNeutral;

    public OpenLoopSchedule(List<Step> steps, double neutralS) {
        this.steps = Collections.unmodifiableList(new ArrayList<>(steps));
        this.neutralS = neutralS;
    }

    /** Builds the standard characterization sequence from validated settings. */
    public static OpenLoopSchedule build(TurretDiagnosticSettings s) {
        List<Step> list = new ArrayList<>();
        double[] levels = s.openLoopLevels.clone();
        java.util.Arrays.sort(levels);
        list.add(new Step(Kind.NEUTRAL, 0, s.neutralS, -1));
        double[] pulseSigns = {1, 1, -1, -1};
        for (int i = 0; i < levels.length; i++) {
            for (double sign : pulseSigns) {
                list.add(new Step(Kind.PULSE, sign * levels[i], s.pulseS, i));
                list.add(new Step(Kind.NEUTRAL, 0, s.neutralS, i));
            }
        }
        if (s.includeHolds) {
            double[] holdSigns = {1, -1};
            for (int i = 0; i < levels.length; i++) {
                for (double sign : holdSigns) {
                    list.add(new Step(Kind.HOLD, sign * levels[i], s.holdS, i));
                    list.add(new Step(Kind.NEUTRAL, 0, s.neutralS, i));
                }
            }
        }
        return new OpenLoopSchedule(list, s.neutralS);
    }

    /** Starts, or resumes after {@link #pause()}. */
    public void start(long nanos) {
        if (complete) return;
        // A motion step never restarts mid-way or straight from a stop: neutral first.
        resumeNeutral = steps.get(index).kind != Kind.NEUTRAL;
        running = true;
        stepStartNanos = nanos;
    }

    /** Aborts the current step; the next {@link #start} repeats it after a neutral interval. */
    public void pause() {
        running = false;
    }

    /** Advances time and returns the command for this loop (0 when neutral, paused or complete). */
    public double update(long nanos) {
        if (!running) return 0;
        double elapsed = (nanos - stepStartNanos) * 1e-9;
        if (resumeNeutral) {
            if (elapsed < neutralS) return 0;
            resumeNeutral = false;
            stepStartNanos += (long) (neutralS * 1e9);
            elapsed = (nanos - stepStartNanos) * 1e-9;
        }
        while (elapsed >= steps.get(index).durationS) {
            stepStartNanos += (long) (steps.get(index).durationS * 1e9);
            elapsed = (nanos - stepStartNanos) * 1e-9;
            index++;
            if (index >= steps.size()) {
                index = steps.size() - 1;
                complete = true;
                running = false;
                return 0;
            }
        }
        return steps.get(index).command;
    }

    public boolean isRunning() { return running; }
    public boolean isComplete() { return complete; }
    public int index() { return index; }
    public int size() { return steps.size(); }
    public List<Step> steps() { return steps; }

    /** Kind in effect now: NEUTRAL during a resume interval or when not running. */
    public Kind currentKind() {
        if (!running || resumeNeutral || complete) return Kind.NEUTRAL;
        return steps.get(index).kind;
    }

    /** Total planned time from the start, in seconds (for the operator display). */
    public double totalDurationS() {
        double total = 0;
        for (Step step : steps) total += step.durationS;
        return total;
    }
}
