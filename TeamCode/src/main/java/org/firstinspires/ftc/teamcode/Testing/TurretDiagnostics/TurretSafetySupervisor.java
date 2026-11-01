package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import java.util.List;
import java.util.Locale;

/**
 * Decides whether the turret servos may receive a non-zero command, every loop.
 *
 * <p>Arming requires all of these:
 * <ul>
 *   <li>operator-confirmed settings, unchanged since confirmation;</li>
 *   <li>a confirmed starting reference;</li>
 *   <li>a fresh encoder sample;</li>
 *   <li>a still turret inside the safe zone;</li>
 *   <li>the deadman released at the moment of arming.</li>
 * </ul>
 *
 * <p>While armed, {@link #check} disarms on any of these:
 * <ul>
 *   <li>deadman release;</li>
 *   <li>a failed or stale encoder read;</li>
 *   <li>an implausible encoder jump;</li>
 *   <li>a position outside the bounds;</li>
 *   <li>a predicted stop beyond the margin;</li>
 *   <li>overspeed;</li>
 *   <li>a frozen encoder under command;</li>
 *   <li>a command that keeps opposing motion.</li>
 * </ul>
 *
 * <p><b>Limits of this protection.</b> The predicted stopping point uses an operator-supplied
 * deceleration and the observed loop latency. If the real deceleration is lower, or the encoder
 * misreports motion, the turret can pass the software bound. The bounds are operator-entered and do
 * not guarantee wiring safety. Keep a human ready to disable the robot.
 *
 * <p>Pure Java: no SDK types, so every stop path is unit tested.
 */
public final class TurretSafetySupervisor {

    private static final int DT_HISTORY = 16;
    /** Turret speed above which arming is refused (someone is moving it). */
    static final double STILL_SPEED_DEG_PER_S = 5.0;
    /** Velocity magnitude below which the command-opposes-motion check ignores the sample. */
    static final double OPPOSE_MIN_SPEED_DEG_PER_S = 5.0;

    private TurretDiagnosticSettings settings;
    private boolean referenceSet;
    private int referenceTicks;

    /** Arming expires if the deadman is not pressed within this time. */
    static final double ARM_WAIT_TIMEOUT_S = 10.0;

    private boolean armed;
    /** Armed and the deadman has been pressed since arming: commands may flow. */
    private boolean engaged;
    private long armedAtNanos;
    private DiagnosticActivity armedActivity = DiagnosticActivity.PASSIVE;
    private StopReason lastStop = StopReason.NONE;
    private String lastStopDetail = "";

    private boolean hasSample;
    private int lastTicks;
    private long lastNanos;
    private boolean ticksChanged;
    private double lastDtS = Double.NaN;
    private double velTicksPerS;
    private boolean readFailed;
    private final double[] dtHistory = new double[DT_HISTORY];
    private int dtCount;
    private int dtIndex;

    private double frozenTimeS;
    private double opposeTimeS;
    private double opposeDispDeg;

    // ---------------------------------------------------------------- confirmations

    /** Snapshot of settings the operator reviewed. Only while disarmed. */
    public boolean confirmSettings(TurretDiagnosticSettings confirmed) {
        if (armed) return false;
        settings = confirmed;
        return true;
    }

    public void clearSettings() {
        if (armed) disarm(StopReason.SETTINGS_CHANGED, "confirmation cleared");
        settings = null;
    }

    /** The turret is physically at the operator's starting reference now. Only while disarmed. */
    public boolean confirmReference(int ticks) {
        if (armed) return false;
        referenceTicks = ticks;
        referenceSet = true;
        return true;
    }

    public TurretDiagnosticSettings settings() { return settings; }
    public boolean isReferenceSet() { return referenceSet; }
    public int referenceTicks() { return referenceTicks; }

    // ---------------------------------------------------------------- feedback

    /** Records one encoder sample. Call every loop, armed or not. */
    public void observe(long nanos, int ticks) {
        readFailed = false;
        if (hasSample) {
            double dt = (nanos - lastNanos) * 1e-9;
            lastDtS = dt;
            dtHistory[dtIndex] = dt;
            dtIndex = (dtIndex + 1) % DT_HISTORY;
            if (dtCount < DT_HISTORY) dtCount++;
            ticksChanged = ticks != lastTicks;
            velTicksPerS = dt > 0 ? (ticks - lastTicks) / dt : 0;
        } else {
            ticksChanged = false;
            velTicksPerS = 0;
        }
        hasSample = true;
        lastTicks = ticks;
        lastNanos = nanos;
    }

    /** The encoder read threw or returned garbage this loop. */
    public void markReadFailed() {
        readFailed = true;
    }

    // ---------------------------------------------------------------- arming

    /** @return null if arming is allowed, otherwise the reason it is refused. */
    public String armRefusal(DiagnosticActivity activity, TurretDiagnosticSettings current,
                             boolean deadmanHeld, long nanos) {
        if (activity == DiagnosticActivity.PASSIVE) return "PASSIVE never arms; select CONTROLLER or OPEN_LOOP";
        if (armed) return "already armed";
        if (settings == null) return "settings not confirmed (LB + A)";
        if (!settings.sameAs(current)) return "settings edited since confirmation: re-confirm (LB + A)";
        List<String> problems = settings.problems(activity);
        if (!problems.isEmpty()) return problems.get(0);
        if (!referenceSet) return "starting reference not confirmed (LB + X)";
        if (deadmanHeld) return "release the deadman before arming";
        if (!hasSample || readFailed) return "no valid encoder sample";
        if ((nanos - lastNanos) * 1e-6 > settings.maxLoopGapMs) return "encoder sample is stale";
        double pos = positionDeg();
        if (pos < settings.safeMinDeg() || pos > settings.safeMaxDeg()) {
            return String.format(Locale.US, "turret at %.2f deg is outside the safe zone [%.2f, %.2f]",
                    pos, settings.safeMinDeg(), settings.safeMaxDeg());
        }
        if (Math.abs(velocityDegPerS()) > STILL_SPEED_DEG_PER_S) return "turret is moving; wait until still";
        return null;
    }

    /** Arms if {@link #armRefusal} allows it. Re-arming acknowledges the last fault. */
    public boolean arm(DiagnosticActivity activity, TurretDiagnosticSettings current,
                       boolean deadmanHeld, long nanos) {
        if (armRefusal(activity, current, deadmanHeld, nanos) != null) return false;
        armed = true;
        engaged = false;
        armedAtNanos = nanos;
        armedActivity = activity;
        lastStop = StopReason.NONE;
        lastStopDetail = "";
        frozenTimeS = 0;
        opposeTimeS = 0;
        opposeDispDeg = 0;
        return true;
    }

    /** Ends command authority. Records the reason even when already disarmed, if it is a fault. */
    public void disarm(StopReason reason, String detail) {
        if (!armed && !reason.fault) return;
        armed = false;
        engaged = false;
        armedActivity = DiagnosticActivity.PASSIVE;
        lastStop = reason;
        lastStopDetail = detail == null ? "" : detail;
        frozenTimeS = 0;
        opposeTimeS = 0;
        opposeDispDeg = 0;
    }

    // ---------------------------------------------------------------- per-loop check

    /**
     * Evaluates every stop condition. Call after {@link #observe} and before computing a new command.
     *
     * @param commandSinceLastSample the command that was applied while the motion just observed happened
     * @return {@link StopReason#NONE} if command authority continues; otherwise the reason (already disarmed)
     */
    public StopReason check(double commandSinceLastSample, boolean deadmanHeld, long nanos) {
        if (!armed) return StopReason.NONE;
        TurretDiagnosticSettings s = settings;

        if (readFailed || !hasSample) return trip(StopReason.FEEDBACK_READ_FAILED, "");
        if (!engaged) {
            // Armed, waiting for the first deadman press. No command flows in this state.
            if (!deadmanHeld) {
                if ((nanos - armedAtNanos) * 1e-9 > ARM_WAIT_TIMEOUT_S) return trip(StopReason.ARM_TIMEOUT, "");
                return StopReason.NONE;
            }
            engaged = true;
        }
        if (!deadmanHeld) return trip(StopReason.DEADMAN_RELEASED, "");

        double sinceSampleMs = (nanos - lastNanos) * 1e-6;
        if (sinceSampleMs > s.maxLoopGapMs) {
            return trip(StopReason.STALE_FEEDBACK, String.format(Locale.US, "%.1f ms since sample", sinceSampleMs));
        }
        if (!Double.isNaN(lastDtS) && lastDtS * 1e3 > s.maxLoopGapMs) {
            return trip(StopReason.STALE_FEEDBACK, String.format(Locale.US, "loop gap %.1f ms", lastDtS * 1e3));
        }

        double vel = velocityDegPerS();
        if (Math.abs(vel) > s.implausibleSpeedDegPerS) {
            return trip(StopReason.IMPLAUSIBLE_JUMP, String.format(Locale.US, "%.0f deg/s", vel));
        }

        double pos = positionDeg();
        if (pos < s.minDeg || pos > s.maxDeg) {
            return trip(StopReason.OUTSIDE_BOUNDS, String.format(Locale.US, "%.2f deg", pos));
        }
        if (Math.abs(vel) > s.maxSpeedDegPerS) {
            return trip(StopReason.OVERSPEED, String.format(Locale.US, "%.1f deg/s", vel));
        }
        double stop = predictedStopDeg();
        if ((vel > 0 && stop > s.safeMaxDeg()) || (vel < 0 && stop < s.safeMinDeg())) {
            return trip(StopReason.PREDICTED_STOP_BEYOND_MARGIN,
                    String.format(Locale.US, "pos %.2f, v %.1f, predicted stop %.2f", pos, vel, stop));
        }

        double dt = Double.isNaN(lastDtS) ? 0 : lastDtS;
        double checkCommand = s.effectiveFrozenCheckCommand();
        boolean commanding = Math.abs(commandSinceLastSample) >= checkCommand;

        frozenTimeS = commanding && !ticksChanged ? frozenTimeS + dt : 0;
        if (frozenTimeS >= s.frozenTimeoutS) {
            return trip(StopReason.ENCODER_FROZEN, String.format(Locale.US,
                    "|cmd| >= %.3f for %.2f s with no encoder change", checkCommand, frozenTimeS));
        }

        if (commanding && vel * commandSinceLastSample < 0 && Math.abs(vel) >= OPPOSE_MIN_SPEED_DEG_PER_S) {
            opposeTimeS += dt;
            opposeDispDeg += Math.abs(vel) * dt;
        } else {
            opposeTimeS = 0;
            opposeDispDeg = 0;
        }
        if (opposeTimeS >= s.opposeWindowS && opposeDispDeg >= s.opposeMinDispDeg) {
            return trip(StopReason.COMMAND_OPPOSES_MOTION, String.format(Locale.US,
                    "%.2f s, %.2f deg against the command", opposeTimeS, opposeDispDeg));
        }
        return StopReason.NONE;
    }

    private StopReason trip(StopReason reason, String detail) {
        disarm(reason, detail);
        return reason;
    }

    // ---------------------------------------------------------------- state for telemetry/recording

    public boolean isArmed() { return armed; }
    /** Armed and deadman engaged: the only state in which a non-zero command may be written. */
    public boolean isRunning() { return armed && engaged; }
    public DiagnosticActivity armedActivity() { return armedActivity; }
    public StopReason lastStop() { return lastStop; }
    public String lastStopDetail() { return lastStopDetail; }
    public boolean hasSample() { return hasSample; }
    public int lastTicks() { return lastTicks; }
    public double lastDtMs() { return lastDtS * 1e3; }

    /** Worst loop interval over the recent history, in seconds (0 before two samples). */
    public double maxRecentDtS() {
        double max = 0;
        for (int i = 0; i < dtCount; i++) max = Math.max(max, dtHistory[i]);
        return max;
    }

    /** Degrees from the confirmed reference using the confirmed scale; NaN until both exist. */
    public double positionDeg() {
        if (settings == null || !referenceSet || !hasSample || !(settings.ticksPerDegree > 0)) return Double.NaN;
        return (lastTicks - referenceTicks) / settings.ticksPerDegree;
    }

    /** Differenced velocity (no filter, no added delay), deg/s; NaN without a confirmed scale. */
    public double velocityDegPerS() {
        if (settings == null || !(settings.ticksPerDegree > 0)) return Double.NaN;
        return velTicksPerS / settings.ticksPerDegree;
    }

    /**
     * Where the turret would stop if the servos went neutral now. Reaction latency is taken as two
     * of the worst recent loop intervals (one to notice, one for the write to take effect), plus
     * braking at the operator's assumed deceleration.
     */
    public double predictedStopDeg() {
        double pos = positionDeg();
        double vel = velocityDegPerS();
        if (Double.isNaN(pos) || Double.isNaN(vel) || settings == null) return Double.NaN;
        double latency = 2 * maxRecentDtS();
        double distance = Math.abs(vel) * latency + vel * vel / (2 * settings.decelDegPerS2);
        return pos + Math.signum(vel) * distance;
    }
}
