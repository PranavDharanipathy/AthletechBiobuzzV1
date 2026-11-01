package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Immutable snapshot of the operator-confirmed diagnostic settings.
 *
 * <p>The safety-relevant fields have no usable default: they start as NaN, and every active test
 * is refused until the operator has entered and confirmed them. They are: encoder scale, travel
 * bounds, command limit, stopping margin, assumed deceleration and maximum speed. This class never
 * invents a safe angle range.
 *
 * <p>Pure Java, so it can be unit tested without the SDK.
 */
public final class TurretDiagnosticSettings {

    /** Documented theoretical scale: 8192 CPR quadrature, encoder geared 3:1 to the turret. */
    public static final double DOCUMENTED_TICKS_PER_DEGREE = 8192.0 * 3.0 / 360.0;
    /** Value in baseline {@code ShooterConstants} (last season's). Shown, never changed here. */
    public static final double BASELINE_TICKS_PER_DEGREE = 65.738;

    // Hard ceilings that keep a Dashboard typo from producing a long or violent test.
    public static final double COMMAND_CEILING = 1.0;
    public static final double SOFT_COMMAND_WARNING = 0.5;
    public static final double MAX_PULSE_S = 1.0;
    public static final double MAX_HOLD_S = 3.0;
    public static final double MIN_NEUTRAL_S = 0.3;
    public static final double MAX_NEUTRAL_S = 10.0;
    public static final double MAX_DWELL_S = 30.0;
    public static final double MAX_EXTRA_DELAY_MS = 80.0;

    public enum Actuators { BOTH, LEFT_ONLY, RIGHT_ONLY }

    public enum GainSource {
        /** {@code TurretConstants.TURRET_POSITIONAL_COEFFICIENTS}, as the TeleOp base uses them. */
        CONSTANTS,
        /** The Dashboard-editable statics of {@code TurretBaseTuner}, in tuning mode. */
        TUNER
    }

    // Operator-confirmed safety inputs (required; NaN = not set).
    public final double ticksPerDegree;
    public final double minDeg;
    public final double maxDeg;
    public final double maxCommand;
    public final double stopMarginDeg;
    public final double decelDegPerS2;
    public final double maxSpeedDegPerS;

    // Fault-detection thresholds (detection heuristics, not physical safe ranges).
    public final double maxLoopGapMs;
    public final double implausibleSpeedDegPerS;
    /** NaN = use half of {@link #maxCommand}. */
    public final double frozenCheckCommand;
    public final double frozenTimeoutS;
    public final double opposeWindowS;
    public final double opposeMinDispDeg;
    public final double motionThresholdDeg;

    // Open-loop characterization.
    public final double[] openLoopLevels;
    public final double pulseS;
    public final double holdS;
    public final double neutralS;
    public final boolean includeHolds;
    public final Actuators actuators;
    public final double jogCommand;

    // Existing-controller test.
    public final double[] controllerTargetsDeg;
    public final double controllerDwellS;
    public final double extraLoopDelayMs;
    public final double extraLoopJitterMs;
    public final GainSource gainSource;

    public final int recordCapacityRows;

    /** Mutable field bag used to build a snapshot (from the Dashboard config, or in tests). */
    public static final class Builder {
        public double ticksPerDegree = Double.NaN;
        public double minDeg = Double.NaN;
        public double maxDeg = Double.NaN;
        public double maxCommand = Double.NaN;
        public double stopMarginDeg = Double.NaN;
        public double decelDegPerS2 = Double.NaN;
        public double maxSpeedDegPerS = Double.NaN;

        public double maxLoopGapMs = 100;
        public double implausibleSpeedDegPerS = 1500;
        public double frozenCheckCommand = Double.NaN;
        public double frozenTimeoutS = 0.5;
        public double opposeWindowS = 0.3;
        public double opposeMinDispDeg = 2.0;
        public double motionThresholdDeg = 0.1;

        public double[] openLoopLevels = {0.04, 0.06, 0.08, 0.10, 0.12};
        public double pulseS = 0.15;
        public double holdS = 0.6;
        public double neutralS = 0.8;
        public boolean includeHolds = true;
        public Actuators actuators = Actuators.BOTH;
        public double jogCommand = 0.05;

        public double[] controllerTargetsDeg = {5, 0, -5, 0};
        public double controllerDwellS = 2.0;
        public double extraLoopDelayMs = 0;
        public double extraLoopJitterMs = 0;
        public GainSource gainSource = GainSource.CONSTANTS;

        public int recordCapacityRows = 20000;

        public TurretDiagnosticSettings build() {
            return new TurretDiagnosticSettings(this);
        }
    }

    private TurretDiagnosticSettings(Builder b) {
        ticksPerDegree = b.ticksPerDegree;
        minDeg = b.minDeg;
        maxDeg = b.maxDeg;
        maxCommand = b.maxCommand;
        stopMarginDeg = b.stopMarginDeg;
        decelDegPerS2 = b.decelDegPerS2;
        maxSpeedDegPerS = b.maxSpeedDegPerS;
        maxLoopGapMs = b.maxLoopGapMs;
        implausibleSpeedDegPerS = b.implausibleSpeedDegPerS;
        frozenCheckCommand = b.frozenCheckCommand;
        frozenTimeoutS = b.frozenTimeoutS;
        opposeWindowS = b.opposeWindowS;
        opposeMinDispDeg = b.opposeMinDispDeg;
        motionThresholdDeg = b.motionThresholdDeg;
        openLoopLevels = b.openLoopLevels == null ? new double[0] : b.openLoopLevels.clone();
        pulseS = b.pulseS;
        holdS = b.holdS;
        neutralS = b.neutralS;
        includeHolds = b.includeHolds;
        actuators = b.actuators == null ? Actuators.BOTH : b.actuators;
        jogCommand = b.jogCommand;
        controllerTargetsDeg = b.controllerTargetsDeg == null ? new double[0] : b.controllerTargetsDeg.clone();
        controllerDwellS = b.controllerDwellS;
        extraLoopDelayMs = b.extraLoopDelayMs;
        extraLoopJitterMs = b.extraLoopJitterMs;
        gainSource = b.gainSource == null ? GainSource.CONSTANTS : b.gainSource;
        recordCapacityRows = b.recordCapacityRows;
    }

    /** Command level above which a non-moving encoder counts as frozen. */
    public double effectiveFrozenCheckCommand() {
        return Double.isNaN(frozenCheckCommand) ? 0.5 * maxCommand : frozenCheckCommand;
    }

    /** Lowest position the turret may be predicted to stop at. */
    public double safeMinDeg() {
        return minDeg + stopMarginDeg;
    }

    /** Highest position the turret may be predicted to stop at. */
    public double safeMaxDeg() {
        return maxDeg - stopMarginDeg;
    }

    /** Problems that refuse arming for the given activity. Empty list = may arm. */
    public List<String> problems(DiagnosticActivity activity) {
        List<String> p = new ArrayList<>();
        if (activity == DiagnosticActivity.PASSIVE) return p;

        if (!positive(ticksPerDegree)) p.add("TICKS_PER_DEGREE not set (confirm the encoder scale)");
        if (!finite(minDeg) || !finite(maxDeg)) {
            p.add("MIN_DEG / MAX_DEG not set (enter wiring-safe bounds you verified by hand)");
        } else if (!(minDeg < 0 && maxDeg > 0)) {
            p.add("bounds must contain the starting reference: MIN_DEG < 0 < MAX_DEG");
        }
        if (!finite(stopMarginDeg) || stopMarginDeg < 0) {
            p.add("STOP_MARGIN_DEG not set (>= 0)");
        } else if (finite(minDeg) && finite(maxDeg) && !(safeMinDeg() < 0 && safeMaxDeg() > 0)) {
            p.add("stopping margin leaves no room around the reference (need MIN+margin < 0 < MAX-margin)");
        }
        if (!positive(maxCommand) || maxCommand > COMMAND_CEILING) {
            p.add("MAX_COMMAND not set (0 < x <= " + COMMAND_CEILING + ")");
        }
        if (!positive(decelDegPerS2)) p.add("ASSUMED_DECEL_DEG_PER_S2 not set (use a low, conservative value)");
        if (!positive(maxSpeedDegPerS)) p.add("MAX_SPEED_DEG_PER_S not set");
        if (!finite(maxLoopGapMs) || maxLoopGapMs < 20 || maxLoopGapMs > 500) {
            p.add("MAX_LOOP_GAP_MS must be within [20, 500]");
        }
        if (!positive(implausibleSpeedDegPerS) || (positive(maxSpeedDegPerS) && implausibleSpeedDegPerS <= maxSpeedDegPerS)) {
            p.add("IMPLAUSIBLE_SPEED_DEG_PER_S must exceed MAX_SPEED_DEG_PER_S");
        }
        if (!Double.isNaN(frozenCheckCommand) && !(frozenCheckCommand > 0 && (!positive(maxCommand) || frozenCheckCommand <= maxCommand))) {
            p.add("FROZEN_CHECK_COMMAND must be in (0, MAX_COMMAND] or NaN");
        }
        if (!positive(frozenTimeoutS) || frozenTimeoutS > 2) p.add("FROZEN_TIMEOUT_S must be in (0, 2]");
        if (!positive(opposeWindowS) || !positive(opposeMinDispDeg)) p.add("oppose-check window/displacement must be > 0");
        if (recordCapacityRows < 1000 || recordCapacityRows > 100000) p.add("RECORD_CAPACITY_ROWS must be in [1000, 100000]");

        if (activity == DiagnosticActivity.OPEN_LOOP) {
            if (openLoopLevels.length == 0) p.add("OPEN_LOOP_LEVELS is empty");
            for (double level : openLoopLevels) {
                if (!positive(level) || (positive(maxCommand) && level > maxCommand)) {
                    p.add(String.format(Locale.US, "open-loop level %.3f not in (0, MAX_COMMAND]", level));
                }
            }
            if (!positive(pulseS) || pulseS > MAX_PULSE_S) p.add("PULSE_S must be in (0, " + MAX_PULSE_S + "]");
            if (includeHolds && (!positive(holdS) || holdS > MAX_HOLD_S)) p.add("HOLD_S must be in (0, " + MAX_HOLD_S + "]");
            if (!finite(neutralS) || neutralS < MIN_NEUTRAL_S || neutralS > MAX_NEUTRAL_S) {
                p.add("NEUTRAL_S must be in [" + MIN_NEUTRAL_S + ", " + MAX_NEUTRAL_S + "]");
            }
            if (!positive(jogCommand) || (positive(maxCommand) && jogCommand > maxCommand)) {
                p.add("JOG_COMMAND must be in (0, MAX_COMMAND]");
            }
        }

        if (activity == DiagnosticActivity.CONTROLLER) {
            if (controllerTargetsDeg.length == 0) p.add("CONTROLLER_TARGETS_DEG is empty");
            for (double target : controllerTargetsDeg) {
                if (!finite(target) || (finite(minDeg) && finite(maxDeg) && finite(stopMarginDeg)
                        && (target < safeMinDeg() || target > safeMaxDeg()))) {
                    p.add(String.format(Locale.US, "controller target %.2f deg outside [MIN+margin, MAX-margin]", target));
                }
            }
            if (!positive(controllerDwellS) || controllerDwellS > MAX_DWELL_S) p.add("CONTROLLER_DWELL_S must be in (0, " + MAX_DWELL_S + "]");
            if (!finite(extraLoopDelayMs) || extraLoopDelayMs < 0 || extraLoopDelayMs > MAX_EXTRA_DELAY_MS
                    || !finite(extraLoopJitterMs) || extraLoopJitterMs < 0 || extraLoopJitterMs > MAX_EXTRA_DELAY_MS) {
                p.add("EXTRA_LOOP_DELAY_MS / JITTER_MS must be in [0, " + MAX_EXTRA_DELAY_MS + "]");
            } else if (finite(maxLoopGapMs) && extraLoopDelayMs + extraLoopJitterMs > maxLoopGapMs / 2) {
                p.add("extra loop delay + jitter must be <= MAX_LOOP_GAP_MS / 2, so stale-feedback detection still works");
            }
        }
        return p;
    }

    /** Non-blocking cautions shown to the operator. */
    public List<String> warnings() {
        List<String> w = new ArrayList<>();
        if (positive(ticksPerDegree)
                && Math.abs(ticksPerDegree / DOCUMENTED_TICKS_PER_DEGREE - 1) > 0.05
                && Math.abs(ticksPerDegree / BASELINE_TICKS_PER_DEGREE - 1) > 0.05) {
            w.add("scale matches neither documented 68.2667 nor baseline 65.738 (+-5%)");
        }
        if (positive(maxCommand) && maxCommand > SOFT_COMMAND_WARNING) {
            w.add("MAX_COMMAND above " + SOFT_COMMAND_WARNING + ": not conservative");
        }
        return w;
    }

    /** NaN-safe field-by-field comparison, used to detect edits after confirmation. */
    public boolean sameAs(TurretDiagnosticSettings o) {
        return o != null
                && same(ticksPerDegree, o.ticksPerDegree) && same(minDeg, o.minDeg) && same(maxDeg, o.maxDeg)
                && same(maxCommand, o.maxCommand) && same(stopMarginDeg, o.stopMarginDeg)
                && same(decelDegPerS2, o.decelDegPerS2) && same(maxSpeedDegPerS, o.maxSpeedDegPerS)
                && same(maxLoopGapMs, o.maxLoopGapMs) && same(implausibleSpeedDegPerS, o.implausibleSpeedDegPerS)
                && same(frozenCheckCommand, o.frozenCheckCommand) && same(frozenTimeoutS, o.frozenTimeoutS)
                && same(opposeWindowS, o.opposeWindowS) && same(opposeMinDispDeg, o.opposeMinDispDeg)
                && same(motionThresholdDeg, o.motionThresholdDeg)
                && Arrays.equals(openLoopLevels, o.openLoopLevels)
                && same(pulseS, o.pulseS) && same(holdS, o.holdS) && same(neutralS, o.neutralS)
                && includeHolds == o.includeHolds && actuators == o.actuators && same(jogCommand, o.jogCommand)
                && Arrays.equals(controllerTargetsDeg, o.controllerTargetsDeg)
                && same(controllerDwellS, o.controllerDwellS) && same(extraLoopDelayMs, o.extraLoopDelayMs)
                && same(extraLoopJitterMs, o.extraLoopJitterMs) && gainSource == o.gainSource
                && recordCapacityRows == o.recordCapacityRows;
    }

    /** One "# key=value" line per setting, for recording headers. */
    public List<String> describe() {
        List<String> d = new ArrayList<>();
        d.add("ticks_per_degree=" + ticksPerDegree);
        d.add("min_deg=" + minDeg);
        d.add("max_deg=" + maxDeg);
        d.add("stop_margin_deg=" + stopMarginDeg);
        d.add("max_command=" + maxCommand);
        d.add("assumed_decel_deg_per_s2=" + decelDegPerS2);
        d.add("max_speed_deg_per_s=" + maxSpeedDegPerS);
        d.add("max_loop_gap_ms=" + maxLoopGapMs);
        d.add("implausible_speed_deg_per_s=" + implausibleSpeedDegPerS);
        d.add("frozen_check_command=" + effectiveFrozenCheckCommand());
        d.add("frozen_timeout_s=" + frozenTimeoutS);
        d.add("oppose_window_s=" + opposeWindowS);
        d.add("oppose_min_disp_deg=" + opposeMinDispDeg);
        d.add("motion_threshold_deg=" + motionThresholdDeg);
        d.add("open_loop_levels=" + Arrays.toString(openLoopLevels));
        d.add("pulse_s=" + pulseS);
        d.add("hold_s=" + holdS);
        d.add("neutral_s=" + neutralS);
        d.add("include_holds=" + includeHolds);
        d.add("actuators=" + actuators);
        d.add("jog_command=" + jogCommand);
        d.add("controller_targets_deg=" + Arrays.toString(controllerTargetsDeg));
        d.add("controller_dwell_s=" + controllerDwellS);
        d.add("extra_loop_delay_ms=" + extraLoopDelayMs);
        d.add("extra_loop_jitter_ms=" + extraLoopJitterMs);
        d.add("gain_source=" + gainSource);
        d.add("record_capacity_rows=" + recordCapacityRows);
        return d;
    }

    private static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    private static boolean positive(double v) {
        return finite(v) && v > 0;
    }

    private static boolean same(double a, double b) {
        return Double.compare(a, b) == 0;
    }
}
