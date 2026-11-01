package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;

/**
 * Dashboard-editable inputs for {@link TurretDiagnosticOpMode}.
 *
 * <p><b>Required before any active test</b> (NaN = not set, and arming is refused):
 * <ul>
 *   <li>{@code TICKS_PER_DEGREE}: the scale you confirm. Documented theory is 68.2667; baseline
 *       {@code ShooterConstants} still has 65.738. Use the passive marker check to measure it.
 *       This value is used only by the diagnostic and never written back.</li>
 *   <li>{@code MIN_DEG} / {@code MAX_DEG}: wiring-safe travel from the starting reference. Verify
 *       it by hand with the robot disabled. There is no hard stop, so nothing else limits travel.
 *       The tool will not guess this range.</li>
 *   <li>{@code STOP_MARGIN_DEG}: room kept between the predicted stopping point and the bound.</li>
 *   <li>{@code MAX_COMMAND}: hard clamp on every servo command (start low, e.g. 0.1-0.2).</li>
 *   <li>{@code ASSUMED_DECEL_DEG_PER_S2}: braking deceleration used to predict the stopping point.
 *       Lower is more conservative. Replace it with measured coast decel from open-loop runs.</li>
 *   <li>{@code MAX_SPEED_DEG_PER_S}: overspeed trip.</li>
 * </ul>
 * After editing, press LB + A on gamepad 1 to confirm. Any later edit cancels the confirmation
 * and stops an active test.
 */
@Config
public class TurretDiagnosticConfig {

    public static double TICKS_PER_DEGREE = Double.NaN;
    public static double MIN_DEG = Double.NaN;
    public static double MAX_DEG = Double.NaN;
    public static double STOP_MARGIN_DEG = Double.NaN;
    public static double MAX_COMMAND = Double.NaN;
    public static double ASSUMED_DECEL_DEG_PER_S2 = Double.NaN;
    public static double MAX_SPEED_DEG_PER_S = Double.NaN;

    public static double MAX_LOOP_GAP_MS = 100;
    public static double IMPLAUSIBLE_SPEED_DEG_PER_S = 1500;
    /** NaN = half of MAX_COMMAND. */
    public static double FROZEN_CHECK_COMMAND = Double.NaN;
    public static double FROZEN_TIMEOUT_S = 0.5;
    public static double OPPOSE_WINDOW_S = 0.3;
    public static double OPPOSE_MIN_DISP_DEG = 2.0;
    public static double MOTION_THRESHOLD_DEG = 0.1;

    public static double[] OPEN_LOOP_LEVELS = {0.04, 0.06, 0.08, 0.10, 0.12};
    public static double PULSE_S = 0.15;
    public static double HOLD_S = 0.6;
    public static double NEUTRAL_S = 0.8;
    public static boolean INCLUDE_HOLDS = true;
    public static TurretDiagnosticSettings.Actuators ACTUATORS = TurretDiagnosticSettings.Actuators.BOTH;
    public static double JOG_COMMAND = 0.05;

    public static double[] CONTROLLER_TARGETS_DEG = {5, 0, -5, 0};
    public static double CONTROLLER_DWELL_S = 2.0;
    public static double EXTRA_LOOP_DELAY_MS = 0;
    public static double EXTRA_LOOP_JITTER_MS = 0;
    public static TurretDiagnosticSettings.GainSource GAIN_SOURCE = TurretDiagnosticSettings.GainSource.CONSTANTS;

    public static int RECORD_CAPACITY_ROWS = 20000;

    private TurretDiagnosticConfig() {}

    /** Copies the current Dashboard values into an immutable snapshot. */
    public static TurretDiagnosticSettings snapshot() {
        TurretDiagnosticSettings.Builder b = new TurretDiagnosticSettings.Builder();
        b.ticksPerDegree = TICKS_PER_DEGREE;
        b.minDeg = MIN_DEG;
        b.maxDeg = MAX_DEG;
        b.stopMarginDeg = STOP_MARGIN_DEG;
        b.maxCommand = MAX_COMMAND;
        b.decelDegPerS2 = ASSUMED_DECEL_DEG_PER_S2;
        b.maxSpeedDegPerS = MAX_SPEED_DEG_PER_S;
        b.maxLoopGapMs = MAX_LOOP_GAP_MS;
        b.implausibleSpeedDegPerS = IMPLAUSIBLE_SPEED_DEG_PER_S;
        b.frozenCheckCommand = FROZEN_CHECK_COMMAND;
        b.frozenTimeoutS = FROZEN_TIMEOUT_S;
        b.opposeWindowS = OPPOSE_WINDOW_S;
        b.opposeMinDispDeg = OPPOSE_MIN_DISP_DEG;
        b.motionThresholdDeg = MOTION_THRESHOLD_DEG;
        b.openLoopLevels = OPEN_LOOP_LEVELS;
        b.pulseS = PULSE_S;
        b.holdS = HOLD_S;
        b.neutralS = NEUTRAL_S;
        b.includeHolds = INCLUDE_HOLDS;
        b.actuators = ACTUATORS;
        b.jogCommand = JOG_COMMAND;
        b.controllerTargetsDeg = CONTROLLER_TARGETS_DEG;
        b.controllerDwellS = CONTROLLER_DWELL_S;
        b.extraLoopDelayMs = EXTRA_LOOP_DELAY_MS;
        b.extraLoopJitterMs = EXTRA_LOOP_JITTER_MS;
        b.gainSource = GAIN_SOURCE;
        b.recordCapacityRows = RECORD_CAPACITY_ROWS;
        return b.build();
    }

    /** True if the live values still equal the confirmed snapshot. No allocation; safe every loop. */
    public static boolean matches(TurretDiagnosticSettings s) {
        return same(TICKS_PER_DEGREE, s.ticksPerDegree) && same(MIN_DEG, s.minDeg) && same(MAX_DEG, s.maxDeg)
                && same(STOP_MARGIN_DEG, s.stopMarginDeg) && same(MAX_COMMAND, s.maxCommand)
                && same(ASSUMED_DECEL_DEG_PER_S2, s.decelDegPerS2) && same(MAX_SPEED_DEG_PER_S, s.maxSpeedDegPerS)
                && same(MAX_LOOP_GAP_MS, s.maxLoopGapMs) && same(IMPLAUSIBLE_SPEED_DEG_PER_S, s.implausibleSpeedDegPerS)
                && same(FROZEN_CHECK_COMMAND, s.frozenCheckCommand) && same(FROZEN_TIMEOUT_S, s.frozenTimeoutS)
                && same(OPPOSE_WINDOW_S, s.opposeWindowS) && same(OPPOSE_MIN_DISP_DEG, s.opposeMinDispDeg)
                && same(MOTION_THRESHOLD_DEG, s.motionThresholdDeg)
                && Arrays.equals(OPEN_LOOP_LEVELS, s.openLoopLevels)
                && same(PULSE_S, s.pulseS) && same(HOLD_S, s.holdS) && same(NEUTRAL_S, s.neutralS)
                && INCLUDE_HOLDS == s.includeHolds && ACTUATORS == s.actuators && same(JOG_COMMAND, s.jogCommand)
                && Arrays.equals(CONTROLLER_TARGETS_DEG, s.controllerTargetsDeg)
                && same(CONTROLLER_DWELL_S, s.controllerDwellS) && same(EXTRA_LOOP_DELAY_MS, s.extraLoopDelayMs)
                && same(EXTRA_LOOP_JITTER_MS, s.extraLoopJitterMs) && GAIN_SOURCE == s.gainSource
                && RECORD_CAPACITY_ROWS == s.recordCapacityRows;
    }

    private static boolean same(double a, double b) {
        return Double.compare(a, b) == 0;
    }
}
