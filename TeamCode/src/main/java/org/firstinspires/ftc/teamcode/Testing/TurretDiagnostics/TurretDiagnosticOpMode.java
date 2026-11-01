package org.firstinspires.ftc.teamcode.Testing.TurretDiagnostics;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Constants.DeviceConstants;
import org.firstinspires.ftc.teamcode.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Constants.TurretConstants;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBase;
import org.firstinspires.ftc.teamcode.Systems.Turret.TurretBasePIDFCoefficients;
import org.firstinspires.ftc.teamcode.Tuners.TurretBaseTuner;
import org.firstinspires.ftc.teamcode.Utility.Encoder;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredCRServo;
import org.firstinspires.ftc.teamcode.Utility.Hardware.MonitoredDcMotorEx;
import org.firstinspires.ftc.teamcode.Utility.Monitoring.HardwareMonitor;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * Turret diagnostic and characterization tool. It collects data and diagnoses; it does not replace
 * the controller.
 *
 * <p><b>Modes</b> (selected while disarmed; every recording row and file is labeled):
 * <ul>
 *   <li><b>PASSIVE</b>: observe and record only. No servo write happens until the first active
 *       run. Use it to check the encoder scale against marks by hand, and to see disturbance with
 *       the servos off.</li>
 *   <li><b>CONTROLLER</b>: runs the baseline {@link TurretBase}, unmodified, through the command
 *       guard, and records its P, I, D, hold-feedforward terms, gains, own dt and output.
 *       Optional extra loop delay/jitter shows how sensitive it is to slow loops.</li>
 *   <li><b>OPEN_LOOP</b>: bounded pulse and hold schedule (per-step summary file), or an operator
 *       jog limited to JOG_COMMAND.</li>
 * </ul>
 *
 * <p><b>Safety</b> (see {@link TurretSafetySupervisor}). Starts in PASSIVE, and init never moves
 * anything. Arming needs every operator confirmation and is only possible after START. Commands
 * flow only while the deadman is held. Every stop condition writes neutral to both servos.
 *
 * <p>Software bounds are only as good as the bounds you enter, the assumed deceleration, and the
 * encoder. They do not guarantee wiring safety. Keep a hand on the stop button.
 *
 * <p><b>Gamepad 1</b>
 * <pre>
 *   RB (hold)        deadman: a run happens only while held; release = stop + disarm
 *   LB + Y           arm (after START, deadman released, all confirmations done)
 *   B or BACK        cancel: stop + disarm
 *   dpad left/right  select mode (disarmed)
 *   dpad up/down     select plan (disarmed): open loop schedule/jog, controller schedule/hold
 *   LB + A           confirm settings from the Dashboard (disarmed)
 *   LB + X           confirm starting reference = current position (disarmed)
 *   X                set scale-check marker (disarmed)
 *   left stick X     jog command (OPEN_LOOP jog plan, while running)
 * </pre>
 */
@TeleOp(name = "Turret Diagnostic", group = "diagnostics")
public class TurretDiagnosticOpMode extends OpMode {

    private static final String VERSION = "turret-diagnostic 1";
    private static final double BATTERY_SAMPLE_PERIOD_S = 0.05;
    private static final double JOG_STICK_DEADBAND = 0.1;

    private List<LynxModule> hubs;
    private GuardedCRServo leftServo;
    private GuardedCRServo rightServo;
    private MonitoredDcMotorEx encoderPort;
    private Encoder encoder;
    private VoltageSensor battery;
    private Telemetry out;

    private final TurretSafetySupervisor supervisor = new TurretSafetySupervisor();
    private DiagnosticRecorder recorder;

    private DiagnosticActivity selected = DiagnosticActivity.PASSIVE;
    /** 0 = schedule, 1 = jog. */
    private int openLoopPlan;
    /** 0 = target schedule, 1 = hold reference. */
    private int controllerPlan;

    private SegmentBuffer segment;
    private int segmentSequence;
    private int runId;
    private boolean wasRunning;

    private OpenLoopSchedule schedule;
    private ControllerTargetSchedule targetSchedule;
    private TurretBase controller;
    private double controllerTargetTicks = Double.NaN;

    private long startNanos;
    private long lastLoopNanos;
    private long lastBatteryNanos;
    private double batteryV = Double.NaN;
    private double lastAppliedCommand;
    private double lastExtraDelayMs;
    private StopReason pendingRowStop = StopReason.NONE;
    private final Random jitter = new Random(31666);

    private Integer markerTicks;
    private String message = "";
    private final boolean[] previous = new boolean[9];

    // ------------------------------------------------------------------ lifecycle

    @Override
    public void init() {
        out = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        startNanos = System.nanoTime();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Same devices and configured directions TurretBase uses. No power is written here.
        MonitoredCRServo left = new MonitoredCRServo(hardwareMap, DeviceConstants.leftTurretCRServoDeviceName, false);
        MonitoredCRServo right = new MonitoredCRServo(hardwareMap, DeviceConstants.rightTurretCRServoDeviceName, false);
        left.setDirection(TurretConstants.TURRET_BASE_DIRECTIONS[0]);
        right.setDirection(TurretConstants.TURRET_BASE_DIRECTIONS[1]);
        leftServo = new GuardedCRServo(left);
        rightServo = new GuardedCRServo(right);

        encoderPort = new MonitoredDcMotorEx(hardwareMap, DeviceConstants.turretEncoderDeviceName, false);
        encoder = new Encoder(encoderPort);
        encoder.setDirection(TurretConstants.TURRET_ENCODER_DIRECTION);

        battery = hardwareMap.voltageSensor.iterator().next();

        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        recorder = new DiagnosticRecorder(new File(new File(AppUtil.ROBOT_DATA_DIR, "TurretDiagnostics"), stamp));
        recorder.event(0, "init", DiagnosticActivity.PASSIVE, 0, VERSION);
        openSegment(DiagnosticActivity.PASSIVE);
        message = recorder.isUsable() ? "Passive. Nothing moves until you arm after START."
                : "RECORDING UNAVAILABLE: " + recorder.lastError();
    }

    @Override
    public void init_loop() {
        cycle(false);
    }

    @Override
    public void start() {
        recorder.event(elapsedS(System.nanoTime()), "start", DiagnosticActivity.PASSIVE, runId, "");
    }

    @Override
    public void loop() {
        cycle(true);
    }

    @Override
    public void stop() {
        stopServos();
        long now = System.nanoTime();
        if (supervisor.isArmed()) endRun(StopReason.OPMODE_STOP, "", now);
        recorder.event(elapsedS(now), "stop", DiagnosticActivity.PASSIVE, runId, "");
        submitSegment();
        recorder.shutdown(1500);
    }

    // ------------------------------------------------------------------ one loop

    private void cycle(boolean afterStart) {
        long now = System.nanoTime();
        double loopDtMs = lastLoopNanos == 0 ? Double.NaN : (now - lastLoopNanos) * 1e-6;
        lastLoopNanos = now;
        try {
            // 1. Feedback: one bulk read per loop, so every reader in this loop sees the same sample.
            long readStart = System.nanoTime();
            for (LynxModule hub : hubs) hub.clearBulkCache();
            HardwareMonitor.get().update();
            int ticks = 0;
            double hubVel = Double.NaN;
            boolean readOk;
            try {
                ticks = encoder.getCurrentPosition();
                hubVel = encoder.getInternalVelocity();
                readOk = true;
                supervisor.observe(readStart, ticks);
            } catch (RuntimeException e) {
                readOk = false;
                supervisor.markReadFailed();
                message = "Encoder read failed: " + e;
            }
            if (lastBatteryNanos == 0 || (now - lastBatteryNanos) * 1e-9 >= BATTERY_SAMPLE_PERIOD_S) {
                batteryV = battery.getVoltage();
                lastBatteryNanos = now;
            }
            long readEnd = System.nanoTime();

            // 2. Operator input.
            handleInput(afterStart, readOk, ticks, now);

            // 3. Settings edited after confirmation: stop and require re-confirmation.
            TurretDiagnosticSettings confirmed = supervisor.settings();
            if (confirmed != null && !TurretDiagnosticConfig.matches(confirmed)) {
                if (supervisor.isArmed()) endRun(StopReason.SETTINGS_CHANGED, "Dashboard edit", now);
                supervisor.clearSettings();
                schedule = null;
                recorder.event(elapsedS(now), "settings_unconfirmed", DiagnosticActivity.PASSIVE, runId, "Dashboard edit");
                message = "Settings changed: review and re-confirm (LB + A).";
            }

            // 4. Safety check against the motion produced by last loop's command.
            boolean deadman = gamepad1.right_bumper;
            if (supervisor.isArmed()) {
                StopReason reason = supervisor.check(lastAppliedCommand, deadman, now);
                if (reason != StopReason.NONE) endRun(reason, supervisor.lastStopDetail(), now);
            }

            // 5. Command.
            boolean running = supervisor.isRunning();
            if (running && !wasRunning) beginRun(now);
            wasRunning = running;
            double requested = 0;
            int stepIndex = -1;
            int stepKind = -1;
            if (running) {
                DiagnosticActivity activity = supervisor.armedActivity();
                if (activity == DiagnosticActivity.OPEN_LOOP) {
                    if (openLoopPlan == 1) {
                        double stick = gamepad1.left_stick_x;
                        double jog = Math.abs(stick) < JOG_STICK_DEADBAND ? 0 : stick;
                        requested = jog * supervisor.settings().jogCommand;
                        stepKind = OpenLoopSchedule.Kind.JOG.code;
                    } else {
                        requested = schedule.update(now);
                        stepIndex = schedule.index();
                        stepKind = schedule.currentKind().code;
                    }
                    applyOpenLoop(requested);
                } else {
                    double targetDeg = targetSchedule.update(now);
                    stepIndex = targetSchedule.phase();
                    if (!Double.isNaN(targetDeg)) {
                        TurretDiagnosticSettings s = supervisor.settings();
                        controllerTargetTicks = Math.round(supervisor.referenceTicks() + targetDeg * s.ticksPerDegree);
                        controller.setPosition(controllerTargetTicks);
                        controller.update(); // writes through both guards
                        requested = controller.getPower();
                    }
                }
                if (schedule != null && activity == DiagnosticActivity.OPEN_LOOP && openLoopPlan == 0 && schedule.isComplete()) {
                    endRun(StopReason.SCHEDULE_COMPLETE, "", now);
                } else if (activity == DiagnosticActivity.CONTROLLER && targetSchedule.isComplete()) {
                    endRun(StopReason.SCHEDULE_COMPLETE, "", now);
                }
            }
            if (!supervisor.isRunning() && (leftServo.isOpen() || rightServo.isOpen())) stopServos();
            lastAppliedCommand = Math.abs(leftServo.applied()) >= Math.abs(rightServo.applied())
                    ? leftServo.applied() : rightServo.applied();
            long writeEnd = System.nanoTime();

            // 6. Record.
            record(now, loopDtMs, ticks, readOk, hubVel, requested, stepIndex, stepKind,
                    readEnd - readStart, writeEnd - readEnd);

            // 7. Optional deliberate loop slowdown (controller runs only).
            lastExtraDelayMs = 0;
            if (supervisor.isRunning() && supervisor.armedActivity() == DiagnosticActivity.CONTROLLER) {
                TurretDiagnosticSettings s = supervisor.settings();
                double delay = s.extraLoopDelayMs + s.extraLoopJitterMs * jitter.nextDouble();
                if (delay > 0) {
                    lastExtraDelayMs = delay;
                    sleepMs(delay);
                }
            }

            showTelemetry(ticks, readOk, afterStart);
        } catch (RuntimeException e) {
            emergencyStop(e, now);
        }
    }

    // ------------------------------------------------------------------ input

    private void handleInput(boolean afterStart, boolean readOk, int ticks, long now) {
        boolean lb = gamepad1.left_bumper;
        boolean y = rising(0, gamepad1.y);
        boolean a = rising(1, gamepad1.a);
        boolean x = rising(2, gamepad1.x);
        boolean cancel = rising(3, gamepad1.b) | rising(4, gamepad1.back);
        boolean dl = rising(5, gamepad1.dpad_left);
        boolean dr = rising(6, gamepad1.dpad_right);
        boolean du = rising(7, gamepad1.dpad_up) | rising(8, gamepad1.dpad_down);

        if (cancel) {
            if (supervisor.isArmed()) endRun(StopReason.CANCELLED, "", now);
            stopServos();
            message = "Cancelled. Disarmed.";
        }
        if (supervisor.isArmed()) return; // nothing else changes while armed

        if (dl || dr) {
            selected = dl ? selected.previous() : selected.next();
            schedule = null;
            recorder.event(elapsedS(now), "mode_selected", selected, runId, "");
            message = "Mode " + selected.label;
        }
        if (du) {
            if (selected == DiagnosticActivity.OPEN_LOOP) { openLoopPlan = 1 - openLoopPlan; schedule = null; }
            if (selected == DiagnosticActivity.CONTROLLER) controllerPlan = 1 - controllerPlan;
            message = "Plan " + planName();
        }
        if (lb && a) {
            TurretDiagnosticSettings snap = TurretDiagnosticConfig.snapshot();
            supervisor.confirmSettings(snap);
            schedule = null;
            StringBuilder described = new StringBuilder();
            for (String line : snap.describe()) described.append(line).append("; "); // no String.join: API 26+
            recorder.event(elapsedS(now), "settings_confirmed", selected, runId, described.toString());
            message = "Settings confirmed.";
        }
        if (x) {
            if (!readOk) {
                message = "No encoder sample; cannot " + (lb ? "set reference" : "set marker");
            } else if (lb) {
                supervisor.confirmReference(ticks);
                recorder.event(elapsedS(now), "reference_confirmed", selected, runId, "ticks=" + ticks);
                message = "Starting reference set at " + ticks + " ticks.";
            } else {
                markerTicks = ticks;
                message = "Marker set. Rotate by a known angle and read the implied scale.";
            }
        }
        if (lb && y) {
            if (!afterStart) {
                message = "Arming is disabled during init. Press START first.";
            } else if (!recorder.isUsable()) {
                message = "Refused: recording unavailable.";
            } else {
                TurretDiagnosticSettings live = TurretDiagnosticConfig.snapshot();
                String refusal = supervisor.armRefusal(selected, live, gamepad1.right_bumper, now);
                if (refusal == null && selected == DiagnosticActivity.CONTROLLER
                        && live.gainSource == TurretDiagnosticSettings.GainSource.CONSTANTS && zeroBaselineGains()) {
                    message = "Armed. NOTE: baseline gains are all zero, so the controller will command 0.";
                }
                if (refusal == null && supervisor.arm(selected, live, gamepad1.right_bumper, now)) {
                    recorder.event(elapsedS(now), "armed", selected, runId + 1, planName());
                    if (!message.startsWith("Armed.")) message = "Armed (" + selected.label + "). Hold RB to run.";
                } else {
                    message = "Arm refused: " + refusal;
                    recorder.event(elapsedS(now), "arm_refused", selected, runId, refusal);
                }
            }
        }
    }

    private boolean rising(int slot, boolean pressed) {
        boolean edge = pressed && !previous[slot];
        previous[slot] = pressed;
        return edge;
    }

    // ------------------------------------------------------------------ runs

    private void beginRun(long now) {
        TurretDiagnosticSettings s = supervisor.settings();
        DiagnosticActivity activity = supervisor.armedActivity();
        runId++;
        submitSegment();
        openSegment(activity);
        if (activity == DiagnosticActivity.OPEN_LOOP) {
            if (openLoopPlan == 0) {
                if (schedule == null || schedule.isComplete()) schedule = OpenLoopSchedule.build(s);
                schedule.start(now);
            }
        } else {
            // A fresh baseline controller per run, so its timer and integral start clean.
            controller = new TurretBase(leftServo, rightServo, new Encoder(encoderPort), battery,
                    (double) supervisor.referenceTicks());
            applyGains(controller, s.gainSource);
            controllerTargetTicks = supervisor.lastTicks();
            controller.setPosition(controllerTargetTicks);
            targetSchedule = new ControllerTargetSchedule(s.controllerTargetsDeg, s.controllerDwellS, controllerPlan == 1);
            targetSchedule.start(now, supervisor.positionDeg());
        }
        leftServo.allow(s.maxCommand);
        rightServo.allow(s.maxCommand);
        recorder.event(elapsedS(now), "run_start", activity, runId, planName());
        message = "Running " + activity.label + " (" + planName() + "). Release RB to stop.";
    }

    /** Stops the servos, disarms with a reason, closes the active segment. */
    private void endRun(StopReason reason, String detail, long now) {
        stopServos();
        DiagnosticActivity activity = supervisor.armedActivity();
        boolean wasEngaged = supervisor.isRunning();
        supervisor.disarm(reason, detail);
        if (schedule != null) schedule.pause();
        if (targetSchedule != null) targetSchedule.stop();
        controller = null;
        controllerTargetTicks = Double.NaN;
        pendingRowStop = reason;
        recorder.event(elapsedS(now), reason.fault ? "fault" : "stop", activity, runId, reason.text + (detail.isEmpty() ? "" : ": " + detail));
        if (wasEngaged) {
            submitSegment();
            openSegment(DiagnosticActivity.PASSIVE);
        }
        wasRunning = false;
        message = (reason.fault ? "FAULT: " : "Stopped: ") + reason.text + (detail.isEmpty() ? "" : " (" + detail + ")");
    }

    private void applyOpenLoop(double command) {
        TurretDiagnosticSettings.Actuators which = supervisor.settings().actuators;
        leftServo.setPower(which == TurretDiagnosticSettings.Actuators.RIGHT_ONLY ? 0 : command);
        rightServo.setPower(which == TurretDiagnosticSettings.Actuators.LEFT_ONLY ? 0 : command);
    }

    private void stopServos() {
        try { leftServo.stop(); } catch (RuntimeException ignored) { }
        try { rightServo.stop(); } catch (RuntimeException ignored) { }
    }

    private void emergencyStop(RuntimeException e, long now) {
        stopServos();
        try {
            if (supervisor.isArmed()) endRun(StopReason.EXCEPTION, e.toString(), now);
            else supervisor.disarm(StopReason.EXCEPTION, e.toString());
            recorder.event(elapsedS(now), "exception", DiagnosticActivity.PASSIVE, runId, e.toString());
        } catch (RuntimeException ignored) {
            // Servos are already neutral; never let the handler throw.
        }
        message = "EXCEPTION, servos stopped: " + e;
        out.addLine(message);
        out.update();
    }

    // ------------------------------------------------------------------ baseline controller

    private static void applyGains(TurretBase turret, TurretDiagnosticSettings.GainSource source) {
        if (source == TurretDiagnosticSettings.GainSource.TUNER) {
            turret.setPositionalCoefficients(new TurretBasePIDFCoefficients(
                    TurretBaseTuner.KP_FAR, TurretBaseTuner.KP_CLOSE,
                    TurretBaseTuner.KI_FAR.clone(), TurretBaseTuner.KI_CLOSE.clone(),
                    TurretBaseTuner.KD_FAR, TurretBaseTuner.KD_CLOSE, TurretBaseTuner.KHOLD,
                    TurretBaseTuner.P_SWITCH, TurretBaseTuner.I_SWITCH.clone(), TurretBaseTuner.D_SWITCH,
                    TurretBaseTuner.KI_SMASH.clone(), TurretBaseTuner.D_ACTIVATION.clone(),
                    TurretBaseTuner.HOLD_DECAY, TurretBaseTuner.MIN_I, TurretBaseTuner.MAX_I));
            turret.setTuning(true);
            turret.setPdInterpolationMode(TurretBase.PD_INTERPOLATION_MODE.fromString(TurretBaseTuner.PD_MODE));
        } else {
            // Exactly what TeleOpBaseOpMode applies. Shared object: its tuning flag is not touched.
            turret.setPositionalCoefficients(TurretConstants.TURRET_POSITIONAL_COEFFICIENTS);
        }
    }

    private static boolean zeroBaselineGains() {
        TurretBasePIDFCoefficients c = TurretConstants.TURRET_POSITIONAL_COEFFICIENTS;
        boolean scalarsZero = c.kpFar == 0 && c.kpClose == 0 && c.kdFar == 0 && c.kdClose == 0 && c.kHold == 0
                && c.lkiFar == 0 && c.rkiFar == 0 && c.lkiClose == 0 && c.rkiClose == 0;
        return scalarsZero && allZero(TurretConstants.TURRET_KPS) && allZero(TurretConstants.TURRET_KDS)
                && allZero(TurretConstants.TURRET_KFS);
    }

    private static boolean allZero(List<Double> values) {
        for (Double v : values) if (v != 0) return false;
        return true;
    }

    private static List<String> describeGains(TurretDiagnosticSettings.GainSource source) {
        List<String> d = new ArrayList<>();
        d.add("gain_source=" + source);
        if (source == TurretDiagnosticSettings.GainSource.TUNER) {
            d.add(String.format(Locale.US, "tuner kp_far=%s kp_close=%s ki_far=%s ki_close=%s kd_far=%s kd_close=%s khold=%s "
                            + "p_switch=%s i_switch=%s d_switch=%s ki_smash=%s d_activation=%s hold_decay=%s min_i=%s max_i=%s pd_mode=%s",
                    TurretBaseTuner.KP_FAR, TurretBaseTuner.KP_CLOSE, Arrays.toString(TurretBaseTuner.KI_FAR),
                    Arrays.toString(TurretBaseTuner.KI_CLOSE), TurretBaseTuner.KD_FAR, TurretBaseTuner.KD_CLOSE,
                    TurretBaseTuner.KHOLD, TurretBaseTuner.P_SWITCH, Arrays.toString(TurretBaseTuner.I_SWITCH),
                    TurretBaseTuner.D_SWITCH, Arrays.toString(TurretBaseTuner.KI_SMASH),
                    Arrays.toString(TurretBaseTuner.D_ACTIVATION), TurretBaseTuner.HOLD_DECAY,
                    TurretBaseTuner.MIN_I, TurretBaseTuner.MAX_I, TurretBaseTuner.PD_MODE));
        } else {
            TurretBasePIDFCoefficients c = TurretConstants.TURRET_POSITIONAL_COEFFICIENTS;
            d.add(String.format(Locale.US, "constants kp_far=%s kp_close=%s ki_far=[%s, %s] ki_close=[%s, %s] kd_far=%s kd_close=%s "
                            + "khold=%s p_switch=%s i_switch=%s d_switch=%s ki_smash=[%s, %s] d_activation=%s hold_decay=%s min_i=%s max_i=%s",
                    c.kpFar, c.kpClose, c.lkiFar, c.rkiFar, c.lkiClose, c.rkiClose, c.kdFar, c.kdClose, c.kHold,
                    c.pSwitch, Arrays.toString(c.iSwitch), c.dSwitch, c.lkISmash, c.rkISmash,
                    Arrays.toString(c.dActivation), c.holdDecay, c.minI, c.maxI));
            d.add("turret_pd_positions=" + TurretConstants.TURRET_PD_POSITIONS + " kps=" + TurretConstants.TURRET_KPS
                    + " kds=" + TurretConstants.TURRET_KDS);
            d.add("turret_ff_positions=" + TurretConstants.TURRET_FEEDFORWARD_POSITIONS + " kfs=" + TurretConstants.TURRET_KFS);
        }
        d.add("note=TurretBase hold-decay uses ShooterConstants.TURRET_TICKS_PER_DEGREE=" + ShooterConstants.TURRET_TICKS_PER_DEGREE
                + " internally; the diagnostic's confirmed scale is used only for targets and recording");
        return d;
    }

    // ------------------------------------------------------------------ recording

    private void openSegment(DiagnosticActivity activity) {
        List<String> meta = new ArrayList<>();
        meta.add("version=" + VERSION);
        meta.add("selected_mode=" + selected.label);
        meta.add("plan=" + planName());
        meta.add("device_left=" + DeviceConstants.leftTurretCRServoDeviceName
                + " device_right=" + DeviceConstants.rightTurretCRServoDeviceName
                + " device_encoder=" + DeviceConstants.turretEncoderDeviceName);
        meta.add("servo_directions=" + Arrays.toString(TurretConstants.TURRET_BASE_DIRECTIONS)
                + " encoder_direction=" + TurretConstants.TURRET_ENCODER_DIRECTION + " (configured values, unchanged)");
        meta.add("documented_ticks_per_degree=" + TurretDiagnosticSettings.DOCUMENTED_TICKS_PER_DEGREE);
        meta.add("baseline_shooter_constants_ticks_per_degree=" + ShooterConstants.TURRET_TICKS_PER_DEGREE);
        meta.add("reference_ticks=" + (supervisor.isReferenceSet() ? String.valueOf(supervisor.referenceTicks()) : "not confirmed"));
        TurretDiagnosticSettings s = supervisor.settings();
        if (s == null) meta.add("settings=not confirmed");
        else meta.addAll(s.describe());
        if (activity == DiagnosticActivity.CONTROLLER && s != null) meta.addAll(describeGains(s.gainSource));
        meta.add("safety_note=software bounds are operator-entered and do not guarantee wiring safety");
        int capacity = s != null ? s.recordCapacityRows : TurretDiagnosticConfig.RECORD_CAPACITY_ROWS;
        if (capacity < 1000 || capacity > 100000) capacity = 20000;
        segment = new SegmentBuffer(activity, ++segmentSequence, activity == DiagnosticActivity.PASSIVE ? 0 : runId, capacity, meta);
        recorder.event(elapsedS(System.nanoTime()), "segment_open", activity, runId, "segment " + segmentSequence);
    }

    private void submitSegment() {
        if (segment == null) return;
        if (segment.rows() == 0) {
            segment = null;
            return;
        }
        TurretDiagnosticSettings s = supervisor.settings();
        recorder.submit(segment, s != null ? s.motionThresholdDeg : TurretDiagnosticConfig.MOTION_THRESHOLD_DEG);
        segment = null;
    }

    private void record(long now, double loopDtMs, int ticks, boolean readOk, double hubVel, double requested,
                        int stepIndex, int stepKind, long readNs, long writeNs) {
        if (!segment.beginRow()) {
            if (segment.activity == DiagnosticActivity.PASSIVE) {
                submitSegment();
                openSegment(DiagnosticActivity.PASSIVE);
                segment.beginRow();
            } else {
                endRun(StopReason.RECORDING_FULL, "", now);
                segment.beginRow();
            }
        }
        boolean running = supervisor.isRunning();
        DiagnosticActivity activity = running ? supervisor.armedActivity() : DiagnosticActivity.PASSIVE;

        segment.set(Col.T_S, elapsedS(now));
        segment.set(Col.LOOP_DT_MS, loopDtMs);
        segment.set(Col.ACTIVITY, activity.code);
        segment.set(Col.SELECTED_MODE, selected.code);
        segment.set(Col.PLAN, selected == DiagnosticActivity.OPEN_LOOP ? openLoopPlan
                : selected == DiagnosticActivity.CONTROLLER ? controllerPlan : 0);
        segment.set(Col.ARMED, supervisor.isArmed());
        segment.set(Col.DEADMAN, gamepad1.right_bumper);
        segment.set(Col.RUN_ID, running ? runId : 0);
        segment.set(Col.STEP_INDEX, stepIndex);
        segment.set(Col.STEP_KIND, stepKind);
        segment.set(Col.CMD_REQUESTED, requested);
        segment.set(Col.CMD_APPLIED_L, leftServo.everWritten() ? leftServo.applied() : Double.NaN);
        segment.set(Col.CMD_APPLIED_R, rightServo.everWritten() ? rightServo.applied() : Double.NaN);
        segment.set(Col.SATURATED, leftServo.saturated() || rightServo.saturated());
        if (readOk) {
            segment.set(Col.TICKS, ticks);
            segment.set(Col.VEL_HUB_TPS, hubVel);
            if (supervisor.isReferenceSet()) {
                double rel = ticks - supervisor.referenceTicks();
                segment.set(Col.REF_TICKS, supervisor.referenceTicks());
                segment.set(Col.POS_DEG_DOCUMENTED, rel / TurretDiagnosticSettings.DOCUMENTED_TICKS_PER_DEGREE);
                segment.set(Col.POS_DEG_BASELINE, rel / ShooterConstants.TURRET_TICKS_PER_DEGREE);
            }
            // Only from a fresh sample: a failed read leaves these cells empty, never stale.
            segment.set(Col.POS_DEG, supervisor.positionDeg());
            segment.set(Col.VEL_DEG_S, supervisor.velocityDegPerS());
            segment.set(Col.PRED_STOP_DEG, supervisor.predictedStopDeg());
        }
        segment.set(Col.BATTERY_V, batteryV);
        segment.set(Col.READ_US, readNs * 1e-3);
        segment.set(Col.WRITE_US, writeNs * 1e-3);
        if (controller != null && running) {
            segment.set(Col.CTRL_TARGET_TICKS, controllerTargetTicks);
            segment.set(Col.CTRL_ERROR_TICKS, controller.getError());
            segment.set(Col.CTRL_KP, controller.kp);
            segment.set(Col.CTRL_KI, controller.ki);
            segment.set(Col.CTRL_KD, controller.kd);
            segment.set(Col.CTRL_KHOLD, controller.kHold);
            segment.set(Col.CTRL_P, controller.p);
            segment.set(Col.CTRL_I, controller.i);
            segment.set(Col.CTRL_D, controller.d);
            segment.set(Col.CTRL_F, controller.f);
            segment.set(Col.CTRL_POWER, controller.getPower());
            segment.set(Col.CTRL_DT_MS, controller.getLoopDt() * 1e3);
        }
        segment.set(Col.EXTRA_DELAY_MS, lastExtraDelayMs);
        segment.set(Col.HWMON_LOOP_MS, HardwareMonitor.get().getLastLoopMs());
        segment.set(Col.STOP_CODE, pendingRowStop.code);
        pendingRowStop = StopReason.NONE;
        segment.commitRow();
    }

    // ------------------------------------------------------------------ telemetry

    private void showTelemetry(int ticks, boolean readOk, boolean afterStart) {
        TurretDiagnosticSettings s = supervisor.settings();
        String state = supervisor.isRunning() ? "RUNNING " + supervisor.armedActivity().label
                : supervisor.isArmed() ? "ARMED: hold RB to run (10 s)" : "DISARMED";
        out.addLine("TURRET DIAGNOSTIC: data collection only. Software bounds do not guarantee wiring safety.");
        out.addData("State", state);
        out.addData("Mode / plan", "%s / %s%s", selected.label, planName(), afterStart ? "" : " (init: arming disabled)");
        out.addData("Message", message);
        if (supervisor.lastStop().fault) {
            out.addData("Last fault", "%s %s", supervisor.lastStop().text, supervisor.lastStopDetail());
        }
        out.addData("Encoder", readOk ? String.valueOf(ticks) : "READ FAILED");
        if (supervisor.isReferenceSet()) {
            double rel = ticks - supervisor.referenceTicks();
            out.addData("From reference", "%.0f ticks | %.2f deg @68.2667 | %.2f deg @%.3f",
                    rel, rel / TurretDiagnosticSettings.DOCUMENTED_TICKS_PER_DEGREE,
                    rel / ShooterConstants.TURRET_TICKS_PER_DEGREE, ShooterConstants.TURRET_TICKS_PER_DEGREE);
        } else {
            out.addData("Reference", "NOT CONFIRMED (place turret at reference, LB + X)");
        }
        if (markerTicks != null) {
            double d = ticks - markerTicks;
            out.addData("Scale check", "%.0f ticks since marker: 90 deg -> %.3f t/deg, 180 deg -> %.3f t/deg",
                    d, Math.abs(d) / 90.0, Math.abs(d) / 180.0);
        }
        if (s == null) {
            out.addData("Settings", "NOT CONFIRMED (edit Dashboard, then LB + A)");
        } else {
            out.addData("Confirmed", "scale %.4f | bounds [%.1f, %.1f] margin %.1f | max cmd %.3f",
                    s.ticksPerDegree, s.minDeg, s.maxDeg, s.stopMarginDeg, s.maxCommand);
            out.addData("Position", "%.2f deg, %.1f deg/s, predicted stop %.2f",
                    supervisor.positionDeg(), supervisor.velocityDegPerS(), supervisor.predictedStopDeg());
            for (String w : s.warnings()) out.addData("Warning", w);
        }
        if (!supervisor.isArmed() && selected != DiagnosticActivity.PASSIVE) {
            String refusal = supervisor.armRefusal(selected, TurretDiagnosticConfig.snapshot(), gamepad1.right_bumper, System.nanoTime());
            out.addData("Ready to arm", refusal == null ? (afterStart ? "yes (LB + Y)" : "after START") : "no: " + refusal);
        }
        if (schedule != null && selected == DiagnosticActivity.OPEN_LOOP) {
            out.addData("Schedule", "step %d/%d (%.0f s total)%s", schedule.index() + 1, schedule.size(),
                    schedule.totalDurationS(), schedule.isComplete() ? " complete" : "");
        }
        out.addData("Command", "L %.3f R %.3f%s", leftServo.applied(), rightServo.applied(),
                leftServo.saturated() || rightServo.saturated() ? " SATURATED" : "");
        out.addData("Loop", "%.1f ms (worst recent %.1f ms)", supervisor.lastDtMs(), supervisor.maxRecentDtS() * 1e3);
        out.addData("Recording", "%s | written %d | pending %d | failures %d",
                recorder.directory().getName(), recorder.segmentsWritten(), recorder.pendingWrites(), recorder.writeFailures());
        out.addLine("RB hold=run  LB+Y arm  B/BACK cancel  dpad L/R mode  U/D plan  LB+A settings  LB+X reference  X marker");
        out.update();
    }

    // ------------------------------------------------------------------ helpers

    private String planName() {
        if (selected == DiagnosticActivity.OPEN_LOOP) return openLoopPlan == 0 ? "schedule" : "jog";
        if (selected == DiagnosticActivity.CONTROLLER) return controllerPlan == 0 ? "target schedule" : "hold reference";
        return "observe";
    }

    private double elapsedS(long nanos) {
        return (nanos - startNanos) * 1e-9;
    }

    private static void sleepMs(double ms) {
        try {
            Thread.sleep((long) ms, (int) ((ms - Math.floor(ms)) * 1e6));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
